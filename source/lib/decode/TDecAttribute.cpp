/* The copyright in this software is being made available under the BSD
* License, included below. This software may be subject to other third party
* and contributor rights, including patent rights, and no such rights are
* granted under this license.
*
* Copyright (c) 2019-2033, Audio Video coding Standard Workgroup of China
* All rights reserved.
*
* Redistribution and use in source and binary forms, with or without
* modification, are permitted provided that the following conditions are met:
*
*  * Redistributions of source code must retain the above copyright notice,
*    this list of conditions and the following disclaimer.
*  * Redistributions in binary form must reproduce the above copyright notice,
*    this list of conditions and the following disclaimer in the documentation
*    and/or other materials provided with the distribution.
*  * Neither the name of Audio Video coding Standard Workgroup of China
*    nor the names of its contributors maybe used to endorse or promote products
*    derived from this software without
*    specific prior written permission.
*
* THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
* AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
* IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
* ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS
* BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
* CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
* SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
* INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
* CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
* ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF
* THE POSSIBILITY OF SUCH DAMAGE.
*/

#include "TDecAttribute.h"
#include "common/AttributePredictor.h"
#include <algorithm>
#include <vector>

Void TDecAttribute::init(TComPointCloud* pointCloudRecon, HighLevelSyntax* hls,
                         TDecBacTop* decBac) {
  m_pointCloudRecon = pointCloudRecon;
  m_hls = hls;
  m_decBac = decBac;

  m_reflectanceCache = new TComRingBuffer<std::pair<PC_POS, PC_REFL>>(ATTRIBUTE_CACHE_SIZE);
  std::pair<PC_POS, PC_REFL> tmpRefl = {0, 0};
  for (int i = 0; i < ATTRIBUTE_CACHE_SIZE; i++) {
    m_reflectanceCache->insertion(tmpRefl);
  }
}

// Attribute decompression consists of the following stages:
//  - Morton sorting
//  - Entropy decode
//  - Inverse quantization
//  - Attribute reconstruction
Void TDecAttribute::decodeAttribute() {
  if (m_hls->aps.withColor)
    ColorInversePredictingResidual();
  if (m_hls->aps.withRef)
    ReflectanceInversePredictingResidual();
}

void TDecAttribute::ReflectanceInversePredictingResidual() {
  const SequenceParameterSet& sps = m_hls->sps;
  const AttributeParameterSet& aps = m_hls->aps;
  TComPointCloud& outputPointCloud = *m_pointCloudRecon;

  const int voxelCount = int(outputPointCloud.getNumPoint());
  outputPointCloud.addReflectances();

  // Reorder
  std::vector<pointMortonCodeWithIndex> pointCloudMorton(voxelCount);
  std::vector<Int> mortonToindex(voxelCount);
  std::vector<Int> mortonTomorton2(voxelCount);
  const Int shift = 42;
  vector<PC_POS_INT> positions(voxelCount);
  if (!sps.attrAdaptPred && aps.numOflevelOfDetail == 0) {
    for (UInt n = 0; n < voxelCount; n++) {
      const auto& point = outputPointCloud[n];
      positions[n][0] = (int32_t)point[0];
      positions[n][1] = (int32_t)point[1];
      positions[n][2] = (int32_t)point[2];
      HilbertAddr(positions[n][0], positions[n][1], positions[n][2],
                  pointCloudMorton[n].mortonCode);
      pointCloudMorton[n].index = n;
    }
    std::sort(pointCloudMorton.begin(), pointCloudMorton.end());
  } else {
    sortReconPoints(outputPointCloud, 0, voxelCount, pointCloudMorton);
  }

  //Build LoD Layers
  std::vector<uint32_t> numberOfPointsPerLayer;
  std::vector<uint32_t> layerToindex;
  levelOfDetailLayeringStructure(aps, outputPointCloud, layerToindex, numberOfPointsPerLayer,
                                 pointCloudMorton);

  std::vector<pointMortonCodeWithIndex> pointCloudIntraMorton;
  UInt numberOfPointsIndex = 0;
  for (UInt i = 1; i < numberOfPointsPerLayer.size(); ++i) {
    if (aps.intraLodFlag) {
      if (aps.numOflevelOfDetail == 0)
        pointCloudIntraMorton = pointCloudMorton;
      else {
        pointCloudIntraMorton.resize(0);
        for (UInt j = numberOfPointsIndex; j < numberOfPointsPerLayer[i] + numberOfPointsIndex; ++j)
          pointCloudIntraMorton.push_back(pointCloudMorton[layerToindex[j]]);
      }

      if (!(!sps.attrAdaptPred && aps.numOflevelOfDetail == 0)) {
        sortReconPointsShift(outputPointCloud, shift, 0, numberOfPointsPerLayer[i],
                             pointCloudIntraMorton, mortonToindex, mortonTomorton2);
      }
    }
    for (int idx = 0; idx < numberOfPointsPerLayer[i]; ++idx) {
      //determine whether to use adaptive decision
      adaptivePredDecision(sps, m_decBac->getForceAttrBasedPred(), m_decBac->getResidualStats());
      auto pointIndex = pointCloudMorton[layerToindex[idx + numberOfPointsIndex]].index;
      PC_REFL& currentValue = outputPointCloud.getReflectance(pointIndex);
      const PC_POS curPosition = outputPointCloud[pointIndex];
      int64_t predictorRefl;
      int64_t codedValue;

      if (!sps.attrAdaptPred && aps.numOflevelOfDetail == 0) {
        // Entropy decode
        codedValue = m_decBac->parseAttr();
        predictorRefl =
          getReflectancePredictor(idx, outputPointCloud, sps, pointCloudIntraMorton, positions);
      } else {
        if (m_decBac->getForceAttrBasedPred()) {
          //obtain the predictor
          int32_t min_idx = m_decBac->parseAttrCacheIndex();
          predictorRefl = (*m_reflectanceCache)[min_idx].second;
          // Entropy decode
          codedValue = m_decBac->parseAttr();
        } else {
          // Entropy decode
          codedValue = m_decBac->parseAttr();
          predictorRefl = getReflectancePredictor(idx, outputPointCloud, sps, pointCloudIntraMorton,
                                                  mortonToindex, mortonTomorton2);
        }
      }

      reflectanceReconstruction(predictorRefl, codedValue, currentValue);
      if (!m_decBac->getResidualStats().collectionDone() || m_decBac->getForceAttrBasedPred()) {
        m_reflectanceCache->insertion({curPosition, currentValue}, false);
      }
    }
    numberOfPointsIndex += numberOfPointsPerLayer[i];
  }
}

void TDecAttribute::ColorInversePredictingResidual() {
  const SequenceParameterSet& sps = m_hls->sps;
  const AttributeParameterSet& aps = m_hls->aps;
  TComPointCloud& outputPointCloud = *m_pointCloudRecon;

  const int voxelCount = int(outputPointCloud.getNumPoint());
  outputPointCloud.addColors();
  // Morton code
  std::vector<pointMortonCodeWithIndex> pointCloudMorton(voxelCount);
  sortReconPoints(outputPointCloud, 0, voxelCount, pointCloudMorton);

  //Build LoD Layers
  std::vector<uint32_t> numberOfPointsPerLayer;
  std::vector<uint32_t> layerToindex;
  levelOfDetailLayeringStructure(aps, outputPointCloud, layerToindex, numberOfPointsPerLayer,
                                 pointCloudMorton);
  std::vector<pointMortonCodeWithIndex> pointCloudIntraMorton;
  UInt numberOfPointsIndex = 0;
  for (UInt i = 1; i < numberOfPointsPerLayer.size(); ++i) {
    if (aps.intraLodFlag) {
      if (aps.numOflevelOfDetail == 0)
        pointCloudIntraMorton = pointCloudMorton;
      else {
        pointCloudIntraMorton.resize(0);
        for (UInt j = numberOfPointsIndex; j < numberOfPointsPerLayer[i] + numberOfPointsIndex; ++j)
          pointCloudIntraMorton.push_back(pointCloudMorton[layerToindex[j]]);
      }
    }
    for (int idx = 0; idx < numberOfPointsPerLayer[i]; ++idx) {
      auto pointIndex = pointCloudMorton[layerToindex[idx + numberOfPointsIndex]].index;
      PC_COL& currentValue = outputPointCloud.getColor(pointIndex);
      V3<int64_t> codedValue;
      for (int i = 0; i < 3; i++) {
        codedValue[i] = m_decBac->parseAttr(i);
      }
      PC_COL predictorColor =
        getColorPredictor(idx, outputPointCloud, sps, aps, pointCloudIntraMorton);
      colorReconstruction(predictorColor, codedValue, currentValue);
    }
    numberOfPointsIndex += numberOfPointsPerLayer[i];
  }
}

void TDecAttribute::reflectanceReconstruction(const int64_t& predictor, const int64_t& codedValue,
                                              PC_REFL& reconValue) {
  int64_t delta = codedValue;
  int sign = (delta < 0) ? -1 : 1;
  uint64_t absDelta = std::abs(delta);

  //inverse quantitzation
  uint64_t inverseResidualQuant = InverseQuantizeResidual(absDelta, m_hls->sps.attrQuantParam);
  int64_t residual = inverseResidualQuant * sign;
  reconValue = TComClip((Int64)INT16_MIN, (Int64)INT16_MAX, residual + predictor);
}

void TDecAttribute::colorReconstruction(const PC_COL& predictor, const V3<int64_t>& codedValue,
                                        PC_COL& reconValue) {
  int residualPrevComponent = 0;
  bool ccp = m_hls->aps.crossComponentPred;
  for (int i = 0; i < 3; i++) {
    int64_t delta = codedValue[i];
    int sign = (delta < 0) ? -1 : 1;
    uint64_t absDelta = std::abs(delta);

    //inverse quantitzation
    uint64_t inverseResidualQuant = InverseQuantizeResidual(absDelta, m_hls->sps.attrQuantParam);
    int64_t residual = inverseResidualQuant * sign;
    reconValue[i] = TComClip((Int64)0, (Int64)255, residual + predictor[i] + residualPrevComponent);
    if (ccp && i <= 1) {
      residualPrevComponent = (Int)reconValue[i] - (Int)predictor[i];
    }
  }
}
