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

#include "TEncAttribute.h"
#include "common/AttributePredictor.h"
#include "common/TComPointCloud.h"
#include <algorithm>
#include <vector>

using namespace std;

////Attribute predicting and compress
Void TEncAttribute::init(TComPointCloud* pointCloudOrg, TComPointCloud* pointCloudRecon,
                         HighLevelSyntax* hls, TEncBacTop* encBac) {
  m_pointCloudOrg = pointCloudOrg;
  m_pointCloudRecon = pointCloudRecon;
  m_pointCloudRecon->setNumPoint(m_pointCloudOrg->getNumPoint());
  m_hls = hls;
  m_encBac = encBac;

  m_reflectanceCache = new TComRingBuffer<std::pair<PC_POS, PC_REFL>>(ATTRIBUTE_CACHE_SIZE);
  std::pair<PC_POS, PC_REFL> tmpRefl = {0, 0};
  for (int i = 0; i < ATTRIBUTE_CACHE_SIZE; i++) {
    m_reflectanceCache->insertion(tmpRefl);
  }
}

// Attribute compression consists of the following stages:
//  - morton sorting
//  - prediction
//  - residual quantization
//  - entropy encode
//  - local decode
Void TEncAttribute::predictAndEncodeAttribute() {
  if (m_hls->aps.withColor) {
    ColorPredictingResidual();
  }
  if (m_hls->aps.withRef) {
    ReflectancePredictingResidual();
  }
}

void TEncAttribute::ReflectancePredictingResidual() {
  const SequenceParameterSet& sps = m_hls->sps;
  const AttributeParameterSet& aps = m_hls->aps;
  TComPointCloud& pointCloudRec = *m_pointCloudRecon;

  const int voxelCount = int(pointCloudRec.getNumPoint());

  // Reorder
  std::vector<pointMortonCodeWithIndex> pointCloudMorton(voxelCount);
  std::vector<Int> mortonToindex(voxelCount);
  std::vector<Int> mortonTomorton2(voxelCount);
  const Int shift = 42;
  vector<PC_POS_INT> positions(voxelCount);
  if (!sps.attrAdaptPred && aps.numOflevelOfDetail == 0) {
    for (UInt n = 0; n < voxelCount; n++) {
      const auto& point = pointCloudRec[n];
      positions[n][0] = (int32_t)point[0];
      positions[n][1] = (int32_t)point[1];
      positions[n][2] = (int32_t)point[2];
      HilbertAddr(positions[n][0], positions[n][1], positions[n][2],
                  pointCloudMorton[n].mortonCode);
      pointCloudMorton[n].index = n;
    }
    std::sort(pointCloudMorton.begin(), pointCloudMorton.end());
  } else {
    sortReconPoints(pointCloudRec, 0, voxelCount, pointCloudMorton);
  }

  //Build LoD Layers
  std::vector<uint32_t> numberOfPointsPerLayer;
  std::vector<uint32_t> layerToindex;
  levelOfDetailLayeringStructure(aps, pointCloudRec, layerToindex, numberOfPointsPerLayer,
                                 pointCloudMorton);

  std::vector<pointMortonCodeWithIndex> pointCloudIntraMorton;
  //obtain the reflectance predictor
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
        sortReconPointsShift(pointCloudRec, shift, 0, numberOfPointsPerLayer[i],
                             pointCloudIntraMorton, mortonToindex, mortonTomorton2);
      }
    }
    for (int idx = 0; idx < numberOfPointsPerLayer[i]; ++idx) {
      //determine whether to use adaptive decision
      adaptivePredDecision(sps, m_encBac->getForceAttrBasedPred(), m_encBac->getResidualStats());
      auto pointIndex = pointCloudMorton[layerToindex[idx + numberOfPointsIndex]].index;
      PC_REFL& curReflectance = pointCloudRec.getReflectance(pointIndex);
      const PC_POS& curPosition = pointCloudRec[pointIndex];
      int64_t predictorRefl;

      if (!sps.attrAdaptPred && aps.numOflevelOfDetail == 0) {
        predictorRefl =
          getReflectancePredictor(idx, pointCloudRec, sps, pointCloudIntraMorton, positions);
      } else {
        if (m_encBac->getForceAttrBasedPred()) {
          int32_t min_idx = getReflectanceCacheIndex(curReflectance);
          m_encBac->encodeAttrCacheIndex(min_idx);
          predictorRefl = (*m_reflectanceCache)[min_idx].second;
        } else {
          predictorRefl = getReflectancePredictor(idx, pointCloudRec, sps, pointCloudIntraMorton,
                                                  mortonToindex, mortonTomorton2);
        }
      }
      reflectancePredictCode(predictorRefl, curReflectance);

      //insert the reconstructed attribue value into Cache
      if (!m_encBac->getResidualStats().collectionDone() || m_encBac->getForceAttrBasedPred()) {
        m_reflectanceCache->insertion({curPosition, curReflectance}, false);
      }
    }
    numberOfPointsIndex += numberOfPointsPerLayer[i];
  }
}

int32_t TEncAttribute::getReflectanceCacheIndex(PC_REFL& curReflectance) {
  //check the reflectance stored in cache and choose the best one as predictor
  int32_t min_idx = -1;
  int32_t cacheSize = m_reflectanceCache->getSize();
  int32_t min_distance = std::numeric_limits<int32_t>::max();
  for (int i = 0; i < cacheSize; i++) {
    int32_t curr_distance = std::abs(curReflectance - (*m_reflectanceCache)[i].second);
    if (curr_distance < min_distance) {
      min_distance = curr_distance;
      min_idx = i;
    }
  }
  return min_idx;
}

void TEncAttribute::ColorPredictingResidual() {
  const SequenceParameterSet& sps = m_hls->sps;
  const AttributeParameterSet& aps = m_hls->aps;
  TComPointCloud& pointCloudRec = *m_pointCloudRecon;

  const int voxelCount = int(pointCloudRec.getNumPoint());
  // Morton sorting
  std::vector<pointMortonCodeWithIndex> pointCloudMorton(voxelCount);
  sortReconPoints(pointCloudRec, 0, voxelCount, pointCloudMorton);
  //Build LoD Layers
  std::vector<uint32_t> numberOfPointsPerLayer;
  std::vector<uint32_t> layerToindex;
  levelOfDetailLayeringStructure(aps, pointCloudRec, layerToindex, numberOfPointsPerLayer,
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
      PC_COL& curColor = pointCloudRec.getColor(pointIndex);
      PC_COL predictorColor =
        getColorPredictor(idx, pointCloudRec, sps, aps, pointCloudIntraMorton);
      colorPredictCode(predictorColor, curColor);
    }
    numberOfPointsIndex += numberOfPointsPerLayer[i];
  }
}

void TEncAttribute::colorPredictCode(const PC_COL& predictor, PC_COL& currValue) {
  const SequenceParameterSet& sps = m_hls->sps;
  const AttributeParameterSet& aps = m_hls->aps;

  int residualPrevComponent = 0;
  bool ccp = aps.crossComponentPred;

  for (int i = 0; i < 3; i++) {
    int64_t delta = currValue[i] - predictor[i];
    delta -= residualPrevComponent;

    // Residual quantization
    int residualSign = delta < 0 ? -1 : 1;
    uint64_t absResidual = std::abs(delta);
    uint64_t residualQuant = QuantizaResidual(absResidual, sps.attrQuantParam);
    int64_t signResidualQuant = residualSign * (int64_t)residualQuant;

    // Entropy encode
    m_encBac->codeAttributerResidual(signResidualQuant, i);

    // Local decode
    uint64_t inverseResidualQuant = InverseQuantizeResidual(residualQuant, sps.attrQuantParam);
    int64_t recResidual = inverseResidualQuant * residualSign;
    currValue[i] =
      TComClip((Int64)0, (Int64)255, recResidual + predictor[i] + residualPrevComponent);
    if (ccp && i <= 1) {
      residualPrevComponent = (Int)currValue[i] - (Int)predictor[i];
    }
  }
}

void TEncAttribute::reflectancePredictCode(const int64_t& predictor, PC_REFL& currValue) {
  const SequenceParameterSet& sps = m_hls->sps;
  int64_t delta = currValue - predictor;

  // Residual quantization
  int residualSign = delta < 0 ? -1 : 1;
  uint64_t absResidual = std::abs(delta);
  uint64_t residualQuant = QuantizaResidual(absResidual, sps.attrQuantParam);
  int64_t signResidualQuant = residualSign * (int64_t)residualQuant;

  // Entropy encode
  m_encBac->codeAttributerResidual(signResidualQuant);

  // Local decode
  uint64_t inverseResidualQuant = InverseQuantizeResidual(residualQuant, sps.attrQuantParam);
  int64_t recResidual = inverseResidualQuant * residualSign;
  currValue = TComClip((Int64)INT16_MIN, (Int64)INT16_MAX, recResidual + predictor);
}
