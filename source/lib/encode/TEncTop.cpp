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

#include "TEncTop.h"
#include "common/CommonDef.h"
#include "common/MD5Sum.h"
#include "common/TComBufferChunk.h"
#include "common/TComPointCloud.h"
#include "common/contributors.h"
#include <fstream>
#include <map>
#include <math.h>
#include <set>
#include <time.h>

using namespace std;

///< \in TLibEncoder \{
/**
 * Implementation of TEncTop
 * encoder class
 */

//////////////////////////////////////////////////////////////////////////
// Public class functions
//////////////////////////////////////////////////////////////////////////

Int TEncTop::encode() {
  Int numDigits = getfileNumLength();  // numDigit indicates digits of file number
  setStartFrameNum(numDigits);         // set start frame name from m_inputFileName

  Int exitState = EXIT_SUCCESS;

  MD5Sum md5Calculator;
  bool md5FileOpenFlag = md5Calculator.openOStream(m_MD5FileName);

  ///Print information
  UInt64 geomBits = 0;
  UInt64 attrBits = 0;
  UInt64 totalBits = 0;
  Double totalUserTime = 0.0;
  UInt64 totalNumReconPoints = 0;

  ///Pc_evalue
  pc_evalue::TMetricRes totalMetricRes;
  pc_evalue::TMetricCfg MetricParam;
  MetricParam.m_calColor = m_calColor;
  MetricParam.m_calReflectance = m_calReflectance;
  MetricParam.m_peakValue = m_peakValue;
  MetricParam.m_symmetry = m_symmetry;
  MetricParam.m_duplicateMode = m_duplicateMode;
  MetricParam.m_multiNeighbourMode = m_multiNeighbourMode;
  MetricParam.m_showHausdorff = m_showHausdorff;

  for (UInt i_frame = 0; i_frame < m_numOfFrames; i_frame++) {
    printFrameCutoffRule(i_frame, numDigits);

    if (i_frame == 0) {
      resetBitstream_Recon_FileName(m_startFrame,
                                    numDigits);  // reset m_bitstreamFileName & m_reconFileName
    } else {
      updateIOFileName(
        m_startFrame + i_frame,
        numDigits);  // update m_inputFileName & m_bitstreamFileName & m_reconFileName
    }

    if (!m_pointCloudOrg.readFromFile(m_inputFileName, m_geomOnlyFlag) ||
        m_pointCloudOrg.getNumPoint() == 0) {
      cerr << "Error: failed to open ply file: " << m_inputFileName << " !" << endl;
      exitState |= EXIT_FAILURE;
      continue;  // if can not open the corresponding file then skip and move on to find next input file
    }

    ofstream bitstreamFile(m_bitstreamFileName, fstream::binary | fstream::out);
    if (!bitstreamFile) {
      cerr << "Error: failed to open bitstream file " << m_bitstreamFileName << " for writing!"
           << endl;
      exitState |= EXIT_FAILURE;
      continue;  // if can not open the corresponding file then skip and move on to find next input file
    }

    Double userTimeTotal = 0.0, userTimeGeometry = 0.0, userTimeAttribute = 0.0,
           userTimeRecolor = 0.0;
    clock_t userTimeTotalBegin = clock();

    ///< preprocessing
    vector<PC_COL> pointCloudOrgColors = m_pointCloudOrg.getColors();
    if (m_colorTransformFlag && m_pointCloudOrg.hasColors()) {
      m_pointCloudOrg.convertRGBToYUV();
    }
    ///< determine bounding box
    PC_POS bbMin, bbMax, bbSize;
    m_pointCloudOrg.computeBoundingBox(bbMin, bbMax);
    for (Int k = 0; k < 3; k++) {
      bbMin[k] = floor(bbMin[k]);
      m_hls.sps.geomBoundingBoxOrigin[k] = Int(bbMin[k]);
    }
    bbSize = bbMax - bbMin;
    m_hls.sps.geomBoundingBoxSize[0] = Int(round(bbSize[0] / m_hls.sps.geomQuantStep)) + 1;
    m_hls.sps.geomBoundingBoxSize[1] = Int(round(bbSize[1] / m_hls.sps.geomQuantStep)) + 1;
    m_hls.sps.geomBoundingBoxSize[2] = Int(round(bbSize[2] / m_hls.sps.geomQuantStep)) + 1;

    initParameters();
    init_aec_context_tab();

    ///< encoding sequence parameter set
    TComBufferChunk bufferChunk(BufferChunkType::BCT_SPS);
    m_encBac.setBitstreamBuffer(bufferChunk);
    m_encBac.codeSPS(m_hls.sps);
    m_encBac.encodeFinish();
    bufferChunk.writeToBitstream(&bitstreamFile, m_encBac.getBitStreamLength());
    bufferChunk.reset();
    m_encBac.reset();

    ///< encoding geometry parameter set
    bufferChunk.setBufferType(BufferChunkType::BCT_GPS);
    m_encBac.setBitstreamBuffer(bufferChunk);
    m_encBac.codeGPS(m_hls.gps);
    m_encBac.encodeFinish();
    bufferChunk.writeToBitstream(&bitstreamFile, m_encBac.getBitStreamLength());
    bufferChunk.reset();
    m_encBac.reset();

    ///< encoding attribute parameter set
    auto& aps = m_hls.aps;
    if (m_pointCloudOrg.hasColors()) {
      aps.withColor = true;
    }
    if (m_pointCloudOrg.hasReflectances()) {
      aps.withRef = true;
    }
    bufferChunk.setBufferType(BufferChunkType::BCT_APS);
    m_encBac.setBitstreamBuffer(bufferChunk);
    m_encBac.codeAPS(m_hls.aps);
    m_encBac.encodeFinish();
    bufferChunk.writeToBitstream(&bitstreamFile, m_encBac.getBitStreamLength());
    bufferChunk.reset();
    m_encBac.reset();
    if (m_numOfSlices == 1) {
      SliceOri = {0, 0, 0};
      SliceID = 0;
      SliceSideLth = m_hls.sps.geomBoundingBoxSize;

      geomPreprocessAndQuantization(m_hls.sps.geomQuantStep, userTimeRecolor);

      UInt maxBB = std::max({1U, SliceSideLth[0], SliceSideLth[1], SliceSideLth[2]});

      ///< node size (log2) for xyz dimensions
      V3<UInt> nodeSizeLog2;
      if (m_hls.gps.im_qtbt_flag) {
        nodeSizeLog2[0] = ceilLog2(SliceSideLth[0]);
        nodeSizeLog2[1] = ceilLog2(SliceSideLth[1]);
        nodeSizeLog2[2] = ceilLog2(SliceSideLth[2]);
      } else {
        nodeSizeLog2 = ceilLog2(maxBB);
      }

      ///<encoding geometry brick header
      bufferChunk.setBufferType(BufferChunkType::BCT_GBH);
      m_encBac.setBitstreamBuffer(bufferChunk);
      m_hls.gbh.sliceID = SliceID;
      m_hls.gbh.geomBoundingBoxOrigin = SliceOri;
      m_hls.gbh.nodeSizeLog2 = nodeSizeLog2;
      m_hls.gbh.geomNumPoints = (UInt)m_pointCloudQuant.getNumPoint();
      m_encBac.codeGBH(m_hls.gbh);
      m_encBac.encodeFinish();
      bufferChunk.writeToBitstream(&bitstreamFile, m_encBac.getBitStreamLength());
      bufferChunk.reset();
      m_encBac.reset();

      ///< start geometry coding
      clock_t userTimeGeometryBegin = clock();
      bufferChunk.setBufferType(BufferChunkType::BCT_GEOM);
      m_encBac.setBitstreamBuffer(bufferChunk);
      m_encBac.initBac();
      compressAndEncodePartition();
      m_encBac.encodeTerminationFlag();
      m_encBac.encodeFinish();
      bufferChunk.writeToBitstream(&bitstreamFile, m_encBac.getBitStreamLength());
      cout << "Number of output points: " << m_pointCloudRecon.getNumPoint() << endl;
      cout << "Geometry bits: " << m_encBac.getBitStreamLength() * 8 << " bits." << endl;
      cout << "Geometry bpp: "
           << (Double)(m_encBac.getBitStreamLength()) * 8 / m_pointCloudOrg.getNumPoint() << " bpp."
           << endl;
      geomBits += m_encBac.getBitStreamLength() * 8;
      bufferChunk.reset();
      m_encBac.reset();
      userTimeGeometry = (Double)(clock() - userTimeGeometryBegin) / CLOCKS_PER_SEC;
      totalNumReconPoints += m_pointCloudRecon.getNumPoint();

      ///< recolor
      if (m_hls.sps.geomRemoveDuplicateFlag && m_hls.sps.recolorMode == 0) {
        clock_t userTimeRecolorBegin = clock();
        recolour(m_pointCloudOrg, float(1.0 / m_hls.sps.geomQuantStep),
                 m_hls.sps.geomBoundingBoxOrigin, &m_pointCloudRecon);
        userTimeRecolor = (Double)(clock() - userTimeRecolorBegin) / CLOCKS_PER_SEC;
      }

      //encoding attribute brick header
      bufferChunk.setBufferType(BufferChunkType::BCT_ABH);
      m_encBac.setBitstreamBuffer(bufferChunk);
      m_hls.abh.sliceID = SliceID;
      m_encBac.codeABH(m_hls.abh);
      m_encBac.encodeFinish();
      bufferChunk.writeToBitstream(&bitstreamFile, m_encBac.getBitStreamLength());
      bufferChunk.reset();
      m_encBac.reset();

      ///< start attribute coding
      clock_t userTimeAttributeBegin = clock();
      bufferChunk.setBufferType(BufferChunkType::BCT_ATTR);
      m_encBac.setBitstreamBuffer(bufferChunk);
      m_encBac.initBac();
      compressAndEncodeAttribute();
      m_encBac.encodeTerminationFlag();
      m_encBac.encodeFinish();
      bufferChunk.writeToBitstream(&bitstreamFile, m_encBac.getBitStreamLength());
      cout << "Attributes bits: " << m_encBac.getBitStreamLength() * 8 << " bits." << endl;
      cout << "Attributes bpp: "
           << (Double)(m_encBac.getBitStreamLength()) * 8 / m_pointCloudOrg.getNumPoint() << " bpp."
           << endl;
      attrBits += m_encBac.getBitStreamLength() * 8;
      bufferChunk.reset();
      m_encBac.reset();
      userTimeAttribute = (Double)(clock() - userTimeAttributeBegin) / CLOCKS_PER_SEC;
    } else {
      geomPreprocessAndQuantizationWithNoduplicate(m_hls.sps.geomQuantStep, &m_pointCloudOrg);
      TComPointCloud pointCloudOrg = m_pointCloudOrg;
      TComPointCloud reconstructionCloud;
      reconstructionCloud.setNumPoint(0);

      //Slice division
      std::vector<SliceRegion> slice;
      slice = SliceDevisionByMortonCode(m_numOfSlices);

      std::cout << "Slice number: " << slice.size() << std::endl;

      for (const auto& region : slice) {
        int regionPointNum = region.indexes.size();
        m_pointCloudOrg.init(regionPointNum, pointCloudOrg.hasColors(),
                             pointCloudOrg.hasReflectances());

        V3<Int> sliceMin = {INT32_MAX, INT32_MAX, INT32_MAX}, sliceMax = {0, 0, 0};

        for (int i = 0; i < regionPointNum; i++) {
          int index = region.indexes[i];
          m_pointCloudOrg[i] = pointCloudOrg[index];

          if (pointCloudOrg.hasColors()) {
            m_pointCloudOrg.setColor(i, pointCloudOrg.getColor(index));
          }

          if (pointCloudOrg.hasReflectances()) {
            m_pointCloudOrg.setReflectance(i, pointCloudOrg.getReflectance(index));
          }

          for (int k = 0; k < 3; k++) {
            if (sliceMin[k] > pointCloudOrg[index][k])
              sliceMin[k] = pointCloudOrg[index][k];
            if (sliceMax[k] < pointCloudOrg[index][k])
              sliceMax[k] = pointCloudOrg[index][k];
          }
        }
        SliceID = region.slice_id;
        for (int k = 0; k < 3; k++) {
          SliceOri[k] =
            round((sliceMin[k] - m_hls.sps.geomBoundingBoxOrigin[k]) / m_hls.sps.geomQuantStep);
          SliceSideLth[k] =
            round((sliceMax[k] - m_hls.sps.geomBoundingBoxOrigin[k]) / m_hls.sps.geomQuantStep) -
            SliceOri[k] + 1;
        }
        UInt maxBB = std::max({1U, SliceSideLth[0], SliceSideLth[1], SliceSideLth[2]});

        ///< node size (log2) for xyz dimensions
        V3<UInt> nodeSizeLog2;
        if (m_hls.gps.im_qtbt_flag) {
          nodeSizeLog2[0] = ceilLog2(SliceSideLth[0]);
          nodeSizeLog2[1] = ceilLog2(SliceSideLth[1]);
          nodeSizeLog2[2] = ceilLog2(SliceSideLth[2]);
        } else {
          nodeSizeLog2 = ceilLog2(maxBB);
        }

        geomPreprocessAndQuantization(m_hls.sps.geomQuantStep, userTimeRecolor);

        m_hls.gbh.sliceID = SliceID;
        m_hls.gbh.geomBoundingBoxOrigin = SliceOri;
        m_hls.gbh.nodeSizeLog2 = nodeSizeLog2;
        m_hls.gbh.geomNumPoints = (UInt)m_pointCloudQuant.getNumPoint();

        ///<encoding geometry brick header
        bufferChunk.setBufferType(BufferChunkType::BCT_GBH);
        m_encBac.setBitstreamBuffer(bufferChunk);
        m_encBac.codeGBH(m_hls.gbh);
        m_encBac.encodeFinish();
        bufferChunk.writeToBitstream(&bitstreamFile, m_encBac.getBitStreamLength());
        bufferChunk.reset();
        m_encBac.reset();

        ///< start geometry coding
        clock_t userTimeGeometryBegin = clock();
        bufferChunk.setBufferType(BufferChunkType::BCT_GEOM);
        m_encBac.setBitstreamBuffer(bufferChunk);
        m_encBac.initBac();
        compressAndEncodePartition();
        m_encBac.encodeTerminationFlag();
        m_encBac.encodeFinish();
        bufferChunk.writeToBitstream(&bitstreamFile, m_encBac.getBitStreamLength());
        cout << "Number of output points: " << m_pointCloudRecon.getNumPoint() << endl;
        cout << "Geometry bits: " << m_encBac.getBitStreamLength() * 8 << " bits." << endl;
        cout << "Geometry bpp: "
             << (Double)(m_encBac.getBitStreamLength()) * 8 / m_pointCloudOrg.getNumPoint()
             << " bpp." << endl;
        geomBits += m_encBac.getBitStreamLength() * 8;
        bufferChunk.reset();
        m_encBac.reset();
        userTimeGeometry = (Double)(clock() - userTimeGeometryBegin) / CLOCKS_PER_SEC;
        totalNumReconPoints += m_pointCloudRecon.getNumPoint();

        ///< recolor
        if (m_hls.sps.geomRemoveDuplicateFlag && m_hls.sps.recolorMode == 0) {
          clock_t userTimeRecolorBegin = clock();
          recolour(m_pointCloudOrg, float(1.0 / m_hls.sps.geomQuantStep),
                   m_hls.sps.geomBoundingBoxOrigin, &m_pointCloudRecon);
          userTimeRecolor = (Double)(clock() - userTimeRecolorBegin) / CLOCKS_PER_SEC;
        }

        m_hls.abh.sliceID = SliceID;
        //encoding attribute brick header
        bufferChunk.setBufferType(BufferChunkType::BCT_ABH);
        m_encBac.setBitstreamBuffer(bufferChunk);
        m_encBac.codeABH(m_hls.abh);
        m_encBac.encodeFinish();
        bufferChunk.writeToBitstream(&bitstreamFile, m_encBac.getBitStreamLength());
        bufferChunk.reset();
        m_encBac.reset();

        ///< start attribute coding
        clock_t userTimeAttributeBegin = clock();
        bufferChunk.setBufferType(BufferChunkType::BCT_ATTR);
        m_encBac.setBitstreamBuffer(bufferChunk);
        m_encBac.initBac();
        compressAndEncodeAttribute();
        m_encBac.encodeTerminationFlag();
        m_encBac.encodeFinish();
        bufferChunk.writeToBitstream(&bitstreamFile, m_encBac.getBitStreamLength());
        cout << "Attributes bits: " << m_encBac.getBitStreamLength() * 8 << " bits." << endl;
        cout << "Attributes bpp: "
             << (Double)(m_encBac.getBitStreamLength()) * 8 / m_pointCloudOrg.getNumPoint()
             << " bpp." << endl;
        attrBits += m_encBac.getBitStreamLength() * 8;
        bufferChunk.reset();
        m_encBac.reset();
        userTimeAttribute = (Double)(clock() - userTimeAttributeBegin) / CLOCKS_PER_SEC;

        addToReconstructionCloud(&reconstructionCloud);
      }
      m_pointCloudRecon = reconstructionCloud;
      m_pointCloudOrg = pointCloudOrg;
    }
    std::cout << "Total bitstream size " << bitstreamFile.tellp() * 8 << " bits" << std::endl;
    totalBits += bitstreamFile.tellp() * 8;
    bitstreamFile.close();

    userTimeTotal = (Double)(clock() - userTimeTotalBegin) / CLOCKS_PER_SEC;
    cout << "Geometry processing time (user): " << userTimeGeometry << " sec." << endl;
    cout << "Recolor processing time (user): " << userTimeRecolor << " sec." << endl;
    cout << "Attribute processing time (user): " << userTimeAttribute << " sec." << endl;
    cout << "Total processing time (user): " << userTimeTotal << " sec." << endl << endl;
    totalUserTime += userTimeTotal;

    geomPostprocessingAndDequantization(m_hls.sps.geomQuantStep);
    if (m_colorTransformFlag && m_pointCloudRecon.hasColors()) {
      m_pointCloudRecon.convertYUVToRGB();
    }

    if (md5FileOpenFlag) {
      md5Calculator.calculateMD5(&m_pointCloudRecon);
      md5Calculator.writeToFile();
      cout << endl;
    }

    ///< write recon ply
    if (m_reconFileName.length() > 0)
      m_pointCloudRecon.writeToFile(m_reconFileName, m_writePlyInAsciiFlag);

    ///< compute metrics
    if (m_metricsEnable) {
      cout << "Computing dmetrics..." << endl;
      pc_evalue::TMetricRes res;
      m_pointCloudOrg.setColors(pointCloudOrgColors);
      pc_evalue::computeMetric(m_pointCloudOrg, m_pointCloudRecon, MetricParam, res);
      totalMetricRes = totalMetricRes + res;
    }
  }

  cout << "All frames number of output points: " << totalNumReconPoints << endl;
  cout << "All frames geometry bits: " << geomBits << " bits." << endl;
  cout << "All frames attributes bits: " << attrBits << " bits." << endl;
  cout << "All frames total bitstream size: " << totalBits << " bits." << endl;
  cout << "All frames total processing time (user): " << totalUserTime << " sec." << endl << endl;

  if (m_metricsEnable) {
    totalMetricRes = totalMetricRes / m_numOfFrames;
    totalMetricRes.print(MetricParam);
  }
  if (md5FileOpenFlag)
    md5Calculator.closeFile();
  return exitState;
}

//////////////////////////////////////////////////////////////////////////
// Private class functions
//////////////////////////////////////////////////////////////////////////

Void TEncTop::initParameters() {
  m_hls.sps.level = 0;
  m_hls.sps.spsID = 0;

  m_hls.gps.gpsID = 0;
  m_hls.gps.spsID = 0;

  m_hls.aps.apsID = 0;
  m_hls.aps.spsID = 0;
}

Void TEncTop::compressAndEncodePartition() {
  m_geomEncoder.init(&m_pointCloudQuant, &m_pointCloudRecon, &m_hls, &m_encBac);
  m_geomEncoder.compressAndEncodeGeometry();
  m_geomEncoder.clear();
}

Void TEncTop::fastRecolor(const std::pair<const PC_POS, std::vector<int>> it,
                          const Int& uniquePointNumber, const Float& qs) {
  ///< fast recolor by weighted average in terms of the distance to current pos
  vector<Int64> weight;
  Int64 weightSum = 0;
  Double eps = 0.1;
  PC_POS posDequant = m_pointCloudQuant[uniquePointNumber] * qs;
  posDequant[0] += m_hls.sps.geomBoundingBoxOrigin[0];
  posDequant[1] += m_hls.sps.geomBoundingBoxOrigin[1];
  posDequant[2] += m_hls.sps.geomBoundingBoxOrigin[2];

  for (const auto& idx : it.second) {
    PC_POS posOrg = m_pointCloudOrg[idx];
    Double d = (posOrg - posDequant).getNorm1();
    Int64 w = (Int64)round(1024.0 / (eps + d / qs));
    weight.push_back(w);
    weightSum += w;
  }

  if (m_pointCloudOrg.hasColors()) {
    V3<Double> avg_col = 0;
    Int cnt = 0;
    for (const auto& idx : it.second) {
      Int64 w = weight[cnt++];
      avg_col[0] += m_pointCloudOrg.getColor(idx)[0] * w;
      avg_col[1] += m_pointCloudOrg.getColor(idx)[1] * w;
      avg_col[2] += m_pointCloudOrg.getColor(idx)[2] * w;
    }
    avg_col /= Double(weightSum);
    m_pointCloudQuant.getColor(uniquePointNumber)[0] =
      (UInt8)TComClip(0.0, 255.0, std::round(avg_col[0]));
    m_pointCloudQuant.getColor(uniquePointNumber)[1] =
      (UInt8)TComClip(0.0, 255.0, std::round(avg_col[1]));
    m_pointCloudQuant.getColor(uniquePointNumber)[2] =
      (UInt8)TComClip(0.0, 255.0, std::round(avg_col[2]));
  }

  if (m_pointCloudOrg.hasReflectances()) {
    Double avg_ref = 0;
    Int cnt = 0;
    for (const auto& idx : it.second) {
      Int64 w = weight[cnt++];
      avg_ref += m_pointCloudOrg.getReflectance(idx) * w;
    }
    avg_ref /= weightSum;
    avg_ref = TComClip(double(std::numeric_limits<PC_REFL>::min()),
                       double(std::numeric_limits<PC_REFL>::max()), std::round(avg_ref));

    m_pointCloudQuant.getReflectance(uniquePointNumber) = (PC_REFL)avg_ref;
  }
}

Void TEncTop::geomPreprocessAndQuantization(const Float& qs, Double& userTimeRecolor) {
  m_pointCloudQuant = m_pointCloudOrg;

  ///< shift to origin
  const TSize pointCount = m_pointCloudOrg.getNumPoint();
  for (Int i = 0; i < pointCount; i++) {
    m_pointCloudQuant[i][0] -= m_hls.sps.geomBoundingBoxOrigin[0];
    m_pointCloudQuant[i][1] -= m_hls.sps.geomBoundingBoxOrigin[1];
    m_pointCloudQuant[i][2] -= m_hls.sps.geomBoundingBoxOrigin[2];
  }

  ///< geometry quantization
  for (Int i = 0; i < pointCount; i++) {
    m_pointCloudQuant[i][0] = round(m_pointCloudQuant[i][0] / qs) - SliceOri[0];
    m_pointCloudQuant[i][1] = round(m_pointCloudQuant[i][1] / qs) - SliceOri[1];
    m_pointCloudQuant[i][2] = round(m_pointCloudQuant[i][2] / qs) - SliceOri[2];
  }

  ///< remove duplicate points here
  if (m_hls.sps.geomRemoveDuplicateFlag) {
    if (m_hls.sps.recolorMode == 0 || m_geomOnlyFlag == 1) {
      set<PC_POS> uniquePoints;
      Int uniquePointNumber = 0;
      for (Int i = 0; i < pointCount; i++) {
        if (uniquePoints.find(m_pointCloudQuant[i]) == uniquePoints.end()) {
          uniquePoints.insert(m_pointCloudQuant[i]);
          m_pointCloudQuant.swapPoints(i, uniquePointNumber);
          uniquePointNumber++;
        }
      }
      m_pointCloudQuant.setNumPoint(uniquePointNumber);
    } else {
      //fast recolor method
      if (m_pointCloudOrg.hasColors() || m_pointCloudOrg.hasReflectances()) {  ///< fast recolor
        clock_t userTimeRecolorBegin = clock();
        map<PC_POS, vector<Int>> uniquePoints;
        for (Int i = 0; i < pointCount; i++) {
          auto it = uniquePoints.find(m_pointCloudQuant[i]);
          if (it == uniquePoints.end())
            uniquePoints.insert(pair<PC_POS, vector<Int>>(m_pointCloudQuant[i], vector<Int>({i})));
          else
            it->second.push_back(i);
        }
        Int uniquePointNumber = 0;
        for (const auto& it : uniquePoints) {
          m_pointCloudQuant[uniquePointNumber] = it.first;

          if (m_hls.sps.recolorMode == 1) {
            fastRecolor(it, uniquePointNumber, qs);
          }
          uniquePointNumber++;
        }
        m_pointCloudQuant.setNumPoint(uniquePointNumber);
        userTimeRecolor = (Double)(clock() - userTimeRecolorBegin) / CLOCKS_PER_SEC;
      }
    }
  }
}

Void TEncTop::geomPreprocessAndQuantizationWithNoduplicate(const Float& qs,
                                                           TComPointCloud* const pointCloudOrg) {
  m_pointCloudQuant = m_pointCloudOrg;

  ///< shift to origin
  const TSize pointCount = m_pointCloudQuant.getNumPoint();
  for (Int i = 0; i < pointCount; i++) {
    m_pointCloudQuant[i][0] -= m_hls.sps.geomBoundingBoxOrigin[0];
    m_pointCloudQuant[i][1] -= m_hls.sps.geomBoundingBoxOrigin[1];
    m_pointCloudQuant[i][2] -= m_hls.sps.geomBoundingBoxOrigin[2];
  }

  ///< geometry quantization
  for (Int i = 0; i < pointCount; i++) {
    m_pointCloudQuant[i][0] = round(m_pointCloudQuant[i][0] / qs);
    m_pointCloudQuant[i][1] = round(m_pointCloudQuant[i][1] / qs);
    m_pointCloudQuant[i][2] = round(m_pointCloudQuant[i][2] / qs);
  }
}

Void TEncTop::geomPostprocessingAndDequantization(const Float& qs) {
  const TSize numPoints = m_pointCloudRecon.getNumPoint();

  ///< geometry dequantization
  for (TSize i = 0; i < numPoints; i++) {
    m_pointCloudRecon[i][0] = m_pointCloudRecon[i][0] * qs;
    m_pointCloudRecon[i][1] = m_pointCloudRecon[i][1] * qs;
    m_pointCloudRecon[i][2] = m_pointCloudRecon[i][2] * qs;
  }

  ///< shift back to world coordinates
  for (TSize i = 0; i < numPoints; i++) {
    m_pointCloudRecon[i][0] += m_hls.sps.geomBoundingBoxOrigin[0];
    m_pointCloudRecon[i][1] += m_hls.sps.geomBoundingBoxOrigin[1];
    m_pointCloudRecon[i][2] += m_hls.sps.geomBoundingBoxOrigin[2];
  }
}
Void TEncTop::compressAndEncodeAttribute() {
  m_attrEncoder.init(&m_pointCloudRecon, &m_pointCloudRecon, &m_hls, &m_encBac);
  m_attrEncoder.predictAndEncodeAttribute();
}

vector<SliceRegion> TEncTop::SliceDevisionByMortonCode(int slicenum) {
  //Slice

  std::vector<SliceRegion> slice;

  // Morton sorting
  int32_t voxelCount = m_pointCloudQuant.getNumPoint();
  std::vector<pointMortonCodeWithIndex> pointCloudMorton(voxelCount);
  for (int n = 0; n < voxelCount; n++) {
    const auto point = m_pointCloudQuant[n];
    const int32_t x = (int32_t)point[0];
    const int32_t y = (int32_t)point[1];
    const int32_t z = (int32_t)point[2];
    pointCloudMorton[n].mortonCode = mortonAddr(x, y, z);
    pointCloudMorton[n].index = n;
  }
  std::sort(pointCloudMorton.begin(), pointCloudMorton.end());

  //to make sure the num to move right
  int64_t movedigits = 0;
  while (((pointCloudMorton[voxelCount - 1].mortonCode >> movedigits) -
          (pointCloudMorton[0].mortonCode >> movedigits)) >= slicenum)
    movedigits++;
  int slicesize = ((pointCloudMorton[voxelCount - 1].mortonCode >> movedigits) -
                   (pointCloudMorton[0].mortonCode >> movedigits)) +
    1;
  slice.resize(slicesize);

  //to cteate slice
  int64_t fatherMorton = (pointCloudMorton[0].mortonCode >> movedigits);
  auto end = std::lower_bound(pointCloudMorton.begin(), pointCloudMorton.end(),
                              (fatherMorton) << movedigits) -
    pointCloudMorton.begin();
  for (int slice_id = 0;
       fatherMorton <= (pointCloudMorton[voxelCount - 1].mortonCode >> movedigits);
       slice_id++, fatherMorton++) {
    auto begin = end;
    end = std::lower_bound(pointCloudMorton.begin() + begin, pointCloudMorton.end(),
                           (fatherMorton + 1) << movedigits) -
      pointCloudMorton.begin();
    if (begin == end) {
      slice_id--;
      continue;
    }

    int index = 0;
    slice[slice_id].slice_id = slice_id;
    for (int i = begin; i < end && i < voxelCount; i++) {
      slice[slice_id].indexes.push_back(pointCloudMorton[i].index);
    }
  }

  slice.erase(std::remove_if(slice.begin(), slice.end(),
                             [](const SliceRegion& partion) { return partion.indexes.empty(); }),
              slice.end());

  return slice;
}

Void TEncTop::addToReconstructionCloud(TComPointCloud* reconstructionCloud) {
  int voxelCount_add = m_pointCloudRecon.getNumPoint();
  int voxelClout_re = reconstructionCloud->getNumPoint();
  if (m_hls.gbh.geomBoundingBoxOrigin.max())
    for (size_t idx = 0; idx < voxelCount_add; ++idx)
      for (size_t k = 0; k < 3; ++k)
        m_pointCloudRecon[idx][k] += m_hls.gbh.geomBoundingBoxOrigin[k];

  if (!m_hls.gbh.sliceID) {
    *reconstructionCloud = m_pointCloudRecon;
    return;
  }
  reconstructionCloud->init(voxelClout_re + voxelCount_add, m_pointCloudRecon.hasColors(),
                            m_pointCloudRecon.hasReflectances());
  std::copy(m_pointCloudRecon.positions().begin(), m_pointCloudRecon.positions().end(),
            std::next(reconstructionCloud->positions().begin(), voxelClout_re));
  if (reconstructionCloud->hasColors())
    std::copy(m_pointCloudRecon.getColors().begin(), m_pointCloudRecon.getColors().end(),
              std::next(reconstructionCloud->getColors().begin(), voxelClout_re));
  if (reconstructionCloud->hasReflectances())
    std::copy(m_pointCloudRecon.getReflectances().begin(),
              m_pointCloudRecon.getReflectances().end(),
              std::next(reconstructionCloud->getReflectances().begin(), voxelClout_re));
}
///< \}
