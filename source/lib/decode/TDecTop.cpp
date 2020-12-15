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

#include "TDecTop.h"
#include "common/MD5Sum.h"
#include "common/TComBufferChunk.h"
#include "common/contributors.h"
#include <time.h>

using namespace std;

///< \in TLibDecoder \{

/**
 * Implementation of TDecTop
 * decoder class
 */

//////////////////////////////////////////////////////////////////////////
// Public class functions
//////////////////////////////////////////////////////////////////////////

Int TDecTop::decode() {
  Int numDigits = getfileNumLength();  // numDigit indicates digits of file number
  setStartFrameNum(numDigits);         // set start frame name from m_inputFileName

  Int exitState = EXIT_SUCCESS;

  MD5Sum md5Calculator;
  bool CheckMd5Flag = md5Calculator.openIStream(m_md5FileName);

  ///< Print information
  Double totalUserTime = 0.0;
  Bool isMatch = true;

  for (UInt i_frame = 0; i_frame < m_numOfFrames; i_frame++) {
    printFrameCutoffRule(i_frame, numDigits);

    if (i_frame == 0) {
      resetReconFileName(m_startFrame, numDigits);  // reset m_reconFileName
    } else {
      updateIOFileName(m_startFrame + i_frame,
                       numDigits);  // update m_bitstreamFileName & m_reconFileName
    }

    ifstream bitstreamFile(m_bitstreamFileName, ios::binary);
    if (!bitstreamFile) {
      cerr << "Error: failed to open bitstream file " << m_bitstreamFileName << " for writing!"
           << endl;
      exitState |= EXIT_FAILURE;
      continue;  // if can not open the corresponding file then skip and move on to find next input file
    }

    Double userTimeTotal = 0.0, userTimeGeometry = 0.0, userTimeAttribute = 0.0;
    clock_t userTimeTotalBegin = clock();

    TComBufferChunk bufferChunk;
    TComPointCloud reconstructionCloud;
    reconstructionCloud.setNumPoint(0);

    while (!bitstreamFile.eof()) {
      if (bufferChunk.readFromBitstream(bitstreamFile) == EXIT_FAILURE) {
        exitState |= EXIT_FAILURE;
        continue;
      }

      m_decBac.setBitstreamBuffer(bufferChunk);
      switch (bufferChunk.getBufferType()) {
      case BufferChunkType::BCT_SPS: {
        m_decBac.parseSPS(m_hls.sps);
        bufferChunk.reset();
        m_decBac.reset();
        break;
      }
      case BufferChunkType::BCT_GPS: {
        m_decBac.parseGPS(m_hls.gps);
        bufferChunk.reset();
        m_decBac.reset();
        break;
      }
      case BufferChunkType::BCT_GBH: {
        m_decBac.parseGBH(m_hls.gbh);
        bufferChunk.reset();
        m_decBac.reset();
        break;
      }
      case BufferChunkType::BCT_GEOM: {
        clock_t userTimeGeometryBegin = clock();
        m_decBac.initBac();
        init_aec_context_tab();
        decompressAndDecodePartition();
        assert(m_decBac.decodeTerminationFlag());
        bufferChunk.reset();
        m_decBac.reset();
        userTimeGeometry = (Double)(clock() - userTimeGeometryBegin) / CLOCKS_PER_SEC;
        break;
      }
      case BufferChunkType::BCT_APS: {
        m_decBac.parseAPS(m_hls.aps);
        bufferChunk.reset();
        m_decBac.reset();
        break;
      }
      case BufferChunkType::BCT_ABH: {
        m_decBac.parseABH(m_hls.abh);
        bufferChunk.reset();
        m_decBac.reset();
        break;
      }
      case BufferChunkType::BCT_ATTR: {
        clock_t userTimeAttributeBegin = clock();
        m_decBac.initBac();
        init_aec_context_tab();
        decompressAttribute();
        assert(m_decBac.decodeTerminationFlag());
        bufferChunk.reset();
        m_decBac.reset();
        userTimeAttribute = (Double)(clock() - userTimeAttributeBegin) / CLOCKS_PER_SEC;
        addToReconstructionCloud(&reconstructionCloud);
        break;
      }
      default: {
        checkCond(false, "Error: invalid bitstream type!");
        exitState |= EXIT_FAILURE;
        continue;
      }
      }
    }
    bitstreamFile.close();
    m_pointCloudRecon = reconstructionCloud;
    userTimeTotal = (Double)(clock() - userTimeTotalBegin) / CLOCKS_PER_SEC;
    cout << "Geometry processing time (user): " << userTimeGeometry << " sec." << endl;
    cout << "Attribute processing time (user): " << userTimeAttribute << " sec." << endl;
    cout << "Total processing time (user): " << userTimeTotal << " sec." << endl << endl;
    totalUserTime += userTimeTotal;

    geomPostprocessingAndDequantization(m_hls.sps.geomQuantStep);

    if (m_colorTransformFlag && m_pointCloudRecon.hasColors()) {
      m_pointCloudRecon.convertYUVToRGB();
    }

    if (CheckMd5Flag) {
      md5Calculator.calculateMD5(&m_pointCloudRecon);

      string md5InFile = md5Calculator.getMD5InFile();
      string md5InRec = md5Calculator.getMD5Str();

      Bool compareMD5Flag = (md5InFile == md5InRec) ? true : false;
      isMatch &= compareMD5Flag;

      if (compareMD5Flag)
        cout << "MD5 check status: Success." << endl;
      else
        cout << "MD5 check status: Fail." << endl;
    }

    ///< write recon ply
    if (m_reconFileName.length() > 0) {
      m_pointCloudRecon.writeToFile(m_reconFileName, m_writePlyInAsciiFlag);
    }
  }  ///< end for loop all frames

  cout << endl << "All frames total processing time (user): " << totalUserTime << " sec." << endl;
  if (CheckMd5Flag)
    cout << "All frames MD5 check status: " << (isMatch ? "Success." : "Fail.") << endl << endl;

  if (CheckMd5Flag)
    md5Calculator.closeFile();

  return exitState;
}

//////////////////////////////////////////////////////////////////////////
// Private class functions
//////////////////////////////////////////////////////////////////////////

Void TDecTop::decompressAndDecodePartition() {
  m_geomDecoder.init(&m_pointCloudRecon, &m_hls, &m_decBac);
  m_geomDecoder.decodeAndDecompress();
  m_geomDecoder.clear();
}

Void TDecTop::geomPostprocessingAndDequantization(const Float& qs) {
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
Void TDecTop::decompressAttribute() {
  m_attrDecoder.init(&m_pointCloudRecon, &m_hls, &m_decBac);
  m_attrDecoder.decodeAttribute();
}
Void TDecTop::addToReconstructionCloud(TComPointCloud* reconstructionCloud) {
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

  reconstructionCloud->init(voxelClout_re + voxelCount_add, m_hls.aps.withColor, m_hls.aps.withRef);
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
//! \}
