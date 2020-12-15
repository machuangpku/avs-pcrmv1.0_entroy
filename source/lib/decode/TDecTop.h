#pragma once
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
#include "TDecBacTop.h"
#include "TDecCfg.h"
#include "TDecGeometry.h"
#include "common/TComPointCloud.h"
#include "common/contributors.h"

///< \in TLibDecoder \{

/**
 * Class TDecTop
 * decoder class
 */

class TDecTop : public TDecCfg {
private:
  TComPointCloud m_pointCloudRecon;  ///< output point cloud

  TDecGeometry m_geomDecoder;   ///< geometry decoder
  TDecBacTop m_decBac;          ///< AVS3 binary arithmetic decoder
  TDecAttribute m_attrDecoder;  ///< attribute decoder

public:
  TDecTop() = default;
  ~TDecTop() = default;

  Int decode();  ///< main decoding function

  Int getFrameNum() {
    return m_numOfFrames;
  }  // get m_numOfFrames
  Int getstartFrame() {
    return m_startFrame;
  }  // get m_startFrame
  // set start frame name from m_bitstreamFileName
  void setStartFrameNum(Int numDigits) {
    char* numofStartFrame = new char[numDigits + 1];
    Int numStartPos = Int(m_bitstreamFileName.find_last_of('.')) - numDigits;
#pragma warning(push)
#pragma warning(disable : 4996)
    std::strcpy(numofStartFrame, m_bitstreamFileName.substr(numStartPos, numDigits).c_str());
#pragma warning(pop)
    m_startFrame = atoi(numofStartFrame);
    delete[] numofStartFrame;
  }
  // get digit of number of file number
  Int getfileNumLength() {
    Int dotLocation = Int(m_bitstreamFileName.find_last_of('.'));
    Int numDigits = 0;
    Int fileNameLength = Int(m_bitstreamFileName.length());
    dotLocation--;
    while (m_bitstreamFileName[dotLocation - numDigits] >= '0' &&
           m_bitstreamFileName[dotLocation - numDigits] <= '9') {
      numDigits++;
      if ((numDigits + 4) >= fileNameLength)
        break;
    }
    return (numDigits);
  }
  // update m_bitstreamFileName & m_reconFileName based on file number
  void updateIOFileName(Int i_frameNum, Int numDigits) {
    assert(numDigits > 0);

    char* c_frameNum = new char[numDigits + 1];
    snprintf(c_frameNum, numDigits + 1, "%0*d", numDigits, i_frameNum);

    if (m_bitstreamFileName.length() > 0)
      m_bitstreamFileName.replace(m_bitstreamFileName.find_last_of('.') - numDigits, numDigits,
                                  c_frameNum);
    if (m_reconFileName.length() > 0)
      m_reconFileName.replace(m_reconFileName.find_last_of('.') - numDigits, numDigits, c_frameNum);

    delete[] c_frameNum;
  }
  // reset m_reconFileName based on file number
  void resetReconFileName(Int i_frameNum, Int numDigits) {
    if (m_numOfFrames <= 1)
      return;
    char* c_frameNum = new char[numDigits + 1];
    snprintf(c_frameNum, numDigits + 1, "%0*d", numDigits, i_frameNum);
    if (m_reconFileName.length() > 0) {
      assert(numDigits > 0);
      string add = "-" + string(c_frameNum);
      m_reconFileName.insert(m_reconFileName.find_last_of('.'), add);
    }

    delete[] c_frameNum;
  }

  // print cut-off rule in log for each frame
  void printFrameCutoffRule(Int i_frameNum, Int numDigits) {
    char* c_frameNum = new char[numDigits + 1];
    snprintf(c_frameNum, numDigits + 1, "%0*d", numDigits, i_frameNum + m_startFrame);
    string cutoffRule = "------------ frame" + string(c_frameNum) + " ------------";
    cout << endl << cutoffRule << endl;
    delete[] c_frameNum;
  }

private:
  Void decompressAndDecodePartition();
  Void geomPostprocessingAndDequantization(const Float& qs);
  Void decompressAttribute();
  Void addToReconstructionCloud(TComPointCloud* reconstructionCloud);

};  ///< END CLASS TDecTop

///< \}
