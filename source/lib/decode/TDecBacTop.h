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

#include "TDecBacCore.h"
#include "common/CommonDef.h"
#include "common/ContextModel.h"
#include "common/HighLevelSyntax.h"
#include "common/TComBufferChunk.h"
#include "common/TComOccupancyMap.h"
#include "common/TComResidualStats.h"
#include "common/contributors.h"

#define CHECK_ALL_CTX 0

///< \in TLibDecoder \{

/**
 * Class TDecBacTop
 * entropy decoder
 */

class TDecBacTop {
private:
  COM_BS m_bitStream;
  TDecBacCore* m_bac;
  aec_t aec;
  aec_t* p_aec;
  TComResidualStats m_residualStats;
  Bool m_forceAttrBasedPred{false};

public:
  ///< decoding syntax
  Void parseSPS(SequenceParameterSet& sps);
  Void parseGPS(GeometryParameterSet& gps);
  Void parseGBH(GeometryBrickHeader& gbh);
  Void parseAPS(AttributeParameterSet& aps);
  Void parseABH(AttributeBrickHeader& abh);
  UInt decodeOccupancyCode(const UInt& occupancySkip, const TComGeomContext& geomCtx);
  UInt decodeOccupancyCode(int bitIdx, int32_t pattern, int update_pattern, int pattern_big);
  V3<UInt> decodeSinglePointIndex(V3<UInt> nodeSizeLog2);
  UInt decodeDuplicateNumber();
  UInt parseAttrCacheIndex();
  Bool decodeTerminationFlag();
  Int parseAttr(const int componentId = 0);
  context_t* Dec_context_idx(int idx);
  context_t* Dec_context_idx(int idx, int update_pattern);
  TDecBacTop();
  ~TDecBacTop();
  Void reset();
  Void initBac();
  Void setBitstreamBuffer(TComBufferChunk& buffer);
  TComResidualStats& getResidualStats() {
    return m_residualStats;
  }

  Bool& getForceAttrBasedPred() {
    return m_forceAttrBasedPred;
  }

};  ///< END CLASS TDecBacTop

///< \{
