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

#include "TEncBacCore.h"
#include "common/CommonDef.h"
#include "common/ContextModel.h"
#include "common/HighLevelSyntax.h"
#include "common/TComBufferChunk.h"
#include "common/TComOccupancyMap.h"
#include "common/TComResidualStats.h"
#include "common/contributors.h"

///< \in TLibEncoder \{

/**
 * Class TEncBacTop
 * entropy encoder
 */

class TEncBacTop {
private:
  COM_BS m_bitStream;
  TEncBacCore* m_bac;
  aec_t aec;
  aec_t* p_aec;
  TComResidualStats m_residualStats;
  Bool m_forceAttrBasedPred{false};

public:
  ///< encoding syntax
  Void codeSPS(const SequenceParameterSet& sps);
  Void codeGPS(const GeometryParameterSet& gps);
  Void codeGBH(const GeometryBrickHeader& gbh);
  Void codeAPS(const AttributeParameterSet& aps);
  Void codeABH(const AttributeBrickHeader& abh);
  Void codeAttributerResidual(const int64_t& delta, const int componentId = 0);
  Void encodeOccupancyCode(const UInt& occupancyCode, const UInt& occupancySkip,
                           const TComGeomContext& geomCtx);
  Void encodeOccupancyCode(const UInt& occupancyCode, const TComOctreePartitionParams& params,
                           uint32_t* bit_pattern, uint32_t* update_pattern, int pattern);
  Void encodeSinglePointIndex(const V3<UInt>& pos, V3<UInt> nodeSizeLog2);
  Void encodeDuplicateNumber(const UInt& dupNum);
  Void encodeAttrCacheIndex(const UInt& cacheIdx);
  Void encodeTerminationFlag(const Bool terminateFlag = true);
  Void encodeFinish();
  context_t* Enc_context_idx(int idx);
  context_t* Enc_context_idx(int idx, int update_pattern);

  TEncBacTop();
  ~TEncBacTop();
  Void reset();
  Void initBac();
  Void setBitstreamBuffer(TComBufferChunk& buffer);
  UInt64 getBitStreamLength();  ///< get number of byte written
  TComResidualStats& getResidualStats() {
    return m_residualStats;
  }

  Bool& getForceAttrBasedPred() {
    return m_forceAttrBasedPred;
  }

};  ///< END CLASS TEncBacTop

///< \}
