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

#include "TDecBacTop.h"
#include "TDecBacCore.h"
#include "common/TComRom.h"
#include "common/contributors.h"

///< \in TLibDecoder \{

/**
 * Class TDecBacTop
 * entropy decoder
 */

//////////////////////////////////////////////////////////////////////////
// Public class functions
//////////////////////////////////////////////////////////////////////////

Void TDecBacTop::parseSPS(SequenceParameterSet& sps) {
  sps.level = m_bac->com_bsr_read(&m_bitStream, 8);
  m_bac->com_bsr_read1(&m_bitStream);

  Int bb_x_upper = (Int)m_bac->com_bsr_read_se(&m_bitStream);
  m_bac->com_bsr_read1(&m_bitStream);
  Int bb_x_lower = (Int)m_bac->com_bsr_read_se(&m_bitStream);
  m_bac->com_bsr_read1(&m_bitStream);
  sps.geomBoundingBoxOrigin[0] = (bb_x_upper << 16) + bb_x_lower;

  Int bb_y_upper = (Int)m_bac->com_bsr_read_se(&m_bitStream);
  m_bac->com_bsr_read1(&m_bitStream);
  Int bb_y_lower = (Int)m_bac->com_bsr_read_se(&m_bitStream);
  m_bac->com_bsr_read1(&m_bitStream);
  sps.geomBoundingBoxOrigin[1] = (bb_y_upper << 16) + bb_y_lower;

  Int bb_z_upper = (Int)m_bac->com_bsr_read_se(&m_bitStream);
  m_bac->com_bsr_read1(&m_bitStream);
  Int bb_z_lower = (Int)m_bac->com_bsr_read_se(&m_bitStream);
  m_bac->com_bsr_read1(&m_bitStream);
  sps.geomBoundingBoxOrigin[2] = (bb_z_upper << 16) + bb_z_lower;

  UInt bbs_w_upper = (UInt)m_bac->com_bsr_read_ue(&m_bitStream);
  m_bac->com_bsr_read1(&m_bitStream);
  UInt bbs_w_lower = (UInt)m_bac->com_bsr_read_ue(&m_bitStream);
  m_bac->com_bsr_read1(&m_bitStream);
  sps.geomBoundingBoxSize[0] = (bbs_w_upper << 16) + bbs_w_lower;

  UInt bbs_h_upper = (UInt)m_bac->com_bsr_read_ue(&m_bitStream);
  m_bac->com_bsr_read1(&m_bitStream);
  UInt bbs_h_lower = (UInt)m_bac->com_bsr_read_ue(&m_bitStream);
  m_bac->com_bsr_read1(&m_bitStream);
  sps.geomBoundingBoxSize[1] = (bbs_h_upper << 16) + bbs_h_lower;

  UInt bbs_d_upper = (UInt)m_bac->com_bsr_read_ue(&m_bitStream);
  m_bac->com_bsr_read1(&m_bitStream);
  UInt bbs_d_lower = (UInt)m_bac->com_bsr_read_ue(&m_bitStream);
  m_bac->com_bsr_read1(&m_bitStream);
  sps.geomBoundingBoxSize[2] = (bbs_d_upper << 16) + bbs_d_lower;

  UInt32 qs_upper = (UInt32)m_bac->com_bsr_read(&m_bitStream, 16);
  m_bac->com_bsr_read1(&m_bitStream);
  UInt32 qs_lower = (UInt32)m_bac->com_bsr_read(&m_bitStream, 16);

  UInt32 qs = (qs_upper << 16) + qs_lower;

  sps.geomQuantStep = *(Float*)(&qs);

  sps.geomRemoveDuplicateFlag = m_bac->com_bsr_read1(&m_bitStream);
  m_bac->com_bsr_read1(&m_bitStream);

  sps.attrAdaptPred = m_bac->com_bsr_read1(&m_bitStream);
  sps.attrQuantParam = (UInt)m_bac->com_bsr_read_ue(&m_bitStream);

  m_bac->com_bsr_read_byte_align(&m_bitStream);
}

Void TDecBacTop::parseGPS(GeometryParameterSet& gps) {
  gps.depthFirstNodeSizeLog2 = (UInt)m_bac->com_bsr_read_ue(&m_bitStream);
  if (gps.depthFirstNodeSizeLog2 > 0)
    gps.depthFirstNodeSizeLog2++;
  gps.im_qtbt_flag = !!m_bac->com_bsr_read1(&m_bitStream);
  gps.im_qtbt_num_before_ot = (UInt)m_bac->com_bsr_read_ue(&m_bitStream);
  gps.im_qtbt_min_size = (UInt)m_bac->com_bsr_read_ue(&m_bitStream);
  gps.singleModeFlag = !!m_bac->com_bsr_read1(&m_bitStream);
  gps.geom_context_mode = !!m_bac->com_bsr_read1(&m_bitStream);
  m_bac->com_bsr_read_byte_align(&m_bitStream);
}

Void TDecBacTop::parseGBH(GeometryBrickHeader& gbh) {
  //gbh.gbhID = (UInt)m_bac->com_bsr_read_ue(&m_bitStream);
  gbh.sliceID = (UInt)m_bac->com_bsr_read_ue(&m_bitStream);
  Int bb_x_upper = (Int)m_bac->com_bsr_read_se(&m_bitStream);
  m_bac->com_bsr_read1(&m_bitStream);
  Int bb_x_lower = (Int)m_bac->com_bsr_read_se(&m_bitStream);
  m_bac->com_bsr_read1(&m_bitStream);
  gbh.geomBoundingBoxOrigin[0] = (bb_x_upper << 16) + bb_x_lower;

  Int bb_y_upper = (Int)m_bac->com_bsr_read_se(&m_bitStream);
  m_bac->com_bsr_read1(&m_bitStream);
  Int bb_y_lower = (Int)m_bac->com_bsr_read_se(&m_bitStream);
  m_bac->com_bsr_read1(&m_bitStream);
  gbh.geomBoundingBoxOrigin[1] = (bb_y_upper << 16) + bb_y_lower;

  Int bb_z_upper = (Int)m_bac->com_bsr_read_se(&m_bitStream);
  m_bac->com_bsr_read1(&m_bitStream);
  Int bb_z_lower = (Int)m_bac->com_bsr_read_se(&m_bitStream);
  m_bac->com_bsr_read1(&m_bitStream);
  gbh.geomBoundingBoxOrigin[2] = (bb_z_upper << 16) + bb_z_lower;

  UInt nsl_x_upper = (UInt)m_bac->com_bsr_read_ue(&m_bitStream);
  m_bac->com_bsr_read1(&m_bitStream);
  UInt nsl_x_lower = (UInt)m_bac->com_bsr_read_ue(&m_bitStream);
  m_bac->com_bsr_read1(&m_bitStream);
  gbh.nodeSizeLog2[0] = (nsl_x_upper << 16) + nsl_x_lower;

  UInt nsl_y_upper = (UInt)m_bac->com_bsr_read_ue(&m_bitStream);
  m_bac->com_bsr_read1(&m_bitStream);
  UInt nsl_y_lower = (UInt)m_bac->com_bsr_read_ue(&m_bitStream);
  m_bac->com_bsr_read1(&m_bitStream);
  gbh.nodeSizeLog2[1] = (nsl_y_upper << 16) + nsl_y_lower;

  UInt nsl_z_upper = (UInt)m_bac->com_bsr_read_ue(&m_bitStream);
  m_bac->com_bsr_read1(&m_bitStream);
  UInt nsl_z_lower = (UInt)m_bac->com_bsr_read_ue(&m_bitStream);
  m_bac->com_bsr_read1(&m_bitStream);
  gbh.nodeSizeLog2[2] = (nsl_z_upper << 16) + nsl_z_lower;

  UInt np_upper = (UInt)m_bac->com_bsr_read_ue(&m_bitStream);
  m_bac->com_bsr_read1(&m_bitStream);
  UInt np_lower = (UInt)m_bac->com_bsr_read_ue(&m_bitStream);
  m_bac->com_bsr_read1(&m_bitStream);
  gbh.geomNumPoints = (np_upper << 16) + np_lower;

  m_bac->com_bsr_read_byte_align(&m_bitStream);
}

Void TDecBacTop::parseAPS(AttributeParameterSet& aps) {
  aps.withColor = !!m_bac->com_bsr_read1(&m_bitStream);
  aps.withRef = !!m_bac->com_bsr_read1(&m_bitStream);
  if (aps.withColor) {
    aps.maxNumOfNeighbours = (UInt)m_bac->com_bsr_read_ue(&m_bitStream);
    aps.crossComponentPred = !!m_bac->com_bsr_read1(&m_bitStream);
  }
  aps.numOflevelOfDetail = (UInt)m_bac->com_bsr_read_ue(&m_bitStream);
  aps.maxNumOfPredictNeighbours = (UInt)m_bac->com_bsr_read_ue(&m_bitStream);
  aps.intraLodFlag = !!m_bac->com_bsr_read1(&m_bitStream);
  m_bac->com_bsr_read_byte_align(&m_bitStream);
}

Void TDecBacTop::parseABH(AttributeBrickHeader& abh) {
  //gbh.gbhID = (UInt)m_bac->com_bsr_read_ue(&m_bitStream);
  abh.sliceID = (UInt)m_bac->com_bsr_read_ue(&m_bitStream);

  m_bac->com_bsr_read_byte_align(&m_bitStream);
}

UInt TDecBacTop::parseAttrCacheIndex() {
  UInt cacheIdx = 0;
  for (int i = 0; i < ATTRIBUTE_CACHE_SIZE_BIT; i++) {
    cacheIdx += (m_bac->biari_decode_symbol(p_aec, &(p_aec->syn_ctx.ctx_attr_cache_idx[i])) << i);
  }
  return cacheIdx;
}

Int TDecBacTop::parseAttr(const int componentId) {
  int ctx_id = componentId;
  Int val = 0;
  if (!m_bac->biari_decode_symbol(p_aec, &p_aec->syn_ctx.ctx_attr_residual_eq0[ctx_id])) {
    Int sign_bit = m_bac->biari_decode_symbol_eq_prob(p_aec);
    if (!m_bac->biari_decode_symbol(p_aec, &p_aec->syn_ctx.ctx_attr_residual_eq1[ctx_id])) {
      if (!m_bac->biari_decode_symbol(p_aec, &p_aec->syn_ctx.ctx_attr_residual_eq2[ctx_id])) {

		  if (!m_bac->biari_decode_symbol(p_aec, &p_aec->syn_ctx.ctx_attr_residual_eq3[ctx_id])){
           if (!m_bac->biari_decode_symbol(p_aec, &p_aec->syn_ctx.ctx_attr_residual_eq4[ctx_id])) {
             if (!m_bac->biari_decode_symbol(p_aec, &p_aec->syn_ctx.ctx_attr_residual_eq5[ctx_id])) {
               if (!m_bac->biari_decode_symbol(p_aec,
                                               &p_aec->syn_ctx.ctx_attr_residual_eq6[ctx_id])) {
                 if (!m_bac->biari_decode_symbol(p_aec,
                                                 &p_aec->syn_ctx.ctx_attr_residual_eq7[ctx_id])) {
                   val = m_bac->sbac_read_ue_ep(p_aec) + 8;
                 } else {
                   val = 7;
                 }
                 
               } 
			   else {
                 val = 6;
               }
               
          } 
		  
		  
		  else {
            val = 5;
          }
          
          } 
		   else {
            val = 4;
          }
        } 
		  else {val = 3;}
      } 
	  
	  
	  else {
        val = 2;
      }
    } 
	
	else {
      val = 1;
    }
    val = (sign_bit == 1) ? val : -val;
  }

  m_residualStats.collectStats(abs(val));
  return val;
}

UInt TDecBacTop::decodeOccupancyCode(const UInt& occupancySkip, const TComGeomContext& geomCtx) {
  UInt occupancy = 0;
  auto p_ctx = p_aec->syn_ctx.ctx_occupancy;

  static const UInt8 adjacentIdx[8][3] = {{1, 2, 4}, {0, 3, 5}, {0, 3, 6}, {1, 2, 7},
                                          {0, 5, 6}, {1, 4, 7}, {2, 4, 7}, {3, 5, 6}};
  static const UInt8 adjacentPosIdx[8][3] = {{2, 1, 0}, {5, 1, 0}, {4, 2, 0}, {4, 5, 0},
                                             {3, 2, 1}, {3, 5, 1}, {3, 4, 2}, {3, 4, 5}};

  const UInt* ctxChildD1 = geomCtx.ctxChildD1;
  const UInt* ctxParent = geomCtx.ctxParentAdv;

  UInt8 ctxChildNode[8] = {0};
  for (Int i = 0; i < 8; i++) {
    for (Int j = 0; j < 6; j++) {
      ctxChildNode[i] |= (!!(ctxChildD1[j] & (1 << i))) << j;
    }
  }

  for (Int i = 0; i < 8; i++) {
    if (occupancySkip != 0) {
      if ((occupancySkip & 1) && (i & 1))  ///< skip when z = 1
        continue;
      if ((occupancySkip & 2) && (i & 2))  ///< skip when y = 1
        continue;
      if ((occupancySkip & 4) && (i & 4))  ///< skip when x = 1
        continue;
    }

    Bool bit = m_bac->biari_decode_symbol(p_aec, &p_ctx[ctxParent[i]][ctxChildNode[i]]);
    occupancy |= bit << i;

    ///< update adjacent child node status
    if (bit) {
      for (Int k = 0; k < 3; k++) {
        assert(!((ctxChildNode[adjacentIdx[i][k]] >> adjacentPosIdx[i][k]) & 1));
        ctxChildNode[adjacentIdx[i][k]] |= 1 << adjacentPosIdx[i][k];
      }
    }
  }
  return occupancy;
}

UInt TDecBacTop::decodeOccupancyCode(int bitIdx, int32_t pattern_small, int update_pattern,
                                     int pattern_big) {
  UInt bit = 0;
  context_t* bit_ctx = NULL;
  int pattern = 0;
  if ((update_pattern == 0) && (pattern_small == 0)) {
    bit_ctx = Dec_context_idx(bitIdx);
    pattern = pattern_big;
  } else {
    bit_ctx = Dec_context_idx(0, update_pattern);
    pattern = pattern_small;
  }
  // context_t* bit_ctx = Dec_context_idx(bitIdx, update_pattern);
  bit = m_bac->biari_decode_symbol(p_aec, bit_ctx + pattern);
  return bit;
}

V3<UInt> TDecBacTop::decodeSinglePointIndex(V3<UInt> nodeSizeLog2) {
  V3<UInt> pos;
  while (nodeSizeLog2 > 0) {
    for (Int i = 0; i < 3; ++i) {
      if (nodeSizeLog2[i] > 0) {
        pos[i] |= m_bac->biari_decode_symbol_eq_prob(p_aec) << (nodeSizeLog2[i] - 1);
        --nodeSizeLog2[i];
      }
    }
  }
  return pos;
}

UInt TDecBacTop::decodeDuplicateNumber() {
  UInt numDup = 1;
  auto p_ctx = &p_aec->syn_ctx.ctx_geom_num_dup_eq1;
  if (!m_bac->biari_decode_symbol(p_aec, p_ctx))
    numDup = m_bac->sbac_read_ue_ep(p_aec) + 2;
  return numDup;
}

Bool TDecBacTop::decodeTerminationFlag() {
  return !!m_bac->biari_decode_final(p_aec);
}

context_t* TDecBacTop::Dec_context_idx(int idx) {
  switch (idx) {
  case 0:
    return p_aec->syn_ctx.b0;
    break;
  case 1:
    return p_aec->syn_ctx.b1;
    break;
  case 2:
    return p_aec->syn_ctx.b2;
    break;
  case 3:
    return p_aec->syn_ctx.b3;
    break;
  case 4:
    return p_aec->syn_ctx.b4;
    break;
  case 5:
    return p_aec->syn_ctx.b5;
    break;
  case 6:
    return p_aec->syn_ctx.b6;
    break;
  case 7:
    return p_aec->syn_ctx.b7;
    break;
  }
}
context_t* TDecBacTop::Dec_context_idx(int idx, int update_pattern) {
  switch (idx) {
  case 0:
    return p_aec->syn_ctx.B0[update_pattern];
    break;
  case 1:
    return p_aec->syn_ctx.B1[update_pattern];
    break;
  case 2:
    return p_aec->syn_ctx.B2[update_pattern];
    break;
  case 3:
    return p_aec->syn_ctx.B3[update_pattern];
    break;
  case 4:
    return p_aec->syn_ctx.B4[update_pattern];
    break;
  case 5:
    return p_aec->syn_ctx.B5[update_pattern];
    break;
  case 6:
    return p_aec->syn_ctx.B6[update_pattern];
    break;
  case 7:
    return p_aec->syn_ctx.B7[update_pattern];
  }
}

TDecBacTop::TDecBacTop() {
  m_bac = nullptr;
  reset();
}

TDecBacTop::~TDecBacTop() {
  if (m_bac) {
    delete m_bac;
    m_bac = nullptr;
  }
}

Void TDecBacTop::initBac() {
  m_bac = new TDecBacCore;
  m_bac->dec_sbac_init(&m_bitStream);
}

Void TDecBacTop::reset() {
  if (m_bac) {
    delete m_bac;
    m_bac = nullptr;
  }
  memset(&m_bitStream, 0, sizeof(m_bitStream));
  aec.init();
  p_aec = &aec;
}

Void TDecBacTop::setBitstreamBuffer(TComBufferChunk& buffer) {
  m_bac->com_bsr_init(&m_bitStream, (UInt8*)buffer.addr, buffer.ssize, NULL);

  if (buffer.getBufferType() == BufferChunkType::BCT_GEOM ||
      buffer.getBufferType() == BufferChunkType::BCT_ATTR) {
    m_bac->aec_init_contexts(p_aec);
    m_bac->aec_start_decoding(p_aec, m_bitStream.beg, 0,
                              buffer.ssize);  // TODO:╪сио
  }
}

///< \{
