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

#include "TEncBacTop.h"
#include "common/TComRom.h"
#include "common/contributors.h"

///< \in TLibEncoder \{

/**
 * Class TEncBacTop
 * entropy encoder
 */

//////////////////////////////////////////////////////////////////////////
// Public class functions
//////////////////////////////////////////////////////////////////////////

Void TEncBacTop::codeSPS(const SequenceParameterSet& sps) {
  UInt32 geomQuantStep_upper, geomQuantStep_lower;
  Int geomBoundingBoxOrigin_x_upper, geomBoundingBoxOrigin_x_lower;
  Int geomBoundingBoxOrigin_y_upper, geomBoundingBoxOrigin_y_lower;
  Int geomBoundingBoxOrigin_z_upper, geomBoundingBoxOrigin_z_lower;
  UInt geomBoundingBoxSize_w_upper, geomBoundingBoxSize_w_lower;
  UInt geomBoundingBoxSize_h_upper, geomBoundingBoxSize_h_lower;
  UInt geomBoundingBoxSize_d_upper, geomBoundingBoxSize_d_lower;
  UInt geomNumPoints_upper, geomNumPoints_lower;

  geomQuantStep_upper = (*(UInt32*)(&sps.geomQuantStep) >> 16) & 0xffff;
  geomQuantStep_lower = (*(UInt32*)(&sps.geomQuantStep)) & 0xffff;

  m_bac->com_bsw_write(&m_bitStream, sps.level, 8);
  m_bac->com_bsw_write1(&m_bitStream, 1);

  geomBoundingBoxOrigin_x_upper = (*(Int*)(&sps.geomBoundingBoxOrigin[0]) >> 16) & 0xffff;
  geomBoundingBoxOrigin_x_lower = (*(Int*)(&sps.geomBoundingBoxOrigin[0])) & 0xffff;
  m_bac->com_bsw_write_se(&m_bitStream, geomBoundingBoxOrigin_x_upper);
  m_bac->com_bsw_write1(&m_bitStream, 1);
  m_bac->com_bsw_write_se(&m_bitStream, geomBoundingBoxOrigin_x_lower);
  m_bac->com_bsw_write1(&m_bitStream, 1);

  geomBoundingBoxOrigin_y_upper = (*(Int*)(&sps.geomBoundingBoxOrigin[1]) >> 16) & 0xffff;
  geomBoundingBoxOrigin_y_lower = (*(Int*)(&sps.geomBoundingBoxOrigin[1])) & 0xffff;
  m_bac->com_bsw_write_se(&m_bitStream, geomBoundingBoxOrigin_y_upper);
  m_bac->com_bsw_write1(&m_bitStream, 1);
  m_bac->com_bsw_write_se(&m_bitStream, geomBoundingBoxOrigin_y_lower);
  m_bac->com_bsw_write1(&m_bitStream, 1);

  geomBoundingBoxOrigin_z_upper = (*(Int*)(&sps.geomBoundingBoxOrigin[2]) >> 16) & 0xffff;
  geomBoundingBoxOrigin_z_lower = (*(Int*)(&sps.geomBoundingBoxOrigin[2])) & 0xffff;
  m_bac->com_bsw_write_se(&m_bitStream, geomBoundingBoxOrigin_z_upper);
  m_bac->com_bsw_write1(&m_bitStream, 1);
  m_bac->com_bsw_write_se(&m_bitStream, geomBoundingBoxOrigin_z_lower);
  m_bac->com_bsw_write1(&m_bitStream, 1);

  geomBoundingBoxSize_w_upper = (*(Int*)(&sps.geomBoundingBoxSize[0]) >> 16) & 0xffff;
  geomBoundingBoxSize_w_lower = (*(Int*)(&sps.geomBoundingBoxSize[0])) & 0xffff;
  m_bac->com_bsw_write_ue(&m_bitStream, geomBoundingBoxSize_w_upper);
  m_bac->com_bsw_write1(&m_bitStream, 1);
  m_bac->com_bsw_write_ue(&m_bitStream, geomBoundingBoxSize_w_lower);
  m_bac->com_bsw_write1(&m_bitStream, 1);

  geomBoundingBoxSize_h_upper = (*(Int*)(&sps.geomBoundingBoxSize[1]) >> 16) & 0xffff;
  geomBoundingBoxSize_h_lower = (*(Int*)(&sps.geomBoundingBoxSize[1])) & 0xffff;
  m_bac->com_bsw_write_ue(&m_bitStream, geomBoundingBoxSize_h_upper);
  m_bac->com_bsw_write1(&m_bitStream, 1);
  m_bac->com_bsw_write_ue(&m_bitStream, geomBoundingBoxSize_h_lower);
  m_bac->com_bsw_write1(&m_bitStream, 1);

  geomBoundingBoxSize_d_upper = (*(Int*)(&sps.geomBoundingBoxSize[2]) >> 16) & 0xffff;
  geomBoundingBoxSize_d_lower = (*(Int*)(&sps.geomBoundingBoxSize[2])) & 0xffff;
  m_bac->com_bsw_write_ue(&m_bitStream, geomBoundingBoxSize_d_upper);
  m_bac->com_bsw_write1(&m_bitStream, 1);
  m_bac->com_bsw_write_ue(&m_bitStream, geomBoundingBoxSize_d_lower);
  m_bac->com_bsw_write1(&m_bitStream, 1);

  m_bac->com_bsw_write(&m_bitStream, geomQuantStep_upper, 16);
  m_bac->com_bsw_write1(&m_bitStream, 1);
  m_bac->com_bsw_write(&m_bitStream, geomQuantStep_lower, 16);

  m_bac->com_bsw_write1(&m_bitStream, sps.geomRemoveDuplicateFlag);
  m_bac->com_bsw_write1(&m_bitStream, 1);

  // encoding attribute params
  m_bac->com_bsw_write1(&m_bitStream, sps.attrAdaptPred);
  m_bac->com_bsw_write_ue(&m_bitStream, sps.attrQuantParam);
  m_bac->com_bsw_write_byte_align(&m_bitStream);
}

Void TEncBacTop::codeGPS(const GeometryParameterSet& gps) {
  if (gps.depthFirstNodeSizeLog2 == 0)
    m_bac->com_bsw_write_ue(&m_bitStream, 0);
  else
    m_bac->com_bsw_write_ue(&m_bitStream, gps.depthFirstNodeSizeLog2 - 1);
  m_bac->com_bsw_write1(&m_bitStream, gps.im_qtbt_flag);
  m_bac->com_bsw_write_ue(&m_bitStream, gps.im_qtbt_num_before_ot);
  m_bac->com_bsw_write_ue(&m_bitStream, gps.im_qtbt_min_size);
  m_bac->com_bsw_write1(&m_bitStream, gps.singleModeFlag);
  m_bac->com_bsw_write1(&m_bitStream, gps.geom_context_mode);
  m_bac->com_bsw_write_byte_align(&m_bitStream);
}

Void TEncBacTop::codeGBH(const GeometryBrickHeader& gbh) {
  // m_bac->com_bsw_write_ue(&m_bitStream, gbh.gbhID);
  m_bac->com_bsw_write_ue(&m_bitStream, gbh.sliceID);

  Int geomBoundingBoxOrigin_x_upper, geomBoundingBoxOrigin_x_lower;
  Int geomBoundingBoxOrigin_y_upper, geomBoundingBoxOrigin_y_lower;
  Int geomBoundingBoxOrigin_z_upper, geomBoundingBoxOrigin_z_lower;
  UInt nodeSizeLog2_x_upper, nodeSizeLog2_x_lower;
  UInt nodeSizeLog2_y_upper, nodeSizeLog2_y_lower;
  UInt nodeSizeLog2_z_upper, nodeSizeLog2_z_lower;
  UInt geomNumPoints_upper, geomNumPoints_lower;

  geomBoundingBoxOrigin_x_upper = (*(Int*)(&gbh.geomBoundingBoxOrigin[0]) >> 16) & 0xffff;
  geomBoundingBoxOrigin_x_lower = (*(Int*)(&gbh.geomBoundingBoxOrigin[0])) & 0xffff;
  m_bac->com_bsw_write_se(&m_bitStream, geomBoundingBoxOrigin_x_upper);
  m_bac->com_bsw_write1(&m_bitStream, 1);
  m_bac->com_bsw_write_se(&m_bitStream, geomBoundingBoxOrigin_x_lower);
  m_bac->com_bsw_write1(&m_bitStream, 1);

  geomBoundingBoxOrigin_y_upper = (*(Int*)(&gbh.geomBoundingBoxOrigin[1]) >> 16) & 0xffff;
  geomBoundingBoxOrigin_y_lower = (*(Int*)(&gbh.geomBoundingBoxOrigin[1])) & 0xffff;
  m_bac->com_bsw_write_se(&m_bitStream, geomBoundingBoxOrigin_y_upper);
  m_bac->com_bsw_write1(&m_bitStream, 1);
  m_bac->com_bsw_write_se(&m_bitStream, geomBoundingBoxOrigin_y_lower);
  m_bac->com_bsw_write1(&m_bitStream, 1);

  geomBoundingBoxOrigin_z_upper = (*(Int*)(&gbh.geomBoundingBoxOrigin[2]) >> 16) & 0xffff;
  geomBoundingBoxOrigin_z_lower = (*(Int*)(&gbh.geomBoundingBoxOrigin[2])) & 0xffff;
  m_bac->com_bsw_write_se(&m_bitStream, geomBoundingBoxOrigin_z_upper);
  m_bac->com_bsw_write1(&m_bitStream, 1);
  m_bac->com_bsw_write_se(&m_bitStream, geomBoundingBoxOrigin_z_lower);
  m_bac->com_bsw_write1(&m_bitStream, 1);

  nodeSizeLog2_x_upper = (*(Int*)(&gbh.nodeSizeLog2[0]) >> 16) & 0xffff;
  nodeSizeLog2_x_lower = (*(Int*)(&gbh.nodeSizeLog2[0])) & 0xffff;
  m_bac->com_bsw_write_ue(&m_bitStream, nodeSizeLog2_x_upper);
  m_bac->com_bsw_write1(&m_bitStream, 1);
  m_bac->com_bsw_write_ue(&m_bitStream, nodeSizeLog2_x_lower);
  m_bac->com_bsw_write1(&m_bitStream, 1);

  nodeSizeLog2_y_upper = (*(Int*)(&gbh.nodeSizeLog2[1]) >> 16) & 0xffff;
  nodeSizeLog2_y_lower = (*(Int*)(&gbh.nodeSizeLog2[1])) & 0xffff;
  m_bac->com_bsw_write_ue(&m_bitStream, nodeSizeLog2_y_upper);
  m_bac->com_bsw_write1(&m_bitStream, 1);
  m_bac->com_bsw_write_ue(&m_bitStream, nodeSizeLog2_y_lower);
  m_bac->com_bsw_write1(&m_bitStream, 1);

  nodeSizeLog2_z_upper = (*(Int*)(&gbh.nodeSizeLog2[2]) >> 16) & 0xffff;
  nodeSizeLog2_z_lower = (*(Int*)(&gbh.nodeSizeLog2[2])) & 0xffff;
  m_bac->com_bsw_write_ue(&m_bitStream, nodeSizeLog2_z_upper);
  m_bac->com_bsw_write1(&m_bitStream, 1);
  m_bac->com_bsw_write_ue(&m_bitStream, nodeSizeLog2_z_lower);
  m_bac->com_bsw_write1(&m_bitStream, 1);

  geomNumPoints_upper = (*(UInt*)(&gbh.geomNumPoints) >> 16) & 0xffff;
  geomNumPoints_lower = (*(UInt*)(&gbh.geomNumPoints)) & 0xffff;
  m_bac->com_bsw_write_ue(&m_bitStream, geomNumPoints_upper);
  m_bac->com_bsw_write1(&m_bitStream, 1);
  m_bac->com_bsw_write_ue(&m_bitStream, geomNumPoints_lower);
  m_bac->com_bsw_write1(&m_bitStream, 1);

  m_bac->com_bsw_write_byte_align(&m_bitStream);
}

Void TEncBacTop::codeAPS(const AttributeParameterSet& aps) {
  m_bac->com_bsw_write1(&m_bitStream, aps.withColor);
  m_bac->com_bsw_write1(&m_bitStream, aps.withRef);
  if (aps.withColor) {
    m_bac->com_bsw_write_ue(&m_bitStream, aps.maxNumOfNeighbours);
    m_bac->com_bsw_write1(&m_bitStream, aps.crossComponentPred);
  }
  m_bac->com_bsw_write_ue(&m_bitStream, aps.numOflevelOfDetail);
  m_bac->com_bsw_write_ue(&m_bitStream, aps.maxNumOfPredictNeighbours);
  m_bac->com_bsw_write1(&m_bitStream, aps.intraLodFlag);
  m_bac->com_bsw_write_byte_align(&m_bitStream);
}

Void TEncBacTop::codeABH(const AttributeBrickHeader& ah) {
  /*m_bac->com_bsw_write_ue(&m_bitStream, abh.abhID);*/
  m_bac->com_bsw_write_ue(&m_bitStream, ah.sliceID);
  m_bac->com_bsw_write_byte_align(&m_bitStream);
}

Void TEncBacTop::encodeOccupancyCode(const UInt& occupancyCode, const UInt& occupancySkip,
                                     const TComGeomContext& geomCtx) {
  auto p_ctx = p_aec->p_ctx_set->ctx_occupancy;

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
    UInt mask = 1 << i;

    Bool bit = !!(occupancyCode & mask);
    m_bac->biari_encode_symbol_aec(p_aec, bit, &p_ctx[ctxParent[i]][ctxChildNode[i]]);

    ///< update adjacent child node status
    if (bit) {
      for (Int k = 0; k < 3; k++) {
        assert(!((ctxChildNode[adjacentIdx[i][k]] >> adjacentPosIdx[i][k]) & 1));
        ctxChildNode[adjacentIdx[i][k]] |= 1 << adjacentPosIdx[i][k];
      }
    }
  }

  m_bitStream.cur = p_aec->p;
}

Void TEncBacTop::encodeOccupancyCode(const UInt& occupancyCode,
                                     const TComOctreePartitionParams& params, uint32_t* bit_pattern,
                                     uint32_t* update_pattern, int pattern_big) {
  int bit_order[8] = {0, 2, 4, 6, 1, 3, 5, 7};

  //================Todo:update the pattern============
  for (Int i = 0; i < 8; i++) {
    int bitIdx = bit_order[i];
    context_t* bit_ctx = NULL;
    int pattern = 0;
    if ((bit_pattern[bitIdx] == 0) && (update_pattern[bitIdx] == 0)) {
      bit_ctx = Enc_context_idx(bitIdx);
      pattern = pattern_big;
    } else {
      bit_ctx = Enc_context_idx(0, update_pattern[bitIdx]);
      pattern = bit_pattern[bitIdx];
    }

    if (params.bitSkip[bitIdx]) {
      continue;
    }

    UInt mask = 1 << bitIdx;
    m_bac->biari_encode_symbol_aec(p_aec, !!(occupancyCode & mask), bit_ctx + pattern);
  }

  m_bitStream.cur = p_aec->p;
}

Void TEncBacTop::encodeSinglePointIndex(const V3<UInt>& pos, V3<UInt> nodeSizeLog2) {
  while (nodeSizeLog2 > 0) {
    for (Int i = 0; i < 3; ++i) {
      if (nodeSizeLog2[i] > 0) {
        const auto bitMask = 1 << (nodeSizeLog2[i] - 1);
        m_bac->biari_encode_symbol_eq_prob_aec(p_aec, !!(pos[i] & bitMask));
        --nodeSizeLog2[i];
      }
    }
  }
  m_bitStream.cur = p_aec->p;
}

Void TEncBacTop::encodeDuplicateNumber(const UInt& dupNum) {
  assert(dupNum >= 1);
  auto p_ctx = &p_aec->p_ctx_set->ctx_geom_num_dup_eq1;
  m_bac->biari_encode_symbol_aec(p_aec, dupNum == 1, p_ctx);
  if (dupNum > 1)
    m_bac->sbac_write_ue_ep(&m_bitStream, dupNum - 2, p_aec);
}

Void TEncBacTop::encodeAttrCacheIndex(const UInt& cacheIdx) {
  for (int i = 0; i < ATTRIBUTE_CACHE_SIZE_BIT; i++) {
    m_bac->biari_encode_symbol_aec(p_aec, (cacheIdx >> i) & 0x01,
                                   &(p_aec->p_ctx_set->ctx_attr_cache_idx[i]));
  }
  m_bitStream.cur = p_aec->p;
}

Void TEncBacTop::codeAttributerResidual(const int64_t& delta, const int componentId) {
  int ctx_id = componentId;
  m_residualStats.collectStats(abs(delta));

  //Encode significant bit
  m_bac->biari_encode_symbol_aec(p_aec, delta == 0,
                                 &p_aec->p_ctx_set->ctx_attr_residual_eq0[ctx_id]);

  if (delta != 0) {
    int sign_bit = (delta > 0) ? 1 : 0;
    int abs_delta = (delta > 0) ? delta : -delta;
    m_bac->biari_encode_symbol_eq_prob_aec(p_aec, sign_bit);

    m_bac->biari_encode_symbol_aec(p_aec, abs_delta == 1,
                                   &p_aec->p_ctx_set->ctx_attr_residual_eq1[ctx_id]);
    if (abs_delta > 1) {
      m_bac->biari_encode_symbol_aec(p_aec, abs_delta == 2,
                                     &p_aec->p_ctx_set->ctx_attr_residual_eq2[ctx_id]);
      if (abs_delta > 2) {
        m_bac->biari_encode_symbol_aec(p_aec, abs_delta == 3,
                                       &p_aec->p_ctx_set->ctx_attr_residual_eq3[ctx_id]);
        if (abs_delta > 3) {
          m_bac->biari_encode_symbol_aec(p_aec, abs_delta == 4,
                                         &p_aec->p_ctx_set->ctx_attr_residual_eq4[ctx_id]);
          if (abs_delta > 4) {
            m_bac->biari_encode_symbol_aec(p_aec, abs_delta == 5,
                                           &p_aec->p_ctx_set->ctx_attr_residual_eq5[ctx_id]);
            if (abs_delta > 5) {
              m_bac->biari_encode_symbol_aec(p_aec, abs_delta == 6,
                                             &p_aec->p_ctx_set->ctx_attr_residual_eq6[ctx_id]);
              if (abs_delta > 6) {
                m_bac->biari_encode_symbol_aec(p_aec, abs_delta == 7,
                                               &p_aec->p_ctx_set->ctx_attr_residual_eq7[ctx_id]);
                if (abs_delta > 7) {
                  m_bac->biari_encode_symbol_aec(p_aec, abs_delta == 8,
                                                 &p_aec->p_ctx_set->ctx_attr_residual_eq8[ctx_id]);
                  if (abs_delta > 8) {
                    m_bac->biari_encode_symbol_aec(
                      p_aec, abs_delta == 9, &p_aec->p_ctx_set->ctx_attr_residual_eq9[ctx_id]);
                    if (abs_delta > 9) {
                      m_bac->biari_encode_symbol_aec(
                        p_aec, abs_delta == 10, &p_aec->p_ctx_set->ctx_attr_residual_eq10[ctx_id]);
                      if (abs_delta > 10) {
                        m_bac->biari_encode_symbol_aec(
                          p_aec, abs_delta == 11,
                          &p_aec->p_ctx_set->ctx_attr_residual_eq11[ctx_id]);
                        if (abs_delta > 11) {
                          m_bac->biari_encode_symbol_aec(
                            p_aec, abs_delta == 12,
                            &p_aec->p_ctx_set->ctx_attr_residual_eq12[ctx_id]);
                        
		                     if (abs_delta > 12) {
                             m_bac->sbac_write_ue_ep(&m_bitStream, abs_delta - 13, p_aec);
      }
    }
  }
	}}}}}}}}}}
  m_bitStream.cur = p_aec->p;
}

Void TEncBacTop::encodeTerminationFlag(Bool terminateFlag) {
  m_bac->biari_encode_symbol_final_aec(p_aec, terminateFlag ? 1 : 0);
  m_bac->aec_done(p_aec);

  m_bitStream.cur = p_aec->p;
}

Void TEncBacTop::encodeFinish() {
  if (m_bac)
    m_bac->enc_sbac_finish(&m_bitStream);

  m_bac->Demulate(&m_bitStream);

  m_bac->com_bsw_deinit(&m_bitStream);  ///< flush the buffer
}

context_t* TEncBacTop::Enc_context_idx(int idx) {
  switch (idx) {
  case 0:
    return p_aec->p_ctx_set->b0;
    break;
  case 1:
    return p_aec->p_ctx_set->b1;
    break;
  case 2:
    return p_aec->p_ctx_set->b2;
    break;
  case 3:
    return p_aec->p_ctx_set->b3;
    break;
  case 4:
    return p_aec->p_ctx_set->b4;
    break;
  case 5:
    return p_aec->p_ctx_set->b5;
    break;
  case 6:
    return p_aec->p_ctx_set->b6;
    break;
  case 7:
    return p_aec->p_ctx_set->b7;
    break;
  }
}
context_t* TEncBacTop::Enc_context_idx(int idx, int update_pattern) {
  switch (idx) {
  case 0:
    return p_aec->p_ctx_set->B0[update_pattern];
    break;
  case 1:
    return p_aec->p_ctx_set->B1[update_pattern];
    break;
  case 2:
    return p_aec->p_ctx_set->B2[update_pattern];
    break;
  case 3:
    return p_aec->p_ctx_set->B3[update_pattern];
    break;
  case 4:
    return p_aec->p_ctx_set->B4[update_pattern];
    break;
  case 5:
    return p_aec->p_ctx_set->B5[update_pattern];
    break;
  case 6:
    return p_aec->p_ctx_set->B6[update_pattern];
    break;
  case 7:
    return p_aec->p_ctx_set->B7[update_pattern];
    break;
  }
}

TEncBacTop::TEncBacTop() {
  m_bac = nullptr;
  reset();
}

TEncBacTop::~TEncBacTop() {
  if (m_bac) {
    delete m_bac;
    m_bac = nullptr;
  }
}

Void TEncBacTop::setBitstreamBuffer(TComBufferChunk& buffer) {
  buffer.allocateBuffSize();
  m_bac->com_bsw_init(&m_bitStream, (UInt8*)buffer.addr, (UInt8*)buffer.addr2, buffer.bsize, NULL);
  if (buffer.getBufferType() == BufferChunkType::BCT_GEOM ||
      buffer.getBufferType() == BufferChunkType::BCT_ATTR) {
    m_bac->aec_start(p_aec, m_bitStream.beg, m_bitStream.end, 1);
  }
}

Void TEncBacTop::initBac() {
  m_bac = new TEncBacCore;
  m_bac->enc_sbac_init();
}

Void TEncBacTop::reset() {
  if (m_bac) {
    delete m_bac;
    m_bac = nullptr;
  }
  m_bitStream.init();
  aec.init();
  p_aec = &aec;
}

UInt64 TEncBacTop::getBitStreamLength() {
  return m_bitStream.cur - m_bitStream.beg;
}

///< \{
