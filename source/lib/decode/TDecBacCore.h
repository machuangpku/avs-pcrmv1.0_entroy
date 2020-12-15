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
 *    nor the names of its contributors maybe used to endorse or promote
 * products derived from this software without specific prior written
 * permission.
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
#include "common/ContextModel.h"
#include "common/TComBacCore.h"
#include "common/TComBitStream.h"

///< \in TLibDecoder \{
enum aec_const_e {
  QUARTER_SHIFT = (B_BITS - 2),
  HALF = (1 << (B_BITS - 1)),
  AEC_VALUE_BOUND = 254, /* make sure rs1 will not overflow for 8-bit uint8_t */
};
/**
 * Class TDecBacCore
 * entropy decoder engine
 */

class TDecBacCore {
public:
  TDecBacCore();
  ///< entropy decoder (with context)
  Void dec_sbac_init(COM_BS* bs);
  Void aec_init_contexts(aec_t* p_aec);
  int aec_start_decoding(aec_t* p_aec, uint8_t* p_start, int i_byte_pos, int i_bytes);
  int aec_get_next_bit(aec_t* p_aec);
  UInt32 biari_decode_symbol(aec_t* p_aec, context_t* ctx);
  Void update_ctx_lps(context_t* ctx);
  Void update_ctx_mps(context_t* ctx);
  int aec_get_next_n_bit(aec_t* p_aec, int num_bits);
  int biari_decode_final(aec_t* p_aec);
  int biari_decode_symbol_eq_prob(aec_t* p_aec);
  UInt32 sbac_read_unary_sym_ep(COM_BS* bs, aec_t* p_aec);
  UInt32 sbac_read_ue_ep(aec_t* p_aec);

  ///< bypass decoder
  Void com_bsr_init(COM_BS* bs, UInt8* buf, Int size, COM_BS_FN_FLUSH fn_flush);
  UInt32 com_bsr_read(COM_BS* bs, Int size);
  UInt32 com_bsr_next(COM_BS* bs, Int size);
  Int com_bsr_read1(COM_BS* bs);
  UInt64 com_bsr_read_ue(COM_BS* bs);
  Int64 com_bsr_read_se(COM_BS* bs);
  Void com_bsr_read_byte_align(COM_BS* bs);
  UInt64 com_bsr_get_read_byte(COM_BS* bs);  ///< get number of byte consumed

};  ///> END CLASS TDecBacCore

#if defined(X86F)
/* on X86 machine, 32-bit shift means remaining of original value, so we
should set zero in that case. */
#define COM_BSR_SKIP_CODE(bs, size)     \
  com_assert((bs)->leftbits >= (size)); \
  if ((size) == 32) {                   \
    (bs)->code = 0;                     \
    (bs)->leftbits = 0;                 \
  } else {                              \
    (bs)->code <<= (size);              \
    (bs)->leftbits -= (size);           \
  }
#else
#define COM_BSR_SKIP_CODE(bs, size)     \
  com_assert((bs)->leftbits >= (size)); \
  (bs)->code <<= (size);                \
  (bs)->leftbits -= (size);
#endif

///< \{
