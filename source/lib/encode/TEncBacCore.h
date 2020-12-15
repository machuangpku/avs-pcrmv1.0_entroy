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

///< \in TLibEncoder \{

/**
 * Class TEncBacCore
 * entropy encoder engine
 */
static const uint32_t NUM_FLUSH_BITS = 24;

class TEncBacCore {
public:
  UInt32 range;
  UInt32 code;
  Int left_bits;
  UInt32 stacked_ff;
  UInt32 pending_byte;
  UInt32 is_pending_byte;
  UInt32 bitcounter;
  UInt8 is_bitcount;

public:
  TEncBacCore();
  ///< entropy encoder (with context)
  Void enc_sbac_init();
  Void aec_start(aec_t* p_aec, uint8_t* p_bs_start, uint8_t* p_bs_end, int b_writing);
  Void bitstr_put_one_bit(aec_t* p_aec, uint32_t b);
  Void bitstr_end_stream(aec_t* p_aec);
  Void aec_done(aec_t* p_aec);
  Void init_contexts(aec_t* p_aec);

  int aec_get_written_bits(aec_t* p_aec);
  int aec_get_shift(uint32_t v);
  Void bitstr_flush_bits(aec_t* p_aec);
  Void bitstt_put_one_bit_and_remainder(aec_t* p_aec, const int b);
  Void biari_encode_symbol_aec(aec_t* p_aec, uint8_t symbol, context_t* p_ctx);
  Void biari_encode_symbol_final_aec(aec_t* p_aec, uint8_t symbol);
  Void biari_encode_symbol_eq_prob_aec(aec_t* p_aec, uint8_t symbol);
  Void biari_encode_symbols_eq_prob_aec(aec_t* p_aec, uint32_t val, int len);
  Void enc_sbac_finish(COM_BS* bsw);
  Void sbac_write_unary_sym_ep(UInt32 sym, COM_BS* bs, aec_t* p_aec);
  Void sbac_write_ue_ep(COM_BS* bs, UInt64 val, aec_t* p_aec);

  ///< bypass encoder
  Void com_bsw_init(COM_BS* bs, UInt8* buf, UInt8* buftmp, Int size, COM_BS_FN_FLUSH fn_flush);
  Void com_bsw_deinit(COM_BS* bs);  ///< write out all the remaining bits in the bitstream buffer
  Int com_bsw_write1(COM_BS* bs, Int val);
  Int com_bsw_write(COM_BS* bs, UInt32 val, Int len);
  Void com_bsw_write_ue(COM_BS* bs, UInt64 val);
  Void com_bsw_write_se(COM_BS* bs, Int64 val);
  Void com_bsw_write_byte_align(COM_BS* bs);

  Void Demulate(COM_BS* bs);  ///< prevent start code inside buffer chunk data

};  ///< END CLASS TEncBacCore

///< \{
