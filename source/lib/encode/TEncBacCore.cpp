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

#include "TEncBacCore.h"
#include "common/TComRom.h"
#include <algorithm>

///< \in TLibEncoder \{

/**
 * Class TEncBacCore
 * entropy encoder engine
 */

static Int com_bsw_flush(COM_BS* bs) {
  Int bytes = bs->com_bsw_get_sink_byte();
  while (bytes--) {
    *bs->cur++ = (bs->code >> 24) & 0xFF;
    bs->code <<= 8;
  }
  return 0;
}

//////////////////////////////////////////////////////////////////////////
// Public class functions (entropy encoder)
//////////////////////////////////////////////////////////////////////////

TEncBacCore::TEncBacCore()
  : range(0)
  , code(0)
  , left_bits(0)
  , stacked_ff(0)
  , pending_byte(0)
  , is_pending_byte(0)
  , bitcounter(0)
  , is_bitcount(0) {}

Void TEncBacCore::enc_sbac_init() {
  range = 0x1FF;
  code = 0;
  left_bits = 23;
  pending_byte = 0;
  is_pending_byte = 0;
  stacked_ff = 0;
}

int TEncBacCore::aec_get_written_bits(aec_t* p_aec) {
  return (int)(((p_aec->p - p_aec->p_start) << 3) + p_aec->i_bits_to_follow + NUM_FLUSH_BITS -
               p_aec->num_left_flush_bits);
}

Void TEncBacCore::init_contexts(aec_t* p_aec) {
  const uint16_t lg_pmps = ((QUARTER << LG_PMPS_SHIFTNO) - 1);
  uint16_t v = MAKE_CONTEXT(lg_pmps, 0, 0);
  uint16_t* d = (uint16_t*)&p_aec->ctx_set;
  int ctx_cnt = sizeof(ctx_set_t) / sizeof(uint16_t);

  while (ctx_cnt-- != 0) {
    *d++ = v;
  }
  p_aec->p_ctx_set = &p_aec->ctx_set;
}

/* ---------------------------------------------------------------------------
 * initializes the aec_t for the arithmetic coder
 */
Void TEncBacCore::aec_start(aec_t* p_aec, uint8_t* p_bs_start, uint8_t* p_bs_end, int b_writing) {
  p_aec->p_start = p_bs_start;
  p_aec->p = p_bs_start;
  p_aec->p_end = p_bs_end;
  p_aec->i_low = 0;
  p_aec->i_t1 = 0xFF;
  p_aec->i_bits_to_follow = 0;

  p_aec->num_left_flush_bits = NUM_FLUSH_BITS + 1;  // to swallow first redundant bit
  p_aec->reg_flush_bits = 0;
  if (b_writing) {
    memset(p_aec->p_start, 0, p_bs_end - p_bs_start);
  }

  /* init contexts */
  init_contexts(p_aec);
}

Void TEncBacCore::enc_sbac_finish(COM_BS* bs) {
  TEncBacCore* sbac = this;
  if (sbac->code >> (32 - sbac->left_bits)) {
    assert(sbac->pending_byte != 0xff);
    com_bsw_write(bs, sbac->pending_byte + 1, 8);
    while (sbac->stacked_ff != 0) {
      com_bsw_write(bs, 0x00, 8);
      sbac->stacked_ff--;
    }
    sbac->code -= 1 << (32 - sbac->left_bits);
  } else {
    com_bsw_write(bs, sbac->pending_byte, 8);
    while (sbac->stacked_ff != 0) {
      com_bsw_write(bs, 0xFF, 8);
      sbac->stacked_ff--;
    }
  }
  sbac->code |= (1 << 7);
  com_bsw_write(bs, sbac->code >> 8, 24 - sbac->left_bits);

  //if ((23 - sbac->left_bits) % 8)
  if (
    (24 - sbac->left_bits) %
    8)  // write the last byte of low in the end of CABAC, if the number of used bits (23 - left_bits) + 1 is not exactly bytes (Nx8), corresponding to bits_Needed != 0
  {
    com_bsw_write(bs, sbac->code, 8);
  }

  //add termination slice padding bits
  com_bsw_write_byte_align(bs);
}

Void TEncBacCore::bitstr_flush_bits(aec_t* p_aec) {
  switch (NUM_FLUSH_BITS) {
  case 24:
    p_aec->p[0] = (uint8_t)(p_aec->reg_flush_bits >> 16);
    p_aec->p[1] = (uint8_t)(p_aec->reg_flush_bits >> 8);
    p_aec->p[2] = (uint8_t)(p_aec->reg_flush_bits);
    p_aec->p += 3;
    break;
  case 16:
    p_aec->p[0] = (uint8_t)(p_aec->reg_flush_bits >> 8);
    p_aec->p[1] = (uint8_t)(p_aec->reg_flush_bits);
    p_aec->p += 2;
    break;
  case 8:
    p_aec->p[0] = (uint8_t)p_aec->reg_flush_bits;
    p_aec->p += 1;
    break;
  default:
    fprintf(stderr, "Unsupported number of flush bits %d\n", NUM_FLUSH_BITS);
    assert(0);
    break;
  }

  p_aec->reg_flush_bits = 0;
}

Void TEncBacCore::bitstt_put_one_bit_and_remainder(aec_t* p_aec, const int b) {
  uint32_t N = 1 + p_aec->i_bits_to_follow;  // 总共输出的比特数

  if (
    N >
    p_aec
      ->num_left_flush_bits) { /* 编码的比特数超过当前码流字节中剩余的比特数
                                    */
    int header_bits = p_aec->num_left_flush_bits;  // 当前码流最后一个字节剩余位的数量
    uint32_t header_byte = (1 << (header_bits - 1)) - (!b);  // 剩余位的填充值
    int num_left_bytes = (N - header_bits) >> 3;  // 除开当前字节外，剩余应该填充的整字节数
    int num_left_bits = N - header_bits - (num_left_bytes << 3);  // 多余的比特数

    p_aec->reg_flush_bits |= header_byte;
    bitstr_flush_bits(p_aec);
    p_aec->num_left_flush_bits = NUM_FLUSH_BITS - num_left_bits;

    if (b == 0) {
      /* b 为零时中间的bits全部填充 1 */
      while (num_left_bytes != 0) {
        *(p_aec->p) = 0xff;
        p_aec->p++;
        num_left_bytes--;
      }
      /* 最后填充 num_left_bits 位到 reg_flush_bits 的最高位 */
      p_aec->reg_flush_bits = 0xffu >> (8 - num_left_bits) << p_aec->num_left_flush_bits;
    } else {
      p_aec->p += num_left_bytes;
    }
  } else { /* 当前需要输出的bit数量小于码流中写入字节剩余的bit数量 */
    uint32_t bits = (1 << p_aec->i_bits_to_follow) - (!b);  // 输出的比特组成的二进制值

    p_aec->reg_flush_bits |= bits << (p_aec->num_left_flush_bits - N);
    p_aec->num_left_flush_bits -= N;
    if (p_aec->num_left_flush_bits == 0) {
      bitstr_flush_bits(p_aec);
      p_aec->reg_flush_bits = 0;
      p_aec->num_left_flush_bits = NUM_FLUSH_BITS;
    }
  }
  p_aec->i_bits_to_follow = 0;
}

int TEncBacCore::aec_get_shift(uint32_t v) {
  int i;

  for (i = 0; !(v & 0x100); i++) {
    v <<= 1;
  }

  return i;
}

Void TEncBacCore::biari_encode_symbol_aec(aec_t* p_aec, uint8_t symbol, context_t* p_ctx) {
  uint32_t lg_pmps = p_ctx->LG_PMPS;
  uint32_t t = p_aec->i_t1;
  uint32_t low = p_aec->i_low;
  const uint32_t lg_pmps_shifted = lg_pmps >> LG_PMPS_SHIFTNO;
  int s = (t < lg_pmps_shifted);

  if (symbol == p_ctx->MPS) {  // MPS happens
    if (s) {
      if (low & (1 << 9)) {
        bitstt_put_one_bit_and_remainder(p_aec, 1);
      } else if (low & (1 << 8)) {
        p_aec->i_bits_to_follow++;
        low &= ((1 << 8) ^ 0xFFFFFFFF);
      } else {
        bitstt_put_one_bit_and_remainder(p_aec, 0);
      }
      low <<= 1;
    }

    t = (t - lg_pmps_shifted) & 0xFF;
    p_ctx->v = g_tab_ctx_mps[p_ctx->v].v;
  } else {  // LPS
    int shift;
    uint32_t low_buf = (low << s) + 256 + ((t - lg_pmps_shifted) & 0xFF);
    uint32_t bitstogo = 9 + s;
    uint32_t bit_oa;

    t = ((-s) & t) + lg_pmps_shifted;

    shift = aec_get_shift(t);

    t <<= shift;
    s += shift;

    // left shift s2 bits
    bit_oa = ((low_buf >> bitstogo) & 1);

    while (s-- != 0) {
      uint32_t bit_o = bit_oa;
      bitstogo--;
      bit_oa = ((low_buf >> bitstogo) & 1);
      if (bit_o) {
        bitstt_put_one_bit_and_remainder(p_aec, 1);
      } else if (bit_oa) {  // 01

        p_aec->i_bits_to_follow++;
        bit_oa = 0;
      } else {  // 00

        bitstt_put_one_bit_and_remainder(p_aec, 0);
      }
    }

    t &= 0xff;
    low = (low_buf << shift) & ((bit_oa << 9) | 0x1ff);

    p_ctx->v = g_tab_ctx_lps[p_ctx->v].v;
  }

  p_aec->i_t1 = t;
  p_aec->i_low = low;
}

Void TEncBacCore::bitstr_put_one_bit(aec_t* p_aec, uint32_t b) {
  p_aec->reg_flush_bits |= ((b) << --p_aec->num_left_flush_bits);
  if (!p_aec->num_left_flush_bits) {
    bitstr_flush_bits(p_aec);
    p_aec->num_left_flush_bits = NUM_FLUSH_BITS;
  }
}
Void TEncBacCore::bitstr_end_stream(aec_t* p_aec) {
  if (p_aec->num_left_flush_bits == NUM_FLUSH_BITS) {
    return;
  }

  switch (NUM_FLUSH_BITS - p_aec->num_left_flush_bits) {
  case 24:
    p_aec->p[0] = (uint8_t)(p_aec->reg_flush_bits >> (NUM_FLUSH_BITS - 8));
    p_aec->p[1] = (uint8_t)(p_aec->reg_flush_bits >> (NUM_FLUSH_BITS - 16));
    p_aec->p[2] = (uint8_t)(p_aec->reg_flush_bits >> (NUM_FLUSH_BITS - 24));
    p_aec->p += 3;
    break;
  case 16:
    p_aec->p[0] = (uint8_t)(p_aec->reg_flush_bits >> (NUM_FLUSH_BITS - 8));
    p_aec->p[1] = (uint8_t)(p_aec->reg_flush_bits >> (NUM_FLUSH_BITS - 16));
    p_aec->p += 2;
    break;
  case 8:
    p_aec->p[0] = (uint8_t)(p_aec->reg_flush_bits >> (NUM_FLUSH_BITS - 8));
    p_aec->p += 1;
    break;
  default:
    fprintf(stderr, "Un-aligned tail bits %d\n", p_aec->num_left_flush_bits);
    assert(0);
    break;
  }

  p_aec->num_left_flush_bits = NUM_FLUSH_BITS;
}

Void TEncBacCore::aec_done(aec_t* p_aec) {
  int i;
  uint8_t bit_out_standing = (uint8_t)((p_aec->i_low >> (B_BITS - 1)) & 1);
  uint8_t bit_ending;

  bitstt_put_one_bit_and_remainder(p_aec, bit_out_standing);

  bit_ending = (uint8_t)((p_aec->i_low >> (B_BITS - 2)) & 1);
  bitstr_put_one_bit(p_aec, bit_ending);

  /* end of AEC */
  bitstr_put_one_bit(p_aec, 1);
  for (i = 0; i < 7; i++) {
    bitstr_put_one_bit(p_aec, 0);
  }

  /* write stuffing pattern */
  bitstr_put_one_bit(p_aec, 1);
  if (p_aec->num_left_flush_bits != NUM_FLUSH_BITS) {
    for (i = p_aec->num_left_flush_bits & 7; i > 0; i--) {
      bitstr_put_one_bit(p_aec, 0);
    }
  }

  /* end bitstream */
  bitstr_end_stream(p_aec);
}

void TEncBacCore::biari_encode_symbol_final_aec(aec_t* p_aec, uint8_t symbol) {
  uint32_t t = p_aec->i_t1;
  uint32_t low = p_aec->i_low;

  if (symbol) {
    int s = !t;
    uint32_t low_buf = (low << s) + 256 + ((t - 1) & 0xFF);
    uint32_t bitstogo = 9 + s;
    uint8_t bit_oa = (uint8_t)((low_buf >> bitstogo) & 1);
    uint8_t bit_o;

    s += 8;
    while (s-- > 0) {
      bit_o = bit_oa;
      bitstogo--;
      bit_oa = (uint8_t)((low_buf >> bitstogo) & 1);

      if (bit_o) {
        bitstt_put_one_bit_and_remainder(p_aec, 1);
      } else {
        if (bit_oa) {  // 01
          p_aec->i_bits_to_follow++;
          bit_oa = 0;
        } else {  // 00
          bitstt_put_one_bit_and_remainder(p_aec, 0);
        }
      }
    }

    p_aec->i_low = (low_buf << 8) & ((bit_oa << 9) | 0x1ff);
    p_aec->i_t1 = 0;

  } else {  // MPS
    if (!t) {
      if (low & (1 << 9)) {
        bitstt_put_one_bit_and_remainder(p_aec, 1);
      } else {
        if (low & (1 << 8)) {
          p_aec->i_bits_to_follow++;
          low &= ((1 << 8) ^ 0xFFFFFFFF);
        } else {
          bitstt_put_one_bit_and_remainder(p_aec, 0);
        }
      }
      p_aec->i_low = low << 1;
    }
    p_aec->i_t1 = (t - 1) & 0xff;
  }
}

Void TEncBacCore::biari_encode_symbol_eq_prob_aec(aec_t* p_aec, uint8_t symbol) {
  uint32_t low_buf = (p_aec->i_low << 1) + (symbol ? (p_aec->i_t1 + 256) : 0);
  uint8_t bit_oa = (uint8_t)((low_buf >> 9) & 1);

  // out bit
  if ((low_buf >> 10) & 1) {
    bitstt_put_one_bit_and_remainder(p_aec, 1);
  } else {
    if (bit_oa) {  // 01
      p_aec->i_bits_to_follow++;
      bit_oa = 0;
    } else {  // 00
      bitstt_put_one_bit_and_remainder(p_aec, 0);
    }
  }

  p_aec->i_low = low_buf & ((bit_oa << 9) | 0x1ff);
}
Void TEncBacCore::biari_encode_symbols_eq_prob_aec(aec_t* p_aec, uint32_t val, int len) {
  while (--len >= 0) {
    biari_encode_symbol_eq_prob_aec(p_aec, (uint8_t)((val >> len) & 1));
  }
}

Void TEncBacCore::sbac_write_unary_sym_ep(UInt32 sym, COM_BS* bs, aec_t* p_aec) {
  int symbol = sym;
  if (symbol > 31) {
    int exp_golomb_order = 0;

    biari_encode_symbol_final_aec(p_aec, 1);

    symbol -= 32;
    while (symbol >= (1 << exp_golomb_order)) {
      symbol -= (1 << exp_golomb_order);
      exp_golomb_order++;
    }
    biari_encode_symbols_eq_prob_aec(p_aec, 1, exp_golomb_order + 1);   // Exp-Golomb: prefix and 1
    biari_encode_symbols_eq_prob_aec(p_aec, symbol, exp_golomb_order);  // Exp-Golomb: suffix
  } else {
    biari_encode_symbol_final_aec(p_aec, 0);

    do {
      biari_encode_symbol_eq_prob_aec(p_aec, sym ? 0 : 1);
    } while (sym--);
  }
}

Void TEncBacCore::sbac_write_ue_ep(COM_BS* bs, UInt64 val, aec_t* p_aec) {
  Int len = 0;
  val++;
  for (UInt64 nn = val >> 1; nn != 0; len++)
    nn >>= 1;
  for (Int i = 0; i < len; i++)
    biari_encode_symbol_eq_prob_aec(p_aec, 0);
  for (UInt64 m = 1ULL << len; m; m >>= 1)
    biari_encode_symbol_eq_prob_aec(p_aec, !!(val & m));
}

//////////////////////////////////////////////////////////////////////////
// Public class functions (bypass encoder)
//////////////////////////////////////////////////////////////////////////

Void TEncBacCore::com_bsw_init(COM_BS* bs, UInt8* buf, UInt8* buftmp, Int size,
                               COM_BS_FN_FLUSH fn_flush) {
  bs->size = size;
  bs->beg = buf;
  bs->cur = buf;
  bs->buftmp = buftmp;
  bs->end = buf + size - 1;
  bs->code = 0;
  bs->leftbits = 32;
  bs->fn_flush = (fn_flush == NULL ? com_bsw_flush : fn_flush);
}

Void TEncBacCore::com_bsw_deinit(COM_BS* bs) {
  bs->fn_flush(bs);
}

Int TEncBacCore::com_bsw_write1(COM_BS* bs, Int val) {
  com_assert(bs);
  bs->leftbits--;
  bs->code |= ((val & 0x1) << bs->leftbits);
  if (bs->leftbits == 0) {
    com_assert_rv(bs->cur <= bs->end, -1);
    bs->fn_flush(bs);
    bs->code = 0;
    bs->leftbits = 32;
  }
  return 0;
}

Int TEncBacCore::com_bsw_write(COM_BS* bs, UInt32 val, Int len) /* len(1 ~ 32) */
{
  assert(len <= 32 && len >= 1);  ///< will cause mismatch if len larger than 32
  Int leftbits;
  com_assert(bs);
  leftbits = bs->leftbits;
  val <<= (32 - len);
  bs->code |= (val >> (32 - leftbits));
  if (len < leftbits) {
    bs->leftbits -= len;
  } else {
    com_assert_rv(bs->cur + 4 <= bs->end, -1);
    bs->leftbits = 0;
    bs->fn_flush(bs);
#if defined(X86F)
    /* on X86 machine, shift operation works properly when the value of the
       right operand is less than the number of bits of the left operand. */
    bs->code = (leftbits < 32 ? val << leftbits : 0);
#else
    bs->code = (val << leftbits);
#endif
    bs->leftbits = 32 - (len - leftbits);
  }
  return 0;
}

Void TEncBacCore::com_bsw_write_ue(COM_BS* bs, UInt64 val) {
  Int len = 0;
  val++;
  for (UInt64 nn = val >> 1; nn != 0; len++)
    nn >>= 1;
  for (Int i = 0; i < len; i++)
    com_bsw_write1(bs, 0);
  for (UInt64 m = 1ULL << len; m; m >>= 1)
    com_bsw_write1(bs, !!(val & m));
}

Void TEncBacCore::com_bsw_write_se(COM_BS* bs, Int64 val) {
  com_bsw_write_ue(bs, val <= 0 ? (-val << 1) : ((val << 1) - 1));
}

Void TEncBacCore::com_bsw_write_byte_align(COM_BS* bs) {
  while (!bs->com_is_byte_align())
    com_bsw_write1(bs, 1);
}

Void TEncBacCore::Demulate(COM_BS* bs) {
  UInt uiZeroCount = 0;
  bs->fn_flush(bs);
  bs->leftbits = bs->leftbits % 8;
  UInt current_bytes_size = bs->com_bsw_get_write_byte();

  //stuffing bit '1'
  Int stuffbitnum = bs->leftbits;
  if (stuffbitnum > 0) {
    bs->beg[current_bytes_size - 1] =
      bs->beg[current_bytes_size - 1] & (~(1 << (8 - stuffbitnum))) << stuffbitnum;
    bs->beg[current_bytes_size - 1] = bs->beg[current_bytes_size - 1] | (1 << (stuffbitnum - 1));
  } else {
    bs->beg[current_bytes_size++] = 0x80;
  }

  UInt uiWriteOffset = 0;
  UInt uiBitsWriteOffset = 0;
  UChar ucCurByte = bs->beg[0];
  /*demulate*/
  for (UInt uiReadOffset = 0; uiReadOffset < current_bytes_size; uiReadOffset++) {
    if (uiReadOffset > 0)
      ucCurByte = (bs->beg[uiReadOffset - 1] << (8 - uiBitsWriteOffset)) |
        (bs->beg[uiReadOffset] >> uiBitsWriteOffset);
    if (2 <= uiZeroCount && 0 == (ucCurByte & 0xfc)) {
      bs->buftmp[uiWriteOffset++] = 0x02;
      uiBitsWriteOffset += 2;
      uiZeroCount = 0;
      if (uiBitsWriteOffset >= 8) {
        uiBitsWriteOffset = 0;
        uiReadOffset--;
      }
      continue;
    }
    bs->buftmp[uiWriteOffset++] = ucCurByte;

    if (0 == ucCurByte) {
      uiZeroCount++;
    } else {
      uiZeroCount = 0;
    }
  }

  if (uiBitsWriteOffset != 0) {
    /*get the last several bits*/
    UChar ucCurByte = bs->beg[current_bytes_size - 1] << (8 - uiBitsWriteOffset);
    bs->buftmp[uiWriteOffset++] = ucCurByte;
  }

  for (UInt i = 0; i < uiWriteOffset; i++) {
    bs->beg[i] = bs->buftmp[i];
  }

  bs->cur = bs->beg + uiWriteOffset;
  bs->code = 0;
  bs->leftbits = 32;
}

///< \{
