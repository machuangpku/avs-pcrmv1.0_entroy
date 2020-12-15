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

#include "TDecBacCore.h"
#include "TDecBacTop.h"
#include "common/TComRom.h"
#include <algorithm>

///< \in TLibDecoder \{

/**
 * Class TDecBacCore
 * entropy decoder engine
 */

static Int com_bsr_flush(COM_BS* bs) {
  Int byte = 4;
  Int shift = 24, remained;
  UInt32 code = 0;
  com_assert(byte);
  remained = (Int)(bs->end - bs->cur) + 1;
  if (byte > remained) {
    byte = remained;
  }
  if (byte <= 0) {
    bs->code = 0;
    bs->leftbits = 0;
    return -1;
  }
  bs->leftbits = byte << 3;
  bs->cur += byte;
  while (byte) {
    code |= *(bs->cur - byte) << shift;
    byte--;
    shift -= 8;
  }
  bs->code = code;
  return 0;
}

//////////////////////////////////////////////////////////////////////////
// Public class functions (bypass decoder)
//////////////////////////////////////////////////////////////////////////

TDecBacCore::TDecBacCore() {}

Void TDecBacCore::dec_sbac_init(COM_BS* bs) {}

int TDecBacCore::biari_decode_symbol_eq_prob(aec_t* p_aec) {
  if (p_aec->b_val_domain != 0 || (p_aec->i_s1 == AEC_VALUE_BOUND && p_aec->b_val_bound != 0)) {
    p_aec->i_s1 = 0;

    if (aec_get_next_bit(p_aec)) {
      return 0;
    }

    if (p_aec->i_value_t >= (256 + p_aec->i_t1)) {  // LPS
      p_aec->i_value_t -= (256 + p_aec->i_t1);
      return 1;
    } else {
      return 0;
    }
  } else {
    uint32_t s2 = p_aec->i_s1 + 1;
    uint32_t t2 = p_aec->i_t1;
    int is_LPS = s2 > p_aec->i_value_s ||
      ((s2 == p_aec->i_value_s && p_aec->i_value_t >= t2) && p_aec->b_val_bound == 0);

    p_aec->b_val_domain = (uint8_t)is_LPS;

    if (is_LPS) {  // LPS
      if (s2 == p_aec->i_value_s) {
        p_aec->i_value_t -= t2;
      } else {
        if (aec_get_next_bit(p_aec)) {
          return 0;
        }
        p_aec->i_value_t += 256 - t2;
      }
      return 1;
    } else {
      p_aec->i_s1 = s2;
      p_aec->i_t1 = t2;
      return 0;
    }
  }
}

#define FALSE 0
#define AEC_RETURN_ON_ERROR(ret_code)                           \
  if (p_aec->b_bit_error) {                                     \
    p_aec->b_bit_error = FALSE; /* reset error flag */          \
    /* davs2_log(h, DAVS2_LOG_ERROR, "aec decoding error."); */ \
    return (ret_code);                                          \
  }
UInt32 TDecBacCore::sbac_read_unary_sym_ep(COM_BS* bs, aec_t* p_aec) {
  UInt32 val = 0;
  UInt32 bin;
  int Level = 0;
  if (biari_decode_final(p_aec)) {
    int golomb_order = 0;
    int binary_symbol = 0;

    for (;;) {
      int l = biari_decode_symbol_eq_prob(p_aec);
      AEC_RETURN_ON_ERROR(-1);
      if (l) {
        break;
      }
      Level += (1 << golomb_order);
      golomb_order++;
    }

    while (golomb_order--) {
      // next binary part
      int sig = biari_decode_symbol_eq_prob(p_aec);
      binary_symbol |= (sig << golomb_order);
    }

    Level += binary_symbol;
    Level += 32;
    val = Level;
  } else {
    do {
      bin = !biari_decode_symbol_eq_prob(p_aec);
      val += bin;
    } while (bin);
  }
  return val;
}

UInt32 TDecBacCore::sbac_read_ue_ep(aec_t* p_aec) {
  UInt32 val = 0;
  int Level = 0;

  int golomb_order = 0;
  int binary_symbol = 0;

  for (;;) {
    int l = biari_decode_symbol_eq_prob(p_aec);
    AEC_RETURN_ON_ERROR(-1);
    if (l) {
      break;
    }
    Level += (1 << golomb_order);
    golomb_order++;
  }

  while (golomb_order--) {
    // next binary part
    int sig = biari_decode_symbol_eq_prob(p_aec);
    binary_symbol |= (sig << golomb_order);
  }

  Level += binary_symbol;
  val = Level;

  return val;
}

int TDecBacCore::aec_get_next_bit(aec_t* p_aec) {
  uint32_t next_bit;

  if (--p_aec->i_bits_to_go < 0) {
    int diff = p_aec->i_bytes - p_aec->i_byte_pos;
    uint8_t* p_buffer = p_aec->p_buffer + p_aec->i_byte_pos;

#if 1
    if (diff > 7) {
      p_aec->i_byte_buf = ((uint64_t)p_buffer[0] << 56) | ((uint64_t)p_buffer[1] << 48) |
        ((uint64_t)p_buffer[2] << 40) | ((uint64_t)p_buffer[3] << 32) |
        ((uint64_t)p_buffer[4] << 24) | ((uint64_t)p_buffer[5] << 16) |
        ((uint64_t)p_buffer[6] << 8) | (uint64_t)p_buffer[7];
      p_aec->i_bits_to_go = 63;
      p_aec->i_byte_pos += 8;
    } else if (diff > 0) {
      /* 一帧剩余码流长度小于8，这在一帧图像解码过程中只出现一次 */
      int i;
      p_aec->i_bits_to_go += (int8_t)(diff << 3);
      p_aec->i_byte_pos += (p_aec->i_bits_to_go + 1) >> 3;

      p_aec->i_byte_buf = 0;
      for (i = 0; i < diff; i++) {
        p_aec->i_byte_buf = (p_aec->i_byte_buf << 8) | p_buffer[i];
      }
    } else {
      p_aec->b_bit_error = 1;
      return 1;
    }
#else
    int i;
    if (diff > 8) {
      diff = 8;
    } else if (diff <= 0) {
      p_aec->b_bit_error = 1;
      return 1;
    }
    p_aec->i_bits_to_go += (diff << 3);
    p_aec->i_byte_pos += (p_aec->i_bits_to_go + 1) >> 3;

    p_aec->i_byte_buf = 0;
    for (i = 0; i < diff; i++) {
      p_aec->i_byte_buf = (p_aec->i_byte_buf << 8) | p_buffer[i];
    }
#endif
  }

  /* get next bit */
  next_bit = ((p_aec->i_byte_buf >> p_aec->i_bits_to_go) & 0x01);

  p_aec->i_value_t = (p_aec->i_value_t << 1) | next_bit;

  return 0;
}
int TDecBacCore::aec_get_next_n_bit(aec_t* p_aec, int num_bits) {
  if (p_aec->i_bits_to_go >= num_bits) {
    uint32_t next_bits;
    p_aec->i_bits_to_go -= (int8_t)num_bits;
    next_bits = (p_aec->i_byte_buf >> p_aec->i_bits_to_go) & ((1 << num_bits) - 1);

    p_aec->i_value_t = (p_aec->i_value_t << num_bits) | next_bits;

    return 0;
  } else {
    for (; num_bits != 0; num_bits--) {
      aec_get_next_bit(p_aec);
    }
    return p_aec->b_bit_error;
  }
}

Void TDecBacCore::update_ctx_lps(context_t* ctx) {
  ctx->v = g_tab_ctx_lps[ctx->v].v;
}

Void TDecBacCore::update_ctx_mps(context_t* ctx) {
  ctx->v = g_tab_ctx_mps[ctx->v].v;
}

UInt32 TDecBacCore::biari_decode_symbol(aec_t* p_aec, context_t* ctx) {
  uint32_t lg_pmps = ctx->LG_PMPS >> LG_PMPS_SHIFTNO;
  uint32_t t2;
  uint32_t s2;
  uint32_t s_flag;
  uint32_t i_value_s = p_aec->i_value_s;
  int bit = ctx->MPS;
  int is_LPS;

  // p_aec->i_value_t is in R domain  p_aec->i_s1=0 or p_aec->i_s1 ==
  // AEC_VALUE_BOUND
  if (p_aec->b_val_domain != 0 || (p_aec->i_s1 == AEC_VALUE_BOUND && p_aec->b_val_bound != 0)) {
    i_value_s = 0;
    p_aec->i_s1 = 0;

    while (p_aec->i_value_t < QUARTER && i_value_s < AEC_VALUE_BOUND) {
      if (aec_get_next_bit(p_aec)) {
        return 0;
      }
      i_value_s++;
    }

    p_aec->b_val_bound = p_aec->i_value_t < QUARTER;
    p_aec->i_value_t = p_aec->i_value_t & 0xff;
  }

  if (p_aec->i_value_s > AEC_VALUE_BOUND) {
    /// davs2_log(NULL, DAVS2_LOG_ERROR, "p_aec->i_value_s (>254).");
    p_aec->b_bit_error = 1;
    p_aec->i_value_s = i_value_s;
    return 0;
  }

  s_flag = p_aec->i_t1 < lg_pmps;
  s2 = p_aec->i_s1 + s_flag;
  t2 = p_aec->i_t1 - lg_pmps + (s_flag << 8);  // 8bits
  is_LPS =
    (s2 > i_value_s || (s2 == i_value_s && p_aec->i_value_t >= t2)) && p_aec->b_val_bound == 0;

  p_aec->b_val_domain = (uint8_t)is_LPS;

  if (is_LPS) {  // LPS
    uint32_t t_rlps = (s_flag == 0) ? (lg_pmps) : (p_aec->i_t1 + lg_pmps);
    int n_bits = 0;
    bit = !bit;

    if (s2 == i_value_s) {
      p_aec->i_value_t -= t2;
    } else {
      if (aec_get_next_bit(p_aec)) {
        return 0;
      }
      p_aec->i_value_t += 256 - t2;
    }

    // restore range
    while (t_rlps < QUARTER) {
      t_rlps <<= 1;
      n_bits++;
    }
    if (n_bits) {
      if (aec_get_next_n_bit(p_aec, n_bits)) {
        return 0;
      }
    }

    p_aec->i_s1 = 0;
    p_aec->i_t1 = t_rlps & 0xff;
    update_ctx_lps(ctx);
  } else {  // MPS
    p_aec->i_s1 = s2;
    p_aec->i_t1 = t2;
    update_ctx_mps(ctx);
  }

  p_aec->i_value_s = i_value_s;

  return bit;
}

int TDecBacCore::biari_decode_final(aec_t* p_aec) {
  // static context_t ctx = { (1 << LG_PMPS_SHIFTNO), 0, 0 };
  const uint32_t lg_pmps = 1;  // ctx.LG_PMPS >> LG_PMPS_SHIFTNO;
  uint32_t t2;
  uint32_t s2;
  uint32_t s_flag;
  int is_LPS;

  // p_aec->i_value_t is in R domain  p_aec->i_s1=0 or p_aec->i_s1 ==
  // AEC_VALUE_BOUND
  if (p_aec->b_val_domain != 0 || (p_aec->i_s1 == AEC_VALUE_BOUND && p_aec->b_val_bound != 0)) {
    p_aec->i_s1 = 0;
    p_aec->i_value_s = 0;

    while (p_aec->i_value_t < QUARTER && p_aec->i_value_s < AEC_VALUE_BOUND) {
      if (aec_get_next_bit(p_aec)) {
        return 0;
      }
      p_aec->i_value_s++;
    }

    p_aec->b_val_bound = p_aec->i_value_t < QUARTER;
    p_aec->i_value_t = p_aec->i_value_t & 0xff;
  }

  s_flag = p_aec->i_t1 < lg_pmps;
  s2 = p_aec->i_s1 + s_flag;
  t2 = p_aec->i_t1 - lg_pmps + (s_flag << 8);  // 8bits

  /* 返回值 */
  is_LPS = (s2 > p_aec->i_value_s || (s2 == p_aec->i_value_s && p_aec->i_value_t >= t2)) &&
    p_aec->b_val_bound == 0;
  p_aec->b_val_domain = (uint8_t)is_LPS;

  if (is_LPS) {  // LPS
    uint32_t t_rlps = 1;
    int n_bits = 0;

    if (s2 == p_aec->i_value_s) {
      p_aec->i_value_t -= t2;
    } else {
      if (aec_get_next_bit(p_aec)) {
        return 0;
      }
      p_aec->i_value_t += 256 - t2;
    }

    // restore range
    while (t_rlps < QUARTER) {
      t_rlps <<= 1;
      n_bits++;
    }
    if (n_bits) {
      if (aec_get_next_n_bit(p_aec, n_bits)) {
        return 0;
      }
    }

    p_aec->i_s1 = 0;
    p_aec->i_t1 = 0;
    // return 1;  // !ctx.MPS
  } else {  // MPS
    p_aec->i_s1 = s2;
    p_aec->i_t1 = t2;
    // return 0;  // ctx.MPS
  }

  return is_LPS;
}

//////////////////////////////////////////////////////////////////////////
// Public class functions (bypass decoder)
//////////////////////////////////////////////////////////////////////////
Void TDecBacCore::aec_init_contexts(aec_t* p_aec) {
  const uint16_t lg_pmps = ((QUARTER << LG_PMPS_SHIFTNO) - 1);
  uint16_t v = MAKE_CONTEXT(lg_pmps, 0, 0);
  uint16_t* d = (uint16_t*)&p_aec->syn_ctx;
  int ctx_cnt = sizeof(ctx_set_t) / sizeof(uint16_t);

  while (ctx_cnt-- != 0) {
    *d++ = v;
  }
}
int TDecBacCore::aec_start_decoding(aec_t* p_aec, uint8_t* p_start, int i_byte_pos, int i_bytes) {
  p_aec->p_buffer = p_start;
  p_aec->i_byte_pos = i_byte_pos;
  p_aec->i_bytes = i_bytes;
  p_aec->i_bits_to_go = 0;
  p_aec->b_bit_error = 0;
  p_aec->b_val_domain = 1;
  p_aec->i_s1 = 0;
  p_aec->i_t1 = QUARTER - 1;  // 0xff
  p_aec->i_value_s = 0;
  p_aec->i_value_t = 0;

  if (p_aec->i_bits_to_go < B_BITS - 1) {
    if (aec_get_next_n_bit(p_aec, B_BITS - 1)) {
      return 0;
    }
  }

  return 0;
}

Void TDecBacCore::com_bsr_init(COM_BS* bs, UInt8* buf, Int size, COM_BS_FN_FLUSH fn_flush) {
  bs->size = size;
  bs->cur = buf;
  bs->beg = buf;
  bs->end = buf + size - 1;
  bs->code = 0;
  bs->leftbits = 0;
  bs->fn_flush = (fn_flush == NULL) ? com_bsr_flush : fn_flush;
}

UInt32 TDecBacCore::com_bsr_read(COM_BS* bs, Int size) {
  UInt32 code = 0;
  com_assert(size > 0);
  if (bs->leftbits < size) {
    code = bs->code >> (32 - size);
    size -= bs->leftbits;
    if (bs->fn_flush(bs)) {
      return ((UInt32)-1);  ///< already reached the end of bitstream
    }
  }
  code |= bs->code >> (32 - size);
  COM_BSR_SKIP_CODE(bs, size);
  return code;
}

Int TDecBacCore::com_bsr_read1(COM_BS* bs) {
  Int code;
  if (bs->leftbits == 0) {
    if (bs->fn_flush(bs)) {
      return -1;  ///< already reached the end of bitstream
    }
  }
  code = (Int)(bs->code >> 31);
  bs->code <<= 1;
  bs->leftbits -= 1;
  return code;
}

UInt64 TDecBacCore::com_bsr_read_ue(COM_BS* bs) {
  Int len = 0;
  while (!com_bsr_read1(bs))
    len++;

  UInt64 v = 0;
  for (Int i = 0; i < len; i++) {
    v <<= 1;
    v |= com_bsr_read1(bs);
  }
  v += (1ULL << len);
  return v - 1;
}

Int64 TDecBacCore::com_bsr_read_se(COM_BS* bs) {
  com_assert(bs != NULL);
  UInt64 val = com_bsr_read_ue(bs);
  return ((val & 0x01) ? ((val + 1) >> 1) : -(Int64)(val >> 1));
}

UInt32 TDecBacCore::com_bsr_next(COM_BS* bs, Int size) {
  UInt32 code = 0;
  com_assert(size > 0);
  Int shift = 24, remained;
  UInt32 newcode = 0;
  Int byte = 4;
  UInt8* cur = bs->cur;
  if (bs->leftbits < size) {
    code = bs->code >> (32 - size);
    size -= bs->leftbits;
    remained = (Int)(bs->end - bs->cur) + 1;
    if (byte > remained) {
      byte = remained;
    }

    cur += byte;
    while (byte) {
      newcode |= *(cur - byte) << shift;
      byte--;
      shift -= 8;
    }
  } else {
    newcode = bs->code;
  }
  code |= newcode >> (32 - size);

  return code;
}

Void TDecBacCore::com_bsr_read_byte_align(COM_BS* bs) {
  while (!bs->com_is_byte_align()) {
    com_assert(com_bsr_read1(bs) == 1);
  }
}

UInt64 TDecBacCore::com_bsr_get_read_byte(COM_BS* bs) {
  return ((Int)((bs)->cur - (bs)->beg) - ((bs)->leftbits >> 3));
}

///< \{
