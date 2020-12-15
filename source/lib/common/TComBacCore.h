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

#include "common/TComBitStream.h"
#include "common/TComRom.h"

///< \in TLibCommon \{

/**
 * Class TComBacCore
 * entropy encoder engine
 */

#if defined(_MSC_VER)
#pragma warning(disable : 4324) /* disable warning C4324: 由于 \
                                   __declspec(align())，结构被填充 */
#define DECLARE_ALIGNED(var, n) __declspec(align(n)) var
#else
#define DECLARE_ALIGNED(var, n) var __attribute__((aligned(n)))
#endif
#define ALIGN32(var) DECLARE_ALIGNED(var, 32)
#define ALIGN16(var) DECLARE_ALIGNED(var, 16)

class aec_t {
public:
  ALIGN16(uint8_t* p_start); /* actual buffer for written bytes */
  /* bitstream */
  uint8_t* p;                   /* pointer to byte written currently */
  uint8_t* p_end;               /* end of actual buffer for written bytes */
  uint32_t reg_flush_bits;      /* register: flushing bits (not written into byte
                              buffer) */
  uint32_t num_left_flush_bits; /* number of bits in \ref{reg_flush_bits} could
                                   be used */

  /* AEC codec */
  uint32_t i_low;            /* low */
  uint32_t i_t1;             /* t1 */
  uint32_t i_bits_to_follow; /* current bit counter to follow */

  /* context */
  ctx_set_t* p_ctx_set; /* can reference other aec_t object */
  ctx_set_t ctx_set;    /* context models for AEC (current object) */

  /*decoder*/
  ALIGN32(uint8_t* p_buffer);
  uint64_t i_byte_buf;
  int i_byte_pos;
  int i_bytes;
  int8_t i_bits_to_go;
  uint8_t b_bit_error; /* bit error in stream */
  uint8_t b_val_bound;
  uint8_t b_val_domain;  // is value in R domain 1 is R domain 0 is LG domain
  uint32_t i_s1;
  uint32_t i_value_s;
  uint32_t i_value_t;

  /* context */
  ctx_set_t syn_ctx;  // pointer to struct of context models

public:
  void init() {
    p_start = p = p_end = nullptr;
    p_buffer = nullptr;
  }

};  ///< END CLASS TComBacCore

///< \{
