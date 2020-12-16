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

#include "CommonDef.h"
#include "contributors.h"

///< \in TLibCommon
/**
 * struct for context management
 */
#define B_BITS 10
#define QUARTER (1 << (B_BITS - 2))

#define MAKE_CONTEXT(lg_pmps, mps, cycno) \
  (((uint16_t)(cycno) << 12) | ((uint16_t)(mps) << 0) | (uint16_t)(lg_pmps << 1))

typedef union context_t {
  struct {
    unsigned MPS : 1;       // 1  bit
    unsigned LG_PMPS : 11;  // 11 bits
    unsigned cycno : 2;     // 2  bits
  };
  uint16_t v;
} context_t;

#define NUM_OCCUPANCY_CHILD_CTX (1 << 3)
#define NUM_OCCUPANCY_CTX 20
#define NUM_child_CTX 10

typedef struct ctx_set_t {
  context_t ctx_occupancy[1 << 5][NUM_OCCUPANCY_CHILD_CTX];

  context_t ctx_geom_num_dup_eq1;

  context_t ctx_attr_cache_idx[ATTRIBUTE_CACHE_SIZE_BIT];

  context_t ctx_attr_residual_eq0[3];
  context_t ctx_attr_residual_eq1[3];
  context_t ctx_attr_residual_eq2[3];
  context_t ctx_attr_residual_eq3[3];
  context_t ctx_attr_residual_eq4[3];

  context_t b0[NUM_OCCUPANCY_CTX];
  context_t b1[NUM_OCCUPANCY_CTX];
  context_t b2[NUM_OCCUPANCY_CTX];
  context_t b3[NUM_OCCUPANCY_CTX];
  context_t b4[NUM_OCCUPANCY_CTX];
  context_t b5[NUM_OCCUPANCY_CTX];
  context_t b6[NUM_OCCUPANCY_CTX];
  context_t b7[NUM_OCCUPANCY_CTX];
  context_t B0[NUM_OCCUPANCY_CTX][NUM_child_CTX];
  context_t B1[NUM_OCCUPANCY_CTX][NUM_child_CTX];
  context_t B2[NUM_OCCUPANCY_CTX][NUM_child_CTX];
  context_t B3[NUM_OCCUPANCY_CTX][NUM_child_CTX];
  context_t B4[NUM_OCCUPANCY_CTX][NUM_child_CTX];
  context_t B5[NUM_OCCUPANCY_CTX][NUM_child_CTX];
  context_t B6[NUM_OCCUPANCY_CTX][NUM_child_CTX];
  context_t B7[NUM_OCCUPANCY_CTX][NUM_child_CTX];

} ctx_set_t;

///< \}
