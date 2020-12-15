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

#include "TComRom.h"
#include "contributors.h"

///< \in TLibCommon

//////////////////////////////////////////////////////////////////////////
// Entropy coder
//////////////////////////////////////////////////////////////////////////

context_t g_tab_ctx_mps[4096 * 5]; /* [2 * lg_pmps + mps + cycno * 4096] */
context_t g_tab_ctx_lps[4096 * 5]; /* [2 * lg_pmps + mps + cycno * 4096] */
uint8_t tab_cwr[] = {
  3, 3, 4, 5, 5, 5, 5 /* 5, 5, 5, 5 */
};

uint16_t tab_lg_pmps_offset[] = {
  0, 0, 0, 197, 95, 46 /* 5, 5, 5, 5 */
};

/* ---------------------------------------------------------------------------
 */
Void init_aec_context_tab(void) {
  context_t ctx_i;
  context_t ctx_o;
  int cycno;
  int mps;

  /* init context table */
  ctx_i.v = 0;
  ctx_o.v = 0;

  /* mps */
  for (cycno = 0; cycno < 4; cycno++) {
    uint32_t cwr = tab_cwr[cycno];
    ctx_i.cycno = cycno;
    ctx_o.cycno = (uint8_t)AVS_MAX(cycno, 1);

    for (mps = 0; mps < 2; mps++) {
      ctx_i.MPS = (uint8_t)mps;
      ctx_o.MPS = (uint8_t)mps;
      for (ctx_i.LG_PMPS = 0; ctx_i.LG_PMPS <= 1024; ctx_i.LG_PMPS++) {
        uint32_t lg_pmps = ctx_i.LG_PMPS;
        lg_pmps -= (lg_pmps >> cwr) + (lg_pmps >> (cwr + 2));
        ctx_o.LG_PMPS = (uint16_t)lg_pmps;
        g_tab_ctx_mps[ctx_i.v].v = ctx_o.v;
      }
    }
  }

  /* lps */
  for (cycno = 0; cycno < 4; cycno++) {
    uint32_t cwr = tab_cwr[cycno];
    ctx_i.cycno = cycno;
    ctx_o.cycno = (uint8_t)AVS_MIN(cycno + 1, 3);

    for (mps = 0; mps < 2; mps++) {
      ctx_i.MPS = (uint8_t)mps;
      ctx_o.MPS = (uint8_t)mps;
      for (ctx_i.LG_PMPS = 0; ctx_i.LG_PMPS <= 1024; ctx_i.LG_PMPS++) {
        uint32_t lg_pmps = ctx_i.LG_PMPS + tab_lg_pmps_offset[cwr];
        if (lg_pmps >= (256 << LG_PMPS_SHIFTNO)) {
          lg_pmps = (512 << LG_PMPS_SHIFTNO) - 1 - lg_pmps;
          ctx_o.MPS = !mps;
        }
        ctx_o.LG_PMPS = (uint16_t)lg_pmps;
        g_tab_ctx_lps[ctx_i.v].v = ctx_o.v;
      }
    }
  }
}
///< \}
