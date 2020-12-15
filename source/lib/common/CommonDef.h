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

#include "contributors.h"
#include <assert.h>
#include <cstring>
#include <iostream>
#include <string>
#ifdef __MINGW32__
#include <intrin.h>
#endif
#include "TypeDef.h"

using namespace std;

///< \in TLibCommon \{

//////////////////////////////////////////////////////////////////////////
// Software Information
//////////////////////////////////////////////////////////////////////////
#define AVS_PCC_SW_NAME "AVS-PCC-PCRM"
#define AVS_PCC_VERSION "v1.0"  ///< software version

//////////////////////////////////////////////////////////////////////////
// Platform Information
//////////////////////////////////////////////////////////////////////////

#ifdef __GNUC__
#define NVM_COMPILEDBY \
  "[GCC " << __GNUC__ << "." << __GNUC_MINOR__ << "." << __GNUC_PATCHLEVEL__ << "]"
#ifdef __IA64__
#define NVM_ONARCH "[on 64-bit] "
#else
#define NVM_ONARCH "[on 32-bit]  "
#endif
#endif

#ifdef __INTEL_COMPILER
#define NVM_COMPILEDBY "[ICC " << __INTEL_COMPILER << "]"
#elif _MSC_VER
#define NVM_COMPILEDBY "[VS " << _MSC_VER << "]"
#endif

#ifndef NVM_COMPILEDBY
#define NVM_COMPILEDBY "[Unk-CXX]"
#endif

#ifdef _WIN32
#define NVM_ONOS "[Windows]"
#elif __linux
#define NVM_ONOS "[Linux]"
#elif __CYGWIN__
#define NVM_ONOS "[Cygwin]"
#elif __APPLE__
#define NVM_ONOS "[Mac OS X]"
#else
#define NVM_ONOS "[Unk-OS]"
#endif

#if _WIN32 || _WIN64
#if _WIN64
#define NVM_BITS "[64 bit]"
#else
#define NVM_BITS "[32 bit]"
#endif
#endif

#if __GNUC__
#if __x86_64__ || __ppc64__
#define NVM_BITS "[64 bit]"
#else
#define NVM_BITS "[32 bit]"
#endif
#endif

//////////////////////////////////////////////////////////////////////////
// Binary Arithmetic Coder
//////////////////////////////////////////////////////////////////////////

#define LG_PMPS_SHIFTNO 2

//////////////////////////////////////////////////////////////////////////
// adaptive attribute prediction
//////////////////////////////////////////////////////////////////////////
#define ATTRIBUTE_CACHE_SIZE 32
#define ATTRIBUTE_CACHE_SIZE_BIT 5
#define ATTRIBUTE_PRED_LOSSLESS_THRESH 50  //100
#define ATTRIBUTE_PRED_LOSSY_THRESH 25     //50

//////////////////////////////////////////////////////////////////////////
// assert function
//////////////////////////////////////////////////////////////////////////

#define com_assert(x) \
  {                   \
    if (!(x)) {       \
      assert(x);      \
    }                 \
  }
#define com_assert_r(x) \
  {                     \
    if (!(x)) {         \
      assert(x);        \
      return;           \
    }                   \
  }
#define com_assert_rv(x, r) \
  {                         \
    if (!(x)) {             \
      assert(x);            \
      return (r);           \
    }                       \
  }
#define com_assert_g(x, g) \
  {                        \
    if (!(x)) {            \
      assert(x);           \
      goto g;              \
    }                      \
  }
#define com_assert_gv(x, r, v, g) \
  {                               \
    if (!(x)) {                   \
      assert(x);                  \
      (r) = (v);                  \
      goto g;                     \
    }                             \
  }

static inline Bool checkCond(Bool b, string msg) {
  if (!b)
    cerr << msg << endl;
  return b;
}

//////////////////////////////////////////////////////////////////////////
// math utility function
//////////////////////////////////////////////////////////////////////////

#define AVS_MIN(a, b) ((a) < (b) ? (a) : (b))
#define AVS_MAX(a, b) ((a) > (b) ? (a) : (b))

static inline Int floorLog2(UInt x) {
  if (x == 0)  ///< log2(0) is illegal
    return -1;
  Int result = 0;
  while (x > 0) {
    ++result;
    x >>= 1;
  }
  return result;
}

static inline Int ceilLog2(UInt x) {
  if (x == 0)  ///< log2(0) is illegal
    return -1;
  Int result = 0;
  x--;
  while (x > 0) {
    ++result;
    x >>= 1;
  }
  return result;
}

///< \}
