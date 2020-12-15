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

#include "TComBitStream.h"
#include "contributors.h"

///< \in TLibCommon

/**
 * Implementation of TComBitstream
 * bitstream buffer operations
 */

//////////////////////////////////////////////////////////////////////////
// Public class functions
//////////////////////////////////////////////////////////////////////////

TComBitstream::TComBitstream() {
  init();
}

TComBitstream::~TComBitstream() {
  reset();
}

Void TComBitstream::reset() {
  if (addr)
    free((UChar*)addr);
  if (addr2)
    free((UChar*)addr2);
  init();
}

Void TComBitstream::allocateBuffSize(const TSize buffSize) {
  ///< allocate bitstream buffer
  if (addr == nullptr) {
    addr = (UChar*)calloc(1, buffSize);
    assert(checkCond(addr != NULL, "Error: cannot allocate bitstream buffer!"));
  }
  if (addr2 == nullptr) {
    addr2 = (UChar*)calloc(1, buffSize);
    assert(checkCond(addr2 != NULL, "Error: cannot allocate bitstream buffer!"));
  }
  ssize = (Int)buffSize;
  bsize = (Int)buffSize;
  err = 0;
}

Void* TComBitstream::getBitStreamBuffer() {
  return addr;
}

//////////////////////////////////////////////////////////////////////////
// Private class functions
//////////////////////////////////////////////////////////////////////////

Void TComBitstream::init() {
  addr = addr2 = pddr = nullptr;
  bsize = ssize = err = 0;
  memset(ndata, 0, 4 * sizeof(Int));
  memset(pdata, 0, 4 * sizeof(Void*));
  memset(ts, 0, sizeof(COM_MTIME));
}

///< \}
