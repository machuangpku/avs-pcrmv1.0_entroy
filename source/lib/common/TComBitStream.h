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
#include <fstream>

using namespace std;
#define MAX_BS_BUF 1 << 28        ///< 256 * 1024 * 1024 bytes
#define MAX_HEADER_BS_BUF 1 << 8  ///< 256 bytes

///< \in TLibCommon

/**
 * Class TComBitstream
 * bitstream buffer (a pointer to the real buffer)
 */

class TComBitstream {
public:
  /* user space address indicating buffer */
  Void* addr;
  Void* addr2;
  /* physical address indicating buffer, if any */
  Void* pddr;
  /* byte size of buffer memory */
  Int bsize;
  /* byte size of bitstream in buffer */
  Int ssize;
  /* bitstream has an error? */
  Int err;
  /* arbitrary data, if needs */
  Int ndata[4];
  /* arbitrary address, if needs */
  Void* pdata[4];
  /* time-stamps */
  COM_MTIME ts[4];

public:
  TComBitstream();
  ~TComBitstream();

  Void reset();
  Void allocateBuffSize(const TSize buffSize = MAX_BS_BUF);
  Void* getBitStreamBuffer();

private:
  Void init();
};  ///< END CLASS TComBitstream

/**
 * Class COM_BS
 * bitstream buffer (real buffer)
 */

class COM_BS;
typedef Int (*COM_BS_FN_FLUSH)(COM_BS* bs);

class COM_BS {
public:
  /* buffer */
  UInt32 code;
  /* bits left in buffer */
  Int leftbits;
  /*! address of current writing position */
  UInt8* cur;
  /*! address of bitstream buffer end */
  UInt8* end;
  /*! address of bitstream buffer begin */
  UInt8* beg;
  /*! address of temporal buffer for demulation begin */
  UInt8* buftmp;
  /*! size of bitstream buffer in byte */
  Int size;
  /*! address of function for flush */
  COM_BS_FN_FLUSH fn_flush;
  /*! arbitrary data, if needs */
  Int ndata[4];
  /*! arbitrary address, if needs */
  Void* pdata[4];
  // avs2
  UInt8* p_start; /* actual buffer for written bytes */
  UInt8* p;       /* pointer to byte written currently */
  UInt8* p_end;   /* end of the actual buffer */
  int i_left;     /* current bit counter to go */

public:
  Int com_bsw_get_write_byte() {
    return (Int)(cur - beg);  ///< get number of byte written
  }
  Int com_bsw_get_sink_byte()  ///< number of bytes to be sunk
  {
    return ((32 - leftbits + 7) >> 3);
  }
  Bool com_is_byte_align()  ///< check if byte is aligned
  {
    return !(leftbits & 0x7);
  }
  void init() {
    code = 0;
    leftbits = 0;
    cur = end = beg = buftmp = nullptr;
    size = 0;
    fn_flush = 0;
    memset(ndata, 0, 4 * sizeof(Int));
    memset(pdata, 0, 4 * sizeof(Void*));
  }
};  ///< END CLASS COM_BS

///< \}
