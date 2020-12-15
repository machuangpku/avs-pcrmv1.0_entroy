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

#include "TComBufferChunk.h"
#include "CommonDef.h"
#include "contributors.h"

///< \in TLibEncoder

/**
 * Implementation of TComBufferChunk
 * bitstream buffer chunk
 */

//////////////////////////////////////////////////////////////////////////
// Public class functions
//////////////////////////////////////////////////////////////////////////

Void TComBufferChunk::writeToBitstream(ofstream* outBitstream, UInt64 length) {
  ///< write start code
  outBitstream->put(0);
  outBitstream->put(0);
  outBitstream->put(1);

  ///< write buffer chunk type
  outBitstream->put(TChar(m_bufferType));

  ///< write buffer chunk data
  outBitstream->write((TChar*)getBitStreamBuffer(), length);
}

Int TComBufferChunk::readFromBitstream(ifstream& inBitstream) {
  ///< read start code 00 00 01
  if (!checkCond(inBitstream.get() == 0x00, "Error: invalid start code!"))
    return EXIT_FAILURE;
  if (!checkCond(inBitstream.get() == 0x00, "Error: invalid start code!"))
    return EXIT_FAILURE;
  if (!checkCond(inBitstream.get() == 0x01, "Error: invalid start code!"))
    return EXIT_FAILURE;

  ///< read buffer chunk type
  m_bufferType = (BufferChunkType)inBitstream.get();

  if (m_bufferType == BCT_SPS || m_bufferType == BCT_GPS || m_bufferType == BCT_APS ||
      m_bufferType == BCT_GBH || m_bufferType == BCT_ABH)
    allocateBuffSize(MAX_HEADER_BS_BUF);
  else
    allocateBuffSize(MAX_BS_BUF);

  TSize length = 0;
  if (readBufferChunk(inBitstream, length) == EXIT_FAILURE)  ///< read buffer chunk data
    return EXIT_FAILURE;
  length = initParsingConvertPayloadToRBSP(length, (UChar*)addr, (UChar*)addr2);  ///< demulate
  allocateBuffSize(length);
  return EXIT_SUCCESS;
}

TComBufferChunk::TComBufferChunk(BufferChunkType bufferType) {
  m_bufferType = bufferType;
}

Void TComBufferChunk::setBufferType(BufferChunkType bufferType) {
  m_bufferType = bufferType;
}

BufferChunkType TComBufferChunk::getBufferType() {
  return m_bufferType;
}

//////////////////////////////////////////////////////////////////////////
// Private class functions
//////////////////////////////////////////////////////////////////////////

Int TComBufferChunk::readBufferChunk(ifstream& inBitstream, TSize& bufferChunkSize) {
  UChar* pucBuffer = (UChar*)getBitStreamBuffer();
  UChar ucByte = 0;
  char bEndOfStream = 0;

  Int iNextStartCodeBytes = 0;
  UInt iBytesRead = 0;
  UInt uiZeros = 0;
  UChar pucBuffer_Temp[16];
  Int iBytesRead_Temp = 0;

  while (true) {
    inBitstream.read((TChar*)&ucByte, 1);

    if (inBitstream.eof()) {
      iNextStartCodeBytes = 0;
      bEndOfStream = 1;
      break;
    }
    pucBuffer[iBytesRead++] = ucByte;
    if (ucByte > 1)  ///< ucByte != 0 && UChar != 1
      uiZeros = 0;
    else if (ucByte == 0)  ///< 00
      uiZeros++;
    else if (uiZeros > 1)  ///< 00 00
    {
      iBytesRead_Temp = 0;
      pucBuffer_Temp[iBytesRead_Temp] = ucByte;

      iBytesRead_Temp++;
      inBitstream.read((TChar*)&ucByte, 1);

      pucBuffer_Temp[iBytesRead_Temp] = ucByte;
      pucBuffer[iBytesRead++] = ucByte;
      iBytesRead_Temp++;

      if (pucBuffer_Temp[0] == 0x01 &&
          (pucBuffer_Temp[1] > BCT_MIN && pucBuffer_Temp[1] < BCT_MAX))  ///< 00 00 01 CODE
      {
        iNextStartCodeBytes = 2 + 1 + 1;  ///< encounter the next start code
        uiZeros = 0;
        break;
      } else {
        uiZeros = 0;
        iNextStartCodeBytes = 0;
      }
    } else {
      uiZeros = 0;
    }
  }
  bufferChunkSize = iBytesRead - iNextStartCodeBytes;

  if (bEndOfStream)
    return EXIT_SUCCESS;

  inBitstream.seekg(-1 * iNextStartCodeBytes,
                    std::ios_base::cur);  ///< go back four bytes of start code
  return EXIT_SUCCESS;
}

TSize TComBufferChunk::initParsingConvertPayloadToRBSP(const TSize uiBytesRead, UChar* pBuffer,
                                                       UChar* pBuffer2) {
  UInt uiZeroCount = 0;
  UInt uiBytesReadOffset = 0;
  UInt uiBitsReadOffset = 0;
  const UChar* pucRead = pBuffer;
  UChar* pucWrite = pBuffer2;
  UInt uiWriteOffset = uiBytesReadOffset;
  UChar ucCurByte = pucRead[uiBytesReadOffset];

  for (uiBytesReadOffset = 0; uiBytesReadOffset < uiBytesRead; uiBytesReadOffset++) {
    ucCurByte = pucRead[uiBytesReadOffset];
    if (2 <= uiZeroCount && 0x02 == pucRead[uiBytesReadOffset]) {
      pucWrite[uiWriteOffset] = ((pucRead[uiBytesReadOffset] >> 2) << (uiBitsReadOffset + 2));
      uiBitsReadOffset += 2;
      uiZeroCount = 0;
      if (uiBitsReadOffset >= 8) {
        uiBitsReadOffset = 0;
        continue;
      }
      if (uiBytesReadOffset >= uiBytesRead) {
        break;
      }
    } else if (2 <= uiZeroCount && 0x01 == pucRead[uiBytesReadOffset]) {
      uiBitsReadOffset = 0;
      pucWrite[uiWriteOffset] = pucRead[uiBytesReadOffset];
    } else {
      pucWrite[uiWriteOffset] = (pucRead[uiBytesReadOffset] << uiBitsReadOffset);
    }

    if (uiBytesReadOffset + 1 < uiBytesRead) {
      pucWrite[uiWriteOffset] |= (pucRead[uiBytesReadOffset + 1] >> (8 - uiBitsReadOffset));
    }
    uiWriteOffset++;

    if (0x00 == ucCurByte) {
      uiZeroCount++;
    } else {
      uiZeroCount = 0;
    }
  }

  // th just clear the remaining bits in the buffer
  for (UInt ui = uiWriteOffset; ui < uiBytesRead; ui++) {
    pucWrite[ui] = 0;
  }
  memcpy(pBuffer, pBuffer2, uiWriteOffset);
  return uiBytesRead;
}

///< \}
