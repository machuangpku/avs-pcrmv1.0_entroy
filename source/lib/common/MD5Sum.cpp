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

#include "MD5Sum.h"
#include <cassert>
#include <iomanip>
#include <iostream>
#include <sstream>

using std::cout;
using std::endl;
using std::hex;
using std::setfill;
using std::setw;
using std::stringstream;

///< \in TLibCommon

/**
 * Class MD5Sum
 * MD5 calculator class
 */

//////////////////////////////////////////////////////////////////////////
// Public class functions
//////////////////////////////////////////////////////////////////////////

MD5Sum::MD5Sum(UInt64 batchSize)
  : m_pc(nullptr)
  , m_batchSize(batchSize)
  , m_totalByteSize(0)
  , m_dataByteSize(0)
  , m_batchByteSize(0)
  , m_md5Str("") {
  clear();
}

Void MD5Sum::calculateMD5(const TComPointCloud* pc) {
  m_pc = pc;
  ///< calculate byte size per point
  int pointByteSize = sizeof(PC_POS);
  if (m_pc->hasColors()) {
    pointByteSize += sizeof(PC_COL);
  }
  if (m_pc->hasReflectances()) {
    pointByteSize += sizeof(PC_REFL);
  }

  m_totalByteSize = pointByteSize * m_pc->getNumPoint();

  ///< in every iteration of MD5 update, num of m_batchSize points are sent to m_md5Payload
  UInt64 numBatch = m_pc->getNumPoint() / m_batchSize;
  UInt64 numLeftPoint = m_pc->getNumPoint() % m_batchSize;
  m_batchByteSize = m_batchSize * pointByteSize;
  m_md5Payload = (UInt8*)malloc(m_batchByteSize);

  for (TSize bid = 0; bid < numBatch; bid++) {
    UInt8* currentPointer = m_md5Payload;
    for (TSize i = bid * m_batchSize; i < (bid + 1) * m_batchSize; i++) {
      copyPointToPayload(i, currentPointer);
    }
    m_dataByteSize = m_batchByteSize;
    md5Process();
  }

  ///< if the size of the last batch is less than m_batchSize, process with padding
  if (numLeftPoint >= 0) {
    UInt64 pid = m_batchSize * numBatch;
    UInt8* currentPointer = m_md5Payload;
    for (TSize i = pid; i < pid + numLeftPoint; i++) {
      copyPointToPayload(i, currentPointer);
    }
    m_dataByteSize = numLeftPoint * pointByteSize;
    md5Process();
  }

  saveResult();
  free(m_md5Payload);
  clear();
}

Bool MD5Sum::openOStream(const string& fileName) {
  bool checkFlag = true;

  if (fileName.empty()) {
    checkFlag = false;
  } else {
    m_md5File.open(fileName, ios::out | ios::trunc);

    if (!m_md5File) {
      cerr << "Error: failed to open  file " << fileName << "skip MD5 writing!" << endl;
      checkFlag = false;  //无法读取文件，跳过MD5存储过程
    }
  }
  return checkFlag;
}

Void MD5Sum::writeToFile() {
  m_md5File << m_md5Str << endl;
}

Bool MD5Sum::openIStream(const string& inputname) {
  bool checkFlag = true;

  if (inputname.empty()) {
    checkFlag = false;
  } else {
    m_md5File.open(inputname, std::fstream::in);

    if (!m_md5File) {
      cerr << "Error: failed to open  file " << inputname << "skip MD5 checking!" << endl;
      checkFlag = false;  //无法读取文件，跳过 MD5 校验过程
    }
  }
  return checkFlag;
}

Void MD5Sum::closeFile() {
  m_md5File.close();
}

string MD5Sum::getMD5Str() const {
  return m_md5Str;
}

string MD5Sum::getMD5InFile() {
  string input;
  m_md5File >> input;
  return input;
}

//////////////////////////////////////////////////////////////////////////
// Private class functions
//////////////////////////////////////////////////////////////////////////

Void MD5Sum::clear() {
  m_result.h0 = 0x67452301;
  m_result.h1 = 0xEFCDAB89;
  m_result.h2 = 0x98BADCFE;
  m_result.h3 = 0x10325476;

  m_totalByteSize = 0;  ///<数据总长度，字节数，下同
  m_batchByteSize = 0;  ///<缓冲区长度
  m_dataByteSize = 0;
}

Void MD5Sum::compareResult(string counterpart) {
  return Void();
}

Void MD5Sum::copyPointToPayload(TSize i, UInt8*& currentPointer) {
  ///< copy the data of ith point to MD5 buffer
  memcpy(currentPointer, &((*m_pc)[i]), sizeof(PC_POS));
  currentPointer += sizeof(PC_POS);
  if (m_pc->hasColors()) {
    PC_COL temp = m_pc->getColor(i);
    memcpy(currentPointer, &(temp), sizeof(PC_COL));
    currentPointer += sizeof(PC_COL);
  }
  if (m_pc->hasReflectances()) {
    PC_REFL temp = m_pc->getReflectance(i);
    memcpy(currentPointer, &(temp), sizeof(PC_REFL));
    currentPointer += sizeof(PC_REFL);
  }
}

Void MD5Sum::md5Process() {
  if (m_dataByteSize != m_batchByteSize)
    padding();
  getMD5();
}

Void MD5Sum::padding()  ///<填充数据,使用填充数据的尾部为作为参数
{
  const UInt8 first_pad = 128;  ///<第一字节高位为 1
  const UInt8 follow_pad = 0;   ///<后续填充位全为 0

  int temp = 56 - m_dataByteSize % 64;
  UInt32 padding = (temp > 0) ? temp : 64 + temp;  ///<字节数算 padding 的长度

  UInt8* tail = m_md5Payload + m_dataByteSize;  ///<待填充数据位置
  for (UInt32 i = 0; i < padding; i++) {
    if (i == 0)
      tail[i] = first_pad;  ///<第一次填充，高位为 1
    else
      tail[i] = follow_pad;  ///<后续直接使用 0 填充
  }

  tail = tail + padding;  ///<填充数据后尾部

  uint64ToByte(m_totalByteSize * 8, tail);  ///<总数据长度放尾部,注意是 bit 长度，而不是字节长度

  m_dataByteSize = m_dataByteSize + padding + 8;  ///<padding + 原长度 后的数据

  assert((tail + 8 - m_md5Payload) % 64 == 0);  ///<填充后为 64 的倍数
  assert(m_dataByteSize % 64 == 0);
}

Void MD5Sum::calMD5(UInt32 (&block)[N512InN32]) {
  static const UInt32 k[] = {
    0xd76aa478, 0xe8c7b756, 0x242070db, 0xc1bdceee, 0xf57c0faf, 0x4787c62a, 0xa8304613, 0xfd469501,
    0x698098d8, 0x8b44f7af, 0xffff5bb1, 0x895cd7be, 0x6b901122, 0xfd987193, 0xa679438e, 0x49b40821,
    0xf61e2562, 0xc040b340, 0x265e5a51, 0xe9b6c7aa, 0xd62f105d, 0x02441453, 0xd8a1e681, 0xe7d3fbc8,
    0x21e1cde6, 0xc33707d6, 0xf4d50d87, 0x455a14ed, 0xa9e3e905, 0xfcefa3f8, 0x676f02d9, 0x8d2a4c8a,
    0xfffa3942, 0x8771f681, 0x6d9d6122, 0xfde5380c, 0xa4beea44, 0x4bdecfa9, 0xf6bb4b60, 0xbebfbc70,
    0x289b7ec6, 0xeaa127fa, 0xd4ef3085, 0x04881d05, 0xd9d4d039, 0xe6db99e5, 0x1fa27cf8, 0xc4ac5665,
    0xf4292244, 0x432aff97, 0xab9423a7, 0xfc93a039, 0x655b59c3, 0x8f0ccc92, 0xffeff47d, 0x85845dd1,
    0x6fa87e4f, 0xfe2ce6e0, 0xa3014314, 0x4e0811a1, 0xf7537e82, 0xbd3af235, 0x2ad7d2bb, 0xeb86d391};

  static const UInt32 offset[] = {7, 12, 17, 22, 7, 12, 17, 22, 7, 12, 17, 22, 7, 12, 17, 22,
                                  5, 9,  14, 20, 5, 9,  14, 20, 5, 9,  14, 20, 5, 9,  14, 20,
                                  4, 11, 16, 23, 4, 11, 16, 23, 4, 11, 16, 23, 4, 11, 16, 23,
                                  6, 10, 15, 21, 6, 10, 15, 21, 6, 10, 15, 21, 6, 10, 15, 21};

  UInt32 a = m_result.h0;
  UInt32 b = m_result.h1;
  UInt32 c = m_result.h2;
  UInt32 d = m_result.h3;

  UInt32 f = 0;
  UInt32 g = 0;

  for (auto i = 0; i < 64; i++) {
    if (i <= 15) {
      f = F(b, c, d);
      g = i;
    } else if (i <= 31) {
      f = F(d, b, c);
      g = (5 * i + 1) % 16;
    } else if (i <= 47) {
      f = H(b, c, d);
      g = (3 * i + 5) % 16;
    } else {
      f = I(b, c, d);
      g = (7 * i) % 16;
    }

    UInt32 temp = d;
    d = c;
    c = b;
    b = shift(a + f + k[i] + block[g], offset[i]) + b;
    a = temp;
  }

  m_result.h0 += a;
  m_result.h1 += b;
  m_result.h2 += c;
  m_result.h3 += d;
}

Void MD5Sum::getMD5() {
  UInt32 block[N512InN32];  ///<以 512 字节为单位处理, 16 个 uint32
  UInt64 len = m_dataByteSize;
  assert(m_dataByteSize % 64 == 0);  ///<保证 m_datalen 为 512 bit 个倍数

  for (TSize i = 0; i < len / (N512InN32 * 4); i++) {
    UInt8* start = m_md5Payload + i * (N512InN32 * 4);  ///<以 64 为单位

    for (TSize j = 0; j < N512InN32; j++) {
      UInt8* temp = start + j * 4;    ///<每次处理 4 字节
      block[j] = byteToUint32(temp);  ///<4 字节转化为 UInt32
    }
    calMD5(block);  ///<计算 512 bit 即 64 字节的结果
  }
}

Void MD5Sum::saveResult() {
  UInt8 block[16];
  uint32ToByte(m_result.h0, block + 0);
  uint32ToByte(m_result.h1, block + 4);
  uint32ToByte(m_result.h2, block + 8);
  uint32ToByte(m_result.h3, block + 12);

  stringstream result;
  result << setfill('0');
  for (auto i = 0; i < 16; i++)
    result << setw(2) << hex << (unsigned int)block[i];  ///<这里转换为 int 是重载不支持 UInt8 类型
  result << setfill(' ');

  m_md5Str = string(result.str());
  cout << "MD5sum: " << m_md5Str << endl;
}

UInt32 byteToUint32(UInt8* ptr)  ///<8 bit转为 32 bit数据
{
  UInt32 result = 0;

  for (Int i = 0; i < 4; i++) {
    UInt32 temp = ptr[i];
    result += (temp << (i * 8));
  }

  return result;
}

Void uint32ToByte(UInt32 input, UInt8* ptr)  ///<32 bit 转为 8 bit数据,结果存放在 ptr 的连续四字节中
{
  for (Int i = 0; i < 4; i++) {
    ptr[i] = input & 0xff;  ///<取低 8 位
    input = input >> 8;
  }
}

Void uint64ToByte(UInt64 input, UInt8* ptr)  ///<64 bit 转为 8 bit数据,结果存放在 ptr 的连续八字节中
{
  for (Int i = 0; i < 8; i++) {
    ptr[i] = input & 0xff;  ///<取低 8 位
    input = input >> 8;
  }
}

///< \}