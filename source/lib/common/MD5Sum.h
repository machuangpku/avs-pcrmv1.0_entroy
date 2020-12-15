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
#include "TComPointCloud.h"
#include <fstream>
#include <string>

using std::string;

///< \in TLibCommon

/**
 * Class MD5Sum
 * MD5 calculator class
 */

#define N512InN32 16
#define F(x, y, z) (((x) & (y)) | ((~x) & (z)))
#define H(x, y, z) ((x) ^ (y) ^ (z))
#define I(x, y, z) ((y) ^ ((x) | (~z)))
#define shift(x, n) (((x) << (n)) | ((x) >> (32 - (n))))

class MD5Sum {
public:
  MD5Sum(UInt64 batchSize = 1024);
  ~MD5Sum() = default;

  Void calculateMD5(const TComPointCloud* pc);
  Bool openIStream(const string& inputname);
  Bool openOStream(const string& fileName);
  string getMD5Str() const;
  Void writeToFile();
  string getMD5InFile();
  Void closeFile();

private:
  typedef struct output {
    UInt32 h0;
    UInt32 h1;
    UInt32 h2;
    UInt32 h3;
  } MD5_reg;

  const TComPointCloud* m_pc;
  UInt8* m_md5Payload;  ///<每次调用MD5计算时的缓存
  UInt64 m_totalByteSize;
  UInt64 m_batchSize;  ///<每次调用部分MD5计算时处理的点数
  UInt64 m_batchByteSize;
  UInt64 m_dataByteSize;  ///<每批实际处理的数据大小

  MD5_reg m_result;
  string m_md5Str;
  fstream m_md5File;

  Void copyPointToPayload(TSize i, UInt8*& currentPoint);

  Void padding();  ///<填充数据
  Void calMD5(UInt32 (&block)[N512InN32]);
  Void getMD5();  ///<存储数据
  Void saveResult();
  Void md5Process();
  Void clear();

  Void compareResult(string counterpart);
};

uint32_t byteToUint32(UInt8* ptr);  ///<8 bit转为 32 bit数据
Void uint32ToByte(UInt32 input,
                  UInt8* ptr);  ///<32 bit 转为 8 bit数据,结果存放在 ptr 的连续四字节中
Void uint64ToByte(UInt64 input,
                  UInt8* ptr);  ///<64 bit 转为 8 bit数据,结果存放在 ptr 的连续八字节中

///< \}