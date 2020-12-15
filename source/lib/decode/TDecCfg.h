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

#include "common/HighLevelSyntax.h"
#include "common/TComParamParser.h"
#include "common/TypeDef.h"
#include "common/contributors.h"
#include <sstream>

using namespace std;

///< \in TLibDecoder

/**
 * Class TDecCfg
 * decoder configuration class
 */

class TDecCfg {
protected:
  string m_bitstreamFileName;  ///< input bitstream file name
  string m_reconFileName;      ///< output reconstruction file name
  string m_md5FileName;        ///< filename for MD5 in encoding process, set default to skip
  UInt m_startFrame;           ///< start frame number
  UInt m_numOfFrames;          ///< number of frames to be encoded
  Bool m_writePlyInAsciiFlag;  ///< ascii/binary mode of output ply file
  Bool m_colorTransformFlag;   ///< color transform e.g. RGB-YUV, this should be per-attribute
  Bool m_PeakMemoryFlag;       ///< output peak memory
  HighLevelSyntax m_hls;       ///< high-level syntax parameters

private:
  Bool checkParameter();  ///< check validity of configuration values

public:
  TDecCfg() {}
  ~TDecCfg() {}

  Bool parseCfg(Int argc,
                TChar* argv[]);  ///< parse configurations from command line or config files

  Bool getPeakMemoryFlag() {
    return m_PeakMemoryFlag;
  }

};  ///< END CLASS TDecCfg

///< \}
