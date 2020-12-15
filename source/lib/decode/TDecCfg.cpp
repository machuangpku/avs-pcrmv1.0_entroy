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

#include "TDecCfg.h"
#include "common/contributors.h"

///< \in TLibDecoder \{

/**
 * Implementation of TDecCfg
 * decoder configuration class
 */

//////////////////////////////////////////////////////////////////////////
// Public class functions
//////////////////////////////////////////////////////////////////////////

Bool TDecCfg::parseCfg(Int argc, TChar* argv[]) {
  Bool isHelp = false;
  Bool ret = 0;

  // clang-format off
  TComParamParser parser;
  parser.addParameter() ///< data, default value, short option, long option, description
    ("Generic")
    (isHelp,                  false,        "h",   "help",                 "help")
    (ConfigParser,                          "c",   "config",               "config file")
    (m_bitstreamFileName,     string(""),   "b",   "bitstream",            "bitstream output file name")
    (m_reconFileName,         string(""),   "r",   "recon",                "reconstructed PLY output file name")
    (m_numOfFrames,           (UInt)1,      "ftbc","frames_to_be_coded",   "number of frames to be coded. Default: 1")
    (m_writePlyInAsciiFlag,   true,         "awf", "ascii_write_flag",     "recon ply write mode. 1: ascii, 0: binary")
    (m_colorTransformFlag,    false,        "ctf", "color_transform_flag", "apply color transform method. 1: on, 0: off")
    (m_PeakMemoryFlag,        false,        "pmf", "peak_memory_flag",     "output peak memory info. 1: on, 0: off")
    (m_md5FileName,           string(""),   "mdf", "md5_file_name",        "filename for MD5 in encoding process, set default to skip")
  ;
  // clang-format on

  parser.initParameters();                            ///< set the default parameters
  parser.parseParameters(argc, (const TChar**)argv);  ///< parsing
  parser.printInvalidParameters(cout);                ///< print warnings

  if (isHelp || argc == 1)  ///< print help info
  {
    parser.printHelp(cout);
    return false;
  }

  ret = checkParameter();  ///< check the validity of parameters

  parser.printParameters(cout);  ///< print parameter values

  return ret;
}

//////////////////////////////////////////////////////////////////////////
// Private class functions
//////////////////////////////////////////////////////////////////////////

Bool TDecCfg::checkParameter() {
  Bool isFailed = false;

  isFailed |=
    checkCond(m_bitstreamFileName.length() > 0, "Error: bitstream file must be specified.");

  return isFailed;
}

///< \}
