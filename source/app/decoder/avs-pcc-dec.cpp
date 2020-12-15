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

// clang-format off
#include "common/CommonDef.h"
#include "common/contributors.h"
#include "decode/TDecTop.h"
#include <iostream>
#include <time.h>
#include "common/TComMemory.h"
// clang-format on

using namespace std;

///< \in TAppDecoder

/**
 * Main Function
 * encoder entry function
 */

Int main(Int argc, TChar* argv[]) {
  /// print information
  cout << "" << AVS_PCC_SW_NAME << ": Decoder " << AVS_PCC_VERSION << " ";
  cout << NVM_ONOS << NVM_COMPILEDBY << NVM_BITS << endl;

  TDecTop decTop;

  /// parse configuration
  if (!decTop.parseCfg(argc, argv))
    return EXIT_FAILURE;

  Double wallTime;
  clock_t wallTimeBegin = clock();

  Int ret = decTop.decode();  ///< start decoding

  wallTime = (Double)(clock() - wallTimeBegin) / CLOCKS_PER_SEC;
  cout << "Total processing time (wall): " << wallTime << " sec." << endl;

  if (decTop.getPeakMemoryFlag()) {
    size_t peakMemorySize = getPeakMemory() / 1024;
    cout << "Peak physical memory usage: " << peakMemorySize << " KB." << endl;
  }
  
  return ret;
}

///< \}
