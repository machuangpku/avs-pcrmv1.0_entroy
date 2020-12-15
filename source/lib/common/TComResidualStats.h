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
#ifndef TComResidualStats_H
#define TComResidualStats_H

#include "CommonDef.h"
#include "contributors.h"

struct SequenceParameterSet;
/**
 * Class TComResidualStats
 * collect attribute prediction residual statistics 
 */
class TComResidualStats {
private:
  uint64_t m_residual_sum{0};
  int32_t m_residual_count{0};
  int32_t m_residual_eq0{0};
  int32_t m_residual_eq1{0};
  int32_t m_residual_eq2{0};
  int32_t m_residual_avg{0};
  int32_t m_residual_count_threshold{512};

public:
  TComResidualStats() = default;
  TComResidualStats(int32_t _count_threshold) {
    m_residual_count_threshold = _count_threshold;
  }

  ~TComResidualStats() = default;

  int32_t calculateAverage();
  void collectStats(uint32_t delta);

  Bool collectionDone() {
    return (m_residual_count == m_residual_count_threshold);
  }
};  ///< END CLASS TComResidualStats

void adaptivePredDecision(const SequenceParameterSet& sps, bool& forceAttrBasedPred,
                          TComResidualStats& residualStats);
#endif  // TComResidualStats_H