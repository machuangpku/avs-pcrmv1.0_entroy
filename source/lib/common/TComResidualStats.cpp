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

#include "TComResidualStats.h"
#include "HighLevelSyntax.h"

int32_t TComResidualStats::calculateAverage() {
  int64_t residualSum = m_residual_sum - m_residual_eq1 - (m_residual_eq2 << 1);
  int32_t numPoints = m_residual_count - m_residual_eq0 - m_residual_eq1 - m_residual_eq2;
  m_residual_avg = (int32_t)(residualSum / numPoints);
  return m_residual_avg;
}

void TComResidualStats::collectStats(uint32_t delta) {
  if (m_residual_count < m_residual_count_threshold) {
    m_residual_sum += delta;
    if (delta == 0) {
      m_residual_eq0++;
    }
    if (delta == 1) {
      m_residual_eq1++;
    }
    if (delta == 2) {
      m_residual_eq2++;
    }
    m_residual_count++;
  }
}

void adaptivePredDecision(const SequenceParameterSet& sps, bool& forceAttrBasedPred,
                          TComResidualStats& residualStats) {
  static bool decisionMade = false;
  if (decisionMade) {
    return;
  }

  if (!sps.attrAdaptPred || forceAttrBasedPred || !residualStats.collectionDone()) {
    return;
  }

  int32_t avgResidual = residualStats.calculateAverage();

  if ((sps.attrQuantParam == 0 && avgResidual > ATTRIBUTE_PRED_LOSSLESS_THRESH) ||
      (sps.attrQuantParam > 0 && avgResidual > ATTRIBUTE_PRED_LOSSY_THRESH)) {
    forceAttrBasedPred = true;
    int32_t threshold =
      (sps.attrQuantParam == 0) ? ATTRIBUTE_PRED_LOSSLESS_THRESH : ATTRIBUTE_PRED_LOSSY_THRESH;
    std::cout << "Average Residual: " << avgResidual << " > " << threshold
              << "  ==> Switch to attribute based predictor " << std::endl;
  }
  decisionMade = true;
}
