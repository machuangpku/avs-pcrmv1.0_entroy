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

#ifndef _PCEM_PC_EVALUE_H_
#define _PCEM_PC_EVALUE_H_

#include "TComKdTree.h"
#include "common/TComMath.h"
#include "encode/TEncTop.h"
#include <algorithm>
#include <functional>
#include <string>
#include <unordered_map>
#include <utility>

namespace pc_evalue {
class TMetricCfg {
public:
  string m_srcFile;
  string m_disFile;
  bool m_calColor;
  bool m_calReflectance;
  float m_peakValue;
  bool m_duplicateMode;
  bool m_multiNeighbourMode;
  bool m_symmetry;
  bool m_showHausdorff;

  TMetricCfg();

  Bool checkParameter();
  bool parseCfg(Int argc, TChar* argv[]);
};

class TMetricRes {
public:
  float m_geometryMSE;
  float m_geometryHausdorff;
  float m_geometryPSNR;
  float m_geometryHausdorffPSNR;

  float m_colorMSE[3];
  float m_colorHausdorff[3];
  float m_colorPSNR[3];
  float m_colorHausdorffPSNR[3];

  float m_reflectanceMSE;
  float m_reflectanceHausdorff;
  float m_reflectancePSNR;
  float m_reflectanceHausdorffPSNR;

  float m_peakValue;
  TMetricRes();
  TMetricRes operator+(TMetricRes r);
  TMetricRes operator/(unsigned int numOfFrames);
  void print(TMetricCfg para);
};

void RGB2YUV_BT709(const PC_COL& rgb, PC_COL& yuv);
float getPSNR(float MSE, float peakValue, float factor);
bool computeMetric(const TComPointCloud& src, TComPointCloud& dis, TMetricCfg& para,
                   TMetricRes& res);
bool checkPointCloud(const TComPointCloud& in, TComPointCloud& out, TMetricCfg& para);
float computePeakValue(const TComPointCloud& src);
void computeEValue(const TComPointCloud& src, const TComPointCloud& dis, TMetricCfg& para,
                   TMetricRes& res);

}  // namespace pc_evalue

#endif