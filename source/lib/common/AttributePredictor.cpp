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

#include "AttributePredictor.h"
#include <queue>
#include <vector>

//obtain reflectance predictor based Hilbert order
int64_t getReflectancePredictor(int idx, TComPointCloud& outputPointCloud,
                                const SequenceParameterSet& sps,
                                const std::vector<pointMortonCodeWithIndex>& pointCloudMorton,
                                const vector<PC_POS_INT>& positions) {
  int64_t predRef;
  auto& pointIndex = pointCloudMorton[idx].index;

  if (idx == 0) {
    predRef = 0;
  } else if (idx == 1) {
    auto lastPointIndex = pointCloudMorton[idx - 1].index;
    predRef = outputPointCloud.getReflectance(lastPointIndex);
  } else if (idx == 2) {
    auto last1PointIndex = pointCloudMorton[idx - 1].index;
    auto last2PointIndex = pointCloudMorton[idx - 2].index;
    const PC_POS curPoint = outputPointCloud[pointIndex];
    const PC_POS last1Point = outputPointCloud[last1PointIndex];
    const PC_POS last2Point = outputPointCloud[last2PointIndex];
    const auto weight1 = (abs(curPoint[0] - last1Point[0]) + abs(curPoint[1] - last1Point[1]) +
                          16 * abs(curPoint[2] - last1Point[2]));
    const auto weight2 = (abs(curPoint[0] - last2Point[0]) + abs(curPoint[1] - last2Point[1]) +
                          16 * abs(curPoint[2] - last2Point[2]));

    const auto last1Reflectance = outputPointCloud.getReflectance(last1PointIndex);
    const auto last2Reflectance = outputPointCloud.getReflectance(last2PointIndex);

    if (weight1 + weight2 > 0) {
      predRef = (int64_t)round((weight2 * last1Reflectance + weight1 * last2Reflectance) /
                               (weight1 + weight2));
    } else {
      bool w1 = (weight1 == 0);
      bool w2 = (weight2 == 0);
      predRef = (int64_t)round((w1 * last1Reflectance + w2 * last2Reflectance) / (w1 + w2));
    }
  } else {
    auto& curPos = positions[pointIndex];
    int minBegin = std::max(int(idx - 128), 0);

    // Find three nearest neighbors
    int64_t dis;
    std::priority_queue<pair<int64_t, int32_t>> neis;
    for (int i = idx - 1; i >= minBegin; i--) {
      dis = Norm1(curPos, positions[pointCloudMorton[i].index]);
      if (neis.size() < 3) {
        neis.push({dis, pointCloudMorton[i].index});
      } else {
        if (dis < neis.top().first) {
          neis.pop();
          neis.push({dis, pointCloudMorton[i].index});
        }
      }
    }

    //Compute prediction value
    auto w1 = neis.top().first;
    auto& ref1 = outputPointCloud.getReflectance(neis.top().second);
    neis.pop();

    auto w2 = neis.top().first;
    auto& ref2 = outputPointCloud.getReflectance(neis.top().second);
    neis.pop();

    auto w3 = neis.top().first;
    auto& ref3 = outputPointCloud.getReflectance(neis.top().second);

    auto sumW = w1 * w2 + w2 * w3 + w1 * w3;
    if (sumW > 0) {
      predRef = (w2 * w3 * ref1 + w1 * w3 * ref2 + w1 * w2 * ref3) / sumW;
    } else {
      bool wb1 = (w1 == 0);
      bool wb2 = (w2 == 0);
      bool wb3 = (w3 == 0);
      sumW = wb1 + wb2 + wb3;
      predRef = (wb1 * ref1 + wb2 * ref2 + wb3 * ref3) / sumW;
    }
  }

  return predRef;
}

//obtain reflectance predictor
int64_t getReflectancePredictor(int idx, TComPointCloud& outputPointCloud,
                                const SequenceParameterSet& sps,
                                const std::vector<pointMortonCodeWithIndex>& pointCloudMorton,
                                const std::vector<int>& mortonToindex,
                                const std::vector<int>& mortonTomorton2) {
  int64_t predictorRefl;
  auto pointIndex = mortonToindex[idx];
  const Int maxNeghborNum = 3;
  const Int searchrange = 4;
  if (idx == 0) {
    predictorRefl = 0;
  } else if (idx == 1) {
    auto lastPointIndex = mortonToindex[idx - 1];
    predictorRefl = outputPointCloud.getReflectance(lastPointIndex);
  } else if (idx == 2) {
    auto last1PointIndex = mortonToindex[idx - 1];
    auto last2PointIndex = mortonToindex[idx - 2];
    const PC_POS curPoint = outputPointCloud[pointIndex];
    const PC_POS last1Point = outputPointCloud[last1PointIndex];
    const PC_POS last2Point = outputPointCloud[last2PointIndex];
    const auto weight1 = (abs(curPoint[0] - last1Point[0]) + abs(curPoint[1] - last1Point[1]) +
                          16 * abs(curPoint[2] - last1Point[2]));
    const auto weight2 = (abs(curPoint[0] - last2Point[0]) + abs(curPoint[1] - last2Point[1]) +
                          16 * abs(curPoint[2] - last2Point[2]));

    const auto last1Reflectance = outputPointCloud.getReflectance(last1PointIndex);
    const auto last2Reflectance = outputPointCloud.getReflectance(last2PointIndex);

    if (weight1 + weight2 > 0) {
      predictorRefl = (int64_t)round((weight2 * last1Reflectance + weight1 * last2Reflectance) /
                                     (weight1 + weight2));
    } else {
      bool w1 = (weight1 == 0);
      bool w2 = (weight2 == 0);
      predictorRefl = (int64_t)round((w1 * last1Reflectance + w2 * last2Reflectance) / (w1 + w2));
    }
  } else {
    PredictedNearsetNeighbors predicted;
    predicted.init(maxNeghborNum);
    const PC_POS& point = outputPointCloud[pointIndex];
    const int64_t x = (int64_t)point[0];
    const int64_t y = (int64_t)point[1];
    const int64_t z = (int64_t)point[2];
    //sort in morton
    for (Int i = 0; i < searchrange && idx - i > 0; i++) {
      auto pointIndex1 = mortonToindex[idx - i - 1];
      const auto& point1 = outputPointCloud[pointIndex1];
      const int64_t x2 = (int64_t)point1[0];
      const int64_t y2 = (int64_t)point1[1];
      const int64_t z2 = (int64_t)point1[2];
      int64_t dist = abs(x - x2) + abs(y - y2) + abs(z - z2);
      predicted.insertNeighor(pointIndex1, dist);
    }
    //sort in morton2
    int mortonindex2 = mortonTomorton2[idx];
    for (int i = 0; i < searchrange && mortonindex2 - i > 0; i++) {
      int neighmortonIndex = pointCloudMorton[mortonindex2 - i - 1].index;
      if (neighmortonIndex < idx) {
        int pointIndex1 = mortonToindex[neighmortonIndex];
        const auto& point1 = outputPointCloud[pointIndex1];
        const int64_t x2 = (int64_t)point1[0];
        const int64_t y2 = (int64_t)point1[1];
        const int64_t z2 = (int64_t)point1[2];
        int64_t dist = abs(x - x2) + abs(y - y2) + abs(z - z2);
        if (std::find(predicted.begin(), predicted.end(), pointIndex1) != predicted.end())
          continue;
        predicted.insertNeighor(pointIndex1, dist);
      }
    }
    auto neighbors = predicted.getNeigbors();
    const uint32_t last1PointIndex = neighbors[0].predictorIndex;
    const uint32_t last2PointIndex = neighbors[1].predictorIndex;
    const uint32_t last3PointIndex = neighbors[2].predictorIndex;
    const int64_t weight1 = neighbors[0].weight;
    const int64_t weight2 = neighbors[1].weight;
    const int64_t weight3 = neighbors[2].weight;
    const auto last1Reflectance = outputPointCloud.getReflectance(last1PointIndex);
    const auto last2Reflectance = outputPointCloud.getReflectance(last2PointIndex);
    const auto last3Reflectance = outputPointCloud.getReflectance(last3PointIndex);
    if (weight1 * weight2 + weight2 * weight3 + weight1 * weight3 > 0) {
      predictorRefl = (int64_t)round((weight2 * weight3 * last1Reflectance +
                                      weight1 * weight3 * last2Reflectance +
                                      weight1 * weight2 * last3Reflectance) /
                                     (weight1 * weight2 + weight2 * weight3 + weight1 * weight3));
    } else {
      bool w1 = (weight1 == 0);
      bool w2 = (weight2 == 0);
      bool w3 = (weight3 == 0);
      predictorRefl = (int64_t)round(
        (w1 * last1Reflectance + w2 * last2Reflectance + w3 * last3Reflectance) / (w1 + w2 + w3));
    }
  }
  return predictorRefl;
}

//obtain color predictor
PC_COL getColorPredictor(int idx, TComPointCloud& outputPointCloud, const SequenceParameterSet& sps,
                         const AttributeParameterSet& aps,
                         std::vector<pointMortonCodeWithIndex>& pointCloudMorton) {
  PC_COL predictorColor;
  int minBegin = std::max(int(idx - aps.maxNumOfNeighbours), 0);
  if (idx == 0) {
    predictorColor = {0, 0, 0};
  } else {
    //current point Mortoncode
    int64_t cur_pos = pointCloudMorton[idx].mortonCode;
    int predictorNumber;
    int sumWeight = 0;
    //base position Mortoncode
    int64_t base_pos = morton3dAdd(cur_pos, -1ll);
    //neighbor offsets are relative to base_pos
    const uint8_t neighOffset[6] = {3, 5, 6, 1, 2, 4};
    //define neighbor Index & color
    PredictedNearsetNeighbors predicted;
    const Int maxNeighNum = aps.maxNumOfPredictNeighbours;
    predicted.init(maxNeighNum);
    for (int i = 0; i < 6; i++) {
      //compute neighbor address
      int64_t neigh_pos = morton3dAdd(base_pos, neighOffset[i]);
      //find the neighbor's morton big or equal
      auto iter = std::lower_bound(pointCloudMorton.begin() + minBegin,
                                   pointCloudMorton.begin() + idx, neigh_pos);
      if (iter == pointCloudMorton.begin() + idx || iter->mortonCode != neigh_pos)
        continue;
      int32_t index = std::distance(pointCloudMorton.begin(), iter);
      auto pointIndex = pointCloudMorton[index].index;
      if (i < 3) {
        predicted.insertNeighor(pointIndex, 2, false);
        sumWeight += 2;
      } else {
        predicted.insertNeighor(pointIndex, 1, false);
        sumWeight += 1;
      }
      //find three neighbor will break
      if (predicted.getNeighCount() >= neighborCount)
        break;
    }
    predictorNumber = predicted.getNeighCount();
    if (predictorNumber) {
      std::vector<Int64> sum(3, 0);
      auto neighbors = predicted.getNeigbors();
      for (size_t m = 0; m < predicted.getNeighCount(); ++m) {
        PC_COL neighborColor;
        uint64_t weight = neighbors[m].weight;
        neighborColor = outputPointCloud.getColor(neighbors[m].predictorIndex);
        for (size_t k = 0; k < 3; ++k)
          sum[k] += neighborColor[k] * weight;
      }
      for (size_t j = 0; j < 3; ++j)
        predictorColor[j] = sum[j] / sumWeight;
      //if predictorNumber bigger than 1,will used the first neighbor predict U with V
      if (predictorNumber > 1 && sps.geomRemoveDuplicateFlag) {
        PC_COL firstNeighbor = outputPointCloud.getColor(neighbors[0].predictorIndex);
        for (size_t k = 1; k < 3; ++k)
          predictorColor[k] = firstNeighbor[k];
      }
    } else {
      int32_t lastPointIndex = pointCloudMorton[idx - 1].index;
      predictorColor = outputPointCloud.getColor(lastPointIndex);
    }
  }
  return predictorColor;
}
