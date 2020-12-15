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
#include "HighLevelSyntax.h"
#include "TComVector.h"
#include "contributors.h"
#include <vector>

///< \in TLibCommon \{

/**
 * Struct TComOctreePartitionParams
 * common octree partition related parameters
 */
struct TComOctreePartitionParams {
  V3<UInt> nodeSizeLog2;
  V3<UInt> childSizeLog2;
  UInt occupancySkip;
  Bool bitSkip[8];
  UInt occupancySkipParent;
  UInt maxNumImQtbtBeforeOT;
  UInt minSizeImQtbt;
  UInt numNodesInCurrentDepth;
  UInt numNodesInNextDepth;
  UInt depth;
  UInt maxDepth;
};  ///< END STRUCT TComOctreePartitionParams

/**
 * Class TComOctreeNode
 * node in geometry partition
 */

class TComOctreeNode {
public:
  V3<UInt> pos;           ///< top-left position of the node
  vector<UInt> pointIdx;  ///< point indices to the point cloud in the node

};  ///< END CLASS TComOctreeNode

//////////////////////////////////////////////////////////////////////////
// Inline functions
//////////////////////////////////////////////////////////////////////////

static inline Void updateImQtBtParams(UInt& maxNumImQtbtBeforeOT, UInt& minSizeImQtbt,
                                      const V3<UInt>& nodeSizeLog2) {
  UInt minNodeSizeLog2 = nodeSizeLog2.min();
  UInt maxNodeSizeLog2 = nodeSizeLog2.max();
  ///< maxNumImQtbtBeforeOT is bounded by difference between max and min node size
  if (maxNumImQtbtBeforeOT > (maxNodeSizeLog2 - minNodeSizeLog2))
    maxNumImQtbtBeforeOT = maxNodeSizeLog2 - minNodeSizeLog2;
  ///< minSizeImQtbt is bounded by min node size
  if (minSizeImQtbt > minNodeSizeLog2)
    minSizeImQtbt = minNodeSizeLog2;
  ///< if all dimensions have same size, min depth of implicit qtbt should be 0
  if (maxNodeSizeLog2 == minNodeSizeLog2)
    minSizeImQtbt = 0;
}

static inline Void imQtBtDecision(V3<UInt>& nodeSizeLog2, const UInt maxNumImQtbtBeforeOT,
                                  const UInt minSizeImQtbt) {
  if (nodeSizeLog2.max() == 0)
    return;

  int minNodeSizeLog2 = nodeSizeLog2.min();

  if (maxNumImQtbtBeforeOT || minNodeSizeLog2 == minSizeImQtbt) {
    // implicit qt bt
    int maxNodeSizeLog2 = nodeSizeLog2.max();
    for (int k = 0; k < 3; k++) {
      if (nodeSizeLog2[k] == maxNodeSizeLog2)
        nodeSizeLog2[k]--;
    }
  } else  // octree partition
    nodeSizeLog2 = nodeSizeLog2 - 1;
}
static inline Void xir_skipOccupancy(UInt occupancySkip, Bool skip[8]) {
  for (Int i = 0; i < 8; i++) {
    if (occupancySkip != 0) {
      if ((occupancySkip & 1) && (i & 1))  ///< skip when z = 1
      {
        skip[i] = true;
        continue;
      }
      if ((occupancySkip & 2) && (i & 2))  ///< skip when y = 1
      {
        skip[i] = true;
        continue;
      }
      if ((occupancySkip & 4) && (i & 4))  ///< skip when x = 1
      {
        skip[i] = true;
        continue;
      }
      skip[i] = false;
    } else {
      skip[i] = false;
    }
  }
};

static inline void prepareImQtBtIndicator(UInt& indicator, const V3<UInt>& nodeSizeLog2,
                                          const V3<UInt>& childSizeLog2) {
  indicator = 0;
  for (Int k = 0; k < 3; k++) {
    indicator <<= 1;
    indicator |= nodeSizeLog2[k] == childSizeLog2[k];
  }
}

static inline Void initOctreePartitionParams(const HighLevelSyntax& hls,
                                             TComOctreePartitionParams& partitionParams) {
  ///< node size (log2) for xyz dimensions
  V3<UInt> nodeSizeLog2 = hls.gbh.nodeSizeLog2;

  partitionParams.maxDepth = nodeSizeLog2.max();
  partitionParams.depth = 0;

  UInt maxNumImQtbtBeforeOT = hls.gps.im_qtbt_num_before_ot;
  UInt minSizeImQtbt = hls.gps.im_qtbt_min_size;
  updateImQtBtParams(maxNumImQtbtBeforeOT, minSizeImQtbt, nodeSizeLog2);

  V3<UInt> childSizeLog2 = nodeSizeLog2;
  ///< implicit qtbt for child nodes
  imQtBtDecision(childSizeLog2, maxNumImQtbtBeforeOT, minSizeImQtbt);

  ///< prepare parameters for partition and occupancy coding
  UInt occupancySkip = 0;
  prepareImQtBtIndicator(occupancySkip, nodeSizeLog2, childSizeLog2);

  partitionParams.nodeSizeLog2 = nodeSizeLog2;
  partitionParams.childSizeLog2 = childSizeLog2;
  partitionParams.maxNumImQtbtBeforeOT = maxNumImQtbtBeforeOT;
  partitionParams.minSizeImQtbt = minSizeImQtbt;
  partitionParams.occupancySkip = occupancySkip;
  xir_skipOccupancy(occupancySkip, partitionParams.bitSkip);
  partitionParams.occupancySkipParent = occupancySkip;
  partitionParams.numNodesInCurrentDepth = 1;
  partitionParams.numNodesInNextDepth = 0;
}

static inline Void updateOctreePartitionParams(TComOctreePartitionParams& partitionParams) {
  partitionParams.depth++;
  partitionParams.numNodesInCurrentDepth = partitionParams.numNodesInNextDepth;
  partitionParams.numNodesInNextDepth = 0;

  ///< implicit qtbt for current node
  partitionParams.nodeSizeLog2 =
    partitionParams.childSizeLog2;  ///< child node size of previous depth is current node size

  if (partitionParams.maxNumImQtbtBeforeOT)
    partitionParams.maxNumImQtbtBeforeOT--;

  ///< if all dimensions have same size, then performing octree for remaining nodes
  if (partitionParams.nodeSizeLog2[0] == partitionParams.nodeSizeLog2[1] &&
      partitionParams.nodeSizeLog2[1] == partitionParams.nodeSizeLog2[2])
    partitionParams.minSizeImQtbt = 0;

  partitionParams.occupancySkipParent = partitionParams.occupancySkip;
  imQtBtDecision(partitionParams.childSizeLog2, partitionParams.maxNumImQtbtBeforeOT,
                 partitionParams.minSizeImQtbt);

  prepareImQtBtIndicator(partitionParams.occupancySkip, partitionParams.nodeSizeLog2,
                         partitionParams.childSizeLog2);
  xir_skipOccupancy(partitionParams.occupancySkip, partitionParams.bitSkip);
}

static inline Bool judgeSingleMode(const V3<UInt>& nodeSizeLog2) {
  Int zeroCnt = 0;
  Int nZeroSum = 0;
  for (Int i = 0; i < 3; ++i) {
    if (nodeSizeLog2[i] != 0) {
      ++zeroCnt;
      nZeroSum += nodeSizeLog2[i];
    }
  }
  return (nZeroSum > zeroCnt * 2);
}

///< \}
