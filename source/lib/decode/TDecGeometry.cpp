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

#include "TDecGeometry.h"
#include "common/contributors.h"
#include <algorithm>
#include <queue>

///< \in TLibDecoder \{

/**
 * Implementation of TDecGeometry
 * geometry decoder
 */

//////////////////////////////////////////////////////////////////////////
// Public class functions
//////////////////////////////////////////////////////////////////////////

Void TDecGeometry::decodeAndDecompress() {
  TComPointCloud& pcRec = *m_pointCloudRecon;

  queue<TComOctreeNode> fifo;

  TComOctreeNode rootNode;
  rootNode.pos = (UInt)0;
  fifo.push(rootNode);

  pcRec.setNumPoint(m_hls->gbh.geomNumPoints);

  TComOctreePartitionParams partitionParams;
  initOctreePartitionParams(*m_hls, partitionParams);
  m_historyMap[0] = unique_ptr<TComOccupancyMap>(new TComOccupancyMap());
  m_historyMap[1] = unique_ptr<TComOccupancyMap>(new TComOccupancyMap());

  m_numReconPoints = 0;

  for (; !fifo.empty(); fifo.pop()) {
    TComOctreeNode& currentNode = fifo.front();

    ///< start depth-first coding mode
    if (partitionParams.nodeSizeLog2.max() < m_hls->gps.depthFirstNodeSizeLog2 &&
        partitionParams.nodeSizeLog2.max() > 0) {
      depthFirstPartition(currentNode, partitionParams.nodeSizeLog2,
                          partitionParams.maxNumImQtbtBeforeOT, partitionParams.minSizeImQtbt);
      continue;
    }

    breadthFirstOctreeNode(currentNode, partitionParams, fifo);

    partitionParams.numNodesInCurrentDepth--;
    if (partitionParams.numNodesInCurrentDepth == 0) {
      updateOctreePartitionParams(partitionParams);
      m_historyMap[0] = std::move(m_historyMap[1]);
      m_historyMap[1] = unique_ptr<TComOccupancyMap>(new TComOccupancyMap());
    }
  }
  pcRec.setNumPoint(m_numReconPoints);
  return;
}

Void TDecGeometry::init(TComPointCloud* pointCloudRecon, HighLevelSyntax* hls, TDecBacTop* decBac) {
  m_pointCloudRecon = pointCloudRecon;
  m_hls = hls;
  m_decBac = decBac;
}

Void TDecGeometry::clear() {
  m_historyMap[0].release();
  m_historyMap[1].release();
}

//////////////////////////////////////////////////////////////////////////
// Private class functions
//////////////////////////////////////////////////////////////////////////

Void TDecGeometry::breadthFirstOctreeNode(const TComOctreeNode& currentNode,
                                          TComOctreePartitionParams& params,
                                          queue<TComOctreeNode>& fifo) {
  TComGeomContext geomCtx;
  getContextInfor(m_historyMap, params, currentNode, geomCtx, m_hls->gps.geom_context_mode);
  UInt occupancyCode = 0;
  if (m_hls->gps.geom_context_mode) {
    occupancyCode = decodeWithMulContext(geomCtx, m_historyMap, params, currentNode);
  } else {
    occupancyCode = m_decBac->decodeOccupancyCode(params.occupancySkip, geomCtx);
  }

  updateContextInfor(m_historyMap[1], params, currentNode, occupancyCode);
  if (occupancyCode == 0) {
    handleSingleMode(currentNode, params.nodeSizeLog2, params.childSizeLog2, occupancyCode);
  } else {
    for (Int i = 0; i < 8; i++) {
      if (!(occupancyCode & (1 << i)))
        continue;

      Int x = !!(i & 4);
      Int y = !!(i & 2);
      Int z = !!(i & 1);

      if (params.childSizeLog2 == 0) {  ///< reaching the leaf nodes
        decodeLeafNode(currentNode, x, y, z);
        continue;
      }

      /// create new child
      fifo.emplace();
      auto& childNode = fifo.back();

      childNode.pos[0] = currentNode.pos[0] + (x << params.childSizeLog2[0]);
      childNode.pos[1] = currentNode.pos[1] + (y << params.childSizeLog2[1]);
      childNode.pos[2] = currentNode.pos[2] + (z << params.childSizeLog2[2]);

      params.numNodesInNextDepth++;
    }
  }
}

Void TDecGeometry::handleSingleMode(const TComOctreeNode& currentNode, const V3<UInt> nodeSizeLog2,
                                    const V3<UInt> childSizeLog2, UInt& occupancyCode) {
  if (!m_hls->gps.singleModeFlag)
    assert(0);
  TComPointCloud& pcRec = *m_pointCloudRecon;
  V3<UInt> lowerPos = m_decBac->decodeSinglePointIndex(nodeSizeLog2);
  pcRec[m_numReconPoints][0] = (Double)currentNode.pos[0] + lowerPos[0];
  pcRec[m_numReconPoints][1] = (Double)currentNode.pos[1] + lowerPos[1];
  pcRec[m_numReconPoints][2] = (Double)currentNode.pos[2] + lowerPos[2];
  ++m_numReconPoints;

  occupancyCode = 1 << ((lowerPos[0] >> childSizeLog2[0]) << 2) +
      ((lowerPos[1] >> childSizeLog2[1]) << 1) + ((lowerPos[2] >> childSizeLog2[2]));
}

Void TDecGeometry::decodeLeafNode(const TComOctreeNode& currentNode, const Int x, const Int y,
                                  const Int z) {
  TComPointCloud& pcRec = *m_pointCloudRecon;

  const PC_POS point({Double(currentNode.pos[0]) + x, Double(currentNode.pos[1]) + y,
                      Double(currentNode.pos[2]) + z});
  if (m_hls->sps.geomRemoveDuplicateFlag) {
    pcRec[m_numReconPoints++] = point;
  } else {
    UInt dupNum = m_decBac->decodeDuplicateNumber();
    for (UInt i = 0; i < dupNum; i++)
      pcRec[m_numReconPoints++] = point;
  }
}

Void TDecGeometry::depthFirstPartition(const TComOctreeNode& currentNode,
                                       const V3<UInt>& nodeSizeLog2, UInt maxNumImQtbtBeforeOT,
                                       UInt minSizeImQtbt) {
  if (nodeSizeLog2 == 0)
    return;

  TComPointCloud& pcRec = *m_pointCloudRecon;

  V3<UInt> childSizeLog2 = nodeSizeLog2;
  imQtBtDecision(childSizeLog2, maxNumImQtbtBeforeOT, minSizeImQtbt);

  if (maxNumImQtbtBeforeOT)
    maxNumImQtbtBeforeOT--;

  ///< if all dimensions have same size, then performing octree for remaining nodes
  if (nodeSizeLog2[0] == nodeSizeLog2[1] && nodeSizeLog2[1] == nodeSizeLog2[2])
    minSizeImQtbt = 0;

  UInt occupancySkip;
  prepareImQtBtIndicator(occupancySkip, nodeSizeLog2, childSizeLog2);

  TComGeomContext geomCtx;
  UInt occupancyCode = m_decBac->decodeOccupancyCode(occupancySkip, geomCtx);
  if (occupancyCode == 0) {
    handleSingleMode(currentNode, nodeSizeLog2, childSizeLog2, occupancyCode);
  } else {
    for (Int i = 0; i < 8; i++) {
      if (!(occupancyCode & (1 << i)))
        continue;

      Int x = !!(i & 4);
      Int y = !!(i & 2);
      Int z = !!(i & 1);

      if (childSizeLog2 == 0) {  ///< reaching the leaf nodes
        decodeLeafNode(currentNode, x, y, z);
        continue;
      }

      /// create new child
      TComOctreeNode childNode;
      childNode.pos[0] = currentNode.pos[0] + (x << childSizeLog2[0]);
      childNode.pos[1] = currentNode.pos[1] + (y << childSizeLog2[1]);
      childNode.pos[2] = currentNode.pos[2] + (z << childSizeLog2[2]);

      depthFirstPartition(childNode, childSizeLog2, maxNumImQtbtBeforeOT, minSizeImQtbt);
    }
  }
}

UInt TDecGeometry::decodeWithMulContext(const TComGeomContext& geomCtx,
                                        const unique_ptr<TComOccupancyMap>* occupancyMap,
                                        const TComOctreePartitionParams& params,
                                        const TComOctreeNode& currentNode) {
  int pattern = geomCtx.ctxParent;
  UInt occupancyCode = 0;
  uint32_t bit_pattern[8] = {0, 0, 0, 0, 0, 0, 0, 0};
  uint32_t extern_pattern[8] = {0, 0, 0, 0, 0, 0, 0, 0};
  uint32_t map_bit_pattern[8] = {0, 0, 0, 0, 0, 0, 0, 0};
  childPatternMerge stroge[8] = {0};
  int childOrder[8] = {0, 2, 4, 6, 1, 3, 5, 7};
  int update_occupancy[8] = {0, 0, 0, 0, 0, 0, 0, 0};
  for (int i = 0; i < 8; i++) {
    int bitIdx = childOrder[i];
    if (params.bitSkip[bitIdx]) {
      continue;
    } else {
      int bit_pattern = 0;
      int extern_pattern = 0;
      stroge[bitIdx] = occupancy_child(params, currentNode, bitIdx, update_occupancy, geomCtx);
      bit_pattern |= stroge[bitIdx].childX;
      bit_pattern |= stroge[bitIdx].childY ? 2 : 0;
      bit_pattern |= stroge[bitIdx].childZ ? 4 : 0;
      extern_pattern |= stroge[bitIdx].leanX;
      extern_pattern |= stroge[bitIdx].leanY ? 2 : 0;
      extern_pattern |= stroge[bitIdx].leanZ ? 4 : 0;
      extern_pattern = (extern_pattern << 1) + stroge[bitIdx].Dot;
      int bit = m_decBac->decodeOccupancyCode(bitIdx, bit_pattern, extern_pattern, pattern);
      occupancyCode |= (bit << bitIdx);
      update_occupancy[bitIdx] = bit;
    }
  }
  return occupancyCode;
}

///< \}
