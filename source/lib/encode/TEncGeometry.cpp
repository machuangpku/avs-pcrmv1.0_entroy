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

#include "TEncGeometry.h"
#include "common/CommonDef.h"
#include "common/contributors.h"
#include <algorithm>
#include <fstream>
#include <queue>

///< \in TLibEncoder

/**
 * Implementation of TEncGeometry
 * geometry encoder
 */

//////////////////////////////////////////////////////////////////////////
// Public class functions
//////////////////////////////////////////////////////////////////////////

Void TEncGeometry::init(TComPointCloud* pointCloudOrg, TComPointCloud* pointCloudRecon,
                        HighLevelSyntax* hls, TEncBacTop* encBac) {
  m_pointCloudOrg = pointCloudOrg;
  m_pointCloudRecon = pointCloudRecon;
  m_pointCloudRecon->setNumPoint(m_pointCloudOrg->getNumPoint());
  m_hls = hls;
  m_encBac = encBac;

  if (pointCloudOrg->hasColors()) {
    pointCloudRecon->addColors();
  }
  if (pointCloudOrg->hasReflectances()) {
    pointCloudRecon->addReflectances();
  }
}

Void TEncGeometry::clear() {
  m_historyMap[0].release();
  m_historyMap[1].release();
}

Int TEncGeometry::compressAndEncodeGeometry() {
  const TComPointCloud& pcOrg = *m_pointCloudOrg;
  TComPointCloud& pcRec = *m_pointCloudRecon;

  queue<TComOctreeNode> fifo;
  initFifoWithRootNode(fifo);

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
                          partitionParams.maxNumImQtbtBeforeOT, partitionParams.minSizeImQtbt,
                          partitionParams);
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
  return 0;
}

//////////////////////////////////////////////////////////////////////////
// Private class functions
//////////////////////////////////////////////////////////////////////////

Void TEncGeometry::initFifoWithRootNode(queue<TComOctreeNode>& fifo) {
  const TComPointCloud& pcOrg = *m_pointCloudOrg;
  TComOctreeNode rootNode;
  rootNode.pos = 0;

  for (int i = 0; i < pcOrg.getNumPoint(); i++) {
    rootNode.pointIdx.push_back(i);
  }
  fifo.push(rootNode);
}

Void TEncGeometry::breadthFirstOctreeNode(const TComOctreeNode& currentNode,
                                          TComOctreePartitionParams& params,
                                          queue<TComOctreeNode>& fifo) {
  vector<UInt> childPointIdx[8];
  treePartition(currentNode, params.childSizeLog2, params.occupancySkip, childPointIdx);

  UInt occupancyCode = 0;
  for (Int i = 0; i < 8; i++) {
    if (childPointIdx[i].size() > 0) {
      occupancyCode |= 1 << i;
    }
  }

  TComGeomContext geomCtx;
  getContextInfor(m_historyMap, params, currentNode, geomCtx, m_hls->gps.geom_context_mode);

  if (m_hls->gps.singleModeFlag && judgeSingleMode(params.nodeSizeLog2) &&
      (currentNode.pointIdx.size() == 1)) {
    handleSingleMode(currentNode, params.nodeSizeLog2, params.occupancySkip, geomCtx, params);
    updateContextInfor(m_historyMap[1], params, currentNode, 0);

  } else {  // not a singleMode
    if (m_hls->gps.geom_context_mode) {
      encodeWithMulContext(occupancyCode, geomCtx, m_historyMap, params, currentNode);
    } else {
      m_encBac->encodeOccupancyCode(occupancyCode, params.occupancySkip, geomCtx);
    }

    updateContextInfor(m_historyMap[1], params, currentNode, occupancyCode);

    for (Int i = 0; i < 8; i++) {
      if (childPointIdx[i].size() == 0)
        continue;

      if (params.childSizeLog2 == 0) {
        encodeLeafNode(childPointIdx[i]);
        continue;
      }

      /// create new child
      fifo.emplace();
      auto& childNode = fifo.back();
      initChildNode(currentNode, childNode, params.childSizeLog2, i, childPointIdx[i]);
      params.numNodesInNextDepth++;
    }
  }
}

Void TEncGeometry::handleSingleMode(const TComOctreeNode& currentNode, const V3<UInt>& nodeSizeLog2,
                                    const UInt& occupancySkip, TComGeomContext& geomCtx,
                                    const TComOctreePartitionParams& params) {
  assert(currentNode.pointIdx.size() == 1);
  const TComPointCloud& pcOrg = *m_pointCloudOrg;
  TComPointCloud& pcRec = *m_pointCloudRecon;
  if (m_hls->gps.geom_context_mode) {
    encodeWithMulContext(0, geomCtx, m_historyMap, params, currentNode);
  } else {
    m_encBac->encodeOccupancyCode(0, occupancySkip, geomCtx);
  }

  V3<UInt> pos;
  pos[0] = (UInt)pcOrg[currentNode.pointIdx[0]][0];
  pos[1] = (UInt)pcOrg[currentNode.pointIdx[0]][1];
  pos[2] = (UInt)pcOrg[currentNode.pointIdx[0]][2];
  m_encBac->encodeSinglePointIndex(pos, nodeSizeLog2);
  pcRec[m_numReconPoints] = pcOrg[currentNode.pointIdx[0]];
  if (!m_hls->sps.geomRemoveDuplicateFlag || m_hls->sps.recolorMode) {
    ///< keep duplicate points
    if (pcOrg.hasColors()) {
      const auto color0 = pcOrg.getColor(currentNode.pointIdx[0]);
      pcRec.setColor(m_numReconPoints, color0);
    }
    if (pcOrg.hasReflectances()) {
      const auto reflectance0 = pcOrg.getReflectance(currentNode.pointIdx[0]);
      pcRec.setReflectance(m_numReconPoints, reflectance0);
    }
  }
  ++m_numReconPoints;
}

Void TEncGeometry::encodeLeafNode(const vector<UInt>& childPointIdxBuf) {
  const TComPointCloud& pcOrg = *m_pointCloudOrg;
  TComPointCloud& pcRec = *m_pointCloudRecon;

  ///< reaching the leaf nodes
  if (m_hls->sps.geomRemoveDuplicateFlag) {
    ///< remove duplicate points
    assert(childPointIdxBuf.size() == 1);
    const auto& idx = childPointIdxBuf[0];
    pcRec[m_numReconPoints] = pcOrg[idx];
    if (pcOrg.hasColors()) {
      const auto color0 = pcOrg.getColor(idx);
      pcRec.setColor(m_numReconPoints, color0);
    }
    if (pcOrg.hasReflectances()) {
      const auto reflectance0 = pcOrg.getReflectance(idx);
      pcRec.setReflectance(m_numReconPoints, reflectance0);
    }
    m_numReconPoints++;
  } else {
    ///< keep duplicate points
    m_encBac->encodeDuplicateNumber(UInt(childPointIdxBuf.size()));

    for (const auto& idx : childPointIdxBuf) {
      pcRec[m_numReconPoints] = pcOrg[idx];
      if (pcOrg.hasColors()) {
        const auto color0 = pcOrg.getColor(idx);
        pcRec.setColor(m_numReconPoints, color0);
      }
      if (pcOrg.hasReflectances()) {
        const auto reflectance0 = pcOrg.getReflectance(idx);
        pcRec.setReflectance(m_numReconPoints, reflectance0);
      }
      m_numReconPoints++;
    }
  }
}

Void TEncGeometry::initChildNode(const TComOctreeNode& currentNode, TComOctreeNode& childNode,
                                 const V3<UInt>& childSizeLog2, const int childIdx,
                                 const vector<UInt>& childPointIdxBuf) {
  Int x = !!(childIdx & 4);
  Int y = !!(childIdx & 2);
  Int z = !!(childIdx & 1);

  childNode.pos[0] = currentNode.pos[0] + (x << childSizeLog2[0]);
  childNode.pos[1] = currentNode.pos[1] + (y << childSizeLog2[1]);
  childNode.pos[2] = currentNode.pos[2] + (z << childSizeLog2[2]);

  childNode.pointIdx = childPointIdxBuf;
}

Void TEncGeometry::treePartition(const TComOctreeNode& currentNode,
                                 const V3<UInt>& childNodeSizeLog2, const UInt& occupancySkip,
                                 vector<UInt>* childPointIdx) {
  const auto& pointIdx = currentNode.pointIdx;
  V3<UInt> bitMask = 1 << childNodeSizeLog2;
  for (int k = 0; k < 3; k++) {
    if (occupancySkip & (4 >> k))
      bitMask[k] = 0;
  }

  for (auto idx : pointIdx) {
    const auto& pos = (*m_pointCloudOrg)[idx];
    Int childNodeIdx = (!!(Int(pos[0]) & bitMask[0])) << 2;
    childNodeIdx |= (!!(Int(pos[1]) & bitMask[1])) << 1;
    childNodeIdx |= (!!(Int(pos[2]) & bitMask[2]));

    childPointIdx[childNodeIdx].push_back(idx);
  }
}

Void TEncGeometry::depthFirstPartition(const TComOctreeNode& currentNode,
                                       const V3<UInt>& nodeSizeLog2, UInt maxNumImQtbtBeforeOT,
                                       UInt minSizeImQtbt,
                                       const TComOctreePartitionParams& params) {
  if (nodeSizeLog2 == 0)
    return;

  const TComPointCloud& pcOrg = *m_pointCloudOrg;
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

  if (m_hls->gps.singleModeFlag && judgeSingleMode(nodeSizeLog2) &&
      (currentNode.pointIdx.size() == 1)) {
    handleSingleMode(currentNode, nodeSizeLog2, occupancySkip, geomCtx, params);
  } else {
    vector<UInt> childPointIdx[8];
    treePartition(currentNode, childSizeLog2, occupancySkip, childPointIdx);

    UInt occupancyCode = 0;
    for (Int i = 0; i < 8; i++) {
      if (childPointIdx[i].size() > 0)
        occupancyCode |= 1 << i;
    }

    m_encBac->encodeOccupancyCode(occupancyCode, occupancySkip, geomCtx);

    for (Int i = 0; i < 8; i++) {
      if (childPointIdx[i].size() == 0)
        continue;

      if (childSizeLog2 == 0) {  ///< reaching the leaf nodes
        encodeLeafNode(childPointIdx[i]);
        continue;
      }

      TComOctreeNode childNode;
      initChildNode(currentNode, childNode, childSizeLog2, i, childPointIdx[i]);
      depthFirstPartition(childNode, childSizeLog2, maxNumImQtbtBeforeOT, minSizeImQtbt, params);
    }
  }
}

Void TEncGeometry::encodeWithMulContext(const UInt& occupancyCode, const TComGeomContext& geomCtx,
                                        const unique_ptr<TComOccupancyMap>* occupancyMap,
                                        const TComOctreePartitionParams& params,
                                        const TComOctreeNode& currentNode) {
  uint32_t bit_pattern[8] = {0, 0, 0, 0, 0, 0, 0, 0};
  uint32_t extern_pattern[8] = {0, 0, 0, 0, 0, 0, 0, 0};
  uint32_t map_bit_pattern[8] = {0, 0, 0, 0, 0, 0, 0, 0};
  childPatternMerge stroge[8] = {0};
  int pattern = geomCtx.ctxParent;
  int childOrder[8] = {0, 2, 4, 6, 1, 3, 5, 7};
  int update_occupancy[8] = {0, 0, 0, 0, 0, 0, 0, 0};
  for (int i = 0; i < 8; i++) {
    int bitIdx = childOrder[i];
    if (params.bitSkip[bitIdx]) {
      continue;
    } else {
      //new
      stroge[bitIdx] = occupancy_child(params, currentNode, bitIdx, update_occupancy, geomCtx);
      update_occupancy[bitIdx] = (occupancyCode >> bitIdx) & 1;
      int T_pattern = 0;
      int L_pattern = 0;
      T_pattern |= stroge[bitIdx].childX;
      T_pattern |= stroge[bitIdx].childY ? 2 : 0;
      T_pattern |= stroge[bitIdx].childZ ? 4 : 0;
      bit_pattern[bitIdx] = T_pattern;
      L_pattern |= stroge[bitIdx].leanX;
      L_pattern |= stroge[bitIdx].leanY ? 2 : 0;
      L_pattern |= stroge[bitIdx].leanZ ? 4 : 0;
      extern_pattern[bitIdx] = L_pattern;
      extern_pattern[bitIdx] = (L_pattern << 1) + stroge[bitIdx].Dot;
    }
  }
  m_encBac->encodeOccupancyCode(occupancyCode, params, bit_pattern, extern_pattern, pattern);
}

///< \}
