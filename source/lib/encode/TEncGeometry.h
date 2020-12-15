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

#include "TEncBacTop.h"
#include "common/HighLevelSyntax.h"
#include "common/TComOccupancyMap.h"
#include "common/TComOctree.h"
#include "common/TComPointCloud.h"
#include "common/contributors.h"
#include <queue>

///< \in TLibEncoder

/**
 * Implementation of TEncGeometry
 * geometry encoder
 */

class TEncGeometry {
private:
  TComPointCloud* m_pointCloudOrg;               ///< pointer to input point cloud
  TComPointCloud* m_pointCloudRecon;             ///< pointer to output point cloud
  UInt m_numReconPoints;                         ///< number of reconstructed points
  HighLevelSyntax* m_hls;                        ///< pointer to high-level syntax parameters
  TEncBacTop* m_encBac;                          ///< pointer to bac
  unique_ptr<TComOccupancyMap> m_historyMap[2];  ///< context information

public:
  TEncGeometry() = default;
  ~TEncGeometry() = default;

  Void init(TComPointCloud* pointCloudOrg, TComPointCloud* pointCloudRecon, HighLevelSyntax* hls,
            TEncBacTop* encBac);

  Int compressAndEncodeGeometry();
  Void clear();

private:
  Void initFifoWithRootNode(queue<TComOctreeNode>& fifo);
  Void breadthFirstOctreeNode(const TComOctreeNode& currentNode, TComOctreePartitionParams& params,
                              queue<TComOctreeNode>& fifo);
  Void handleSingleMode(const TComOctreeNode& currentNode, const V3<UInt>& nodeSizeLog2,
                        const UInt& occupancySkip, TComGeomContext& geomCtx,
                        const TComOctreePartitionParams& params);
  Void encodeLeafNode(const vector<UInt>& childPointIdxBuf);
  Void initChildNode(const TComOctreeNode& currentNode, TComOctreeNode& childNode,
                     const V3<UInt>& childSizeLog2, const int childIdx,
                     const vector<UInt>& childPointIdxBuf);
  Void treePartition(const TComOctreeNode& currentNode, const V3<UInt>& childNodeSizeLog2,
                     const UInt& occupancySkip, vector<UInt>* childPointIdx);
  Void depthFirstPartition(const TComOctreeNode& currentNode, const V3<UInt>& nodeSizeLog2,
                           UInt maxNumImQtbtBeforeOT, UInt minSizeImQtbt,
                           const TComOctreePartitionParams& params);
  Void encodeWithMulContext(const UInt& occupancyCode, const TComGeomContext& geomCtx,
                            const unique_ptr<TComOccupancyMap>* occupancyMap,
                            const TComOctreePartitionParams& params,
                            const TComOctreeNode& currentNode);

};  ///< END CLASS TEncGeometry

///< \}
