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

#include "PointCloudMortonTable.h"
#include "TComHashMap.h"
#include "TComOctree.h"
#include "TComPointCloud.h"
#include "TComVector.h"
#include "TypeDef.h"
#include <assert.h>
#include <memory>

using namespace std;

///< \in TLibCommon \{

/**
 * Class TComOccupancyMap
 * geometry occupancy map
 */

struct OccupancyInfo {
  UInt8 occupancy;

  OccupancyInfo(const UInt8 occupancy)
    : occupancy(occupancy) {}

  OccupancyInfo()
    : occupancy(0) {}
};

struct TComGeomContext {
  UInt ctxParent;
  UInt ctxParentAdv[8];
  UInt ctxChildD1[6];
  UInt ctxPlane[7][8];
  TComGeomContext() {
    ctxParent = 0;
    fill(begin(ctxChildD1), end(ctxChildD1), 0);
    fill(begin(ctxParentAdv), end(ctxParentAdv), 0);
    fill(ctxPlane[0], ctxPlane[0] + 7 * 8, 0);
  }
};
struct childPatternMerge {
  UInt childX;
  UInt childY;
  UInt childZ;
  UInt leanX;
  UInt leanY;
  UInt leanZ;
  UInt Dot;
};

class TComOccupancyMap {
private:
  unique_ptr<TComHashMapDynamic<OccupancyInfo>> m_map;

public:
  TComOccupancyMap() {
    init();
  }

  ~TComOccupancyMap() {
    clear();
  }

  Void init() {
    m_map = unique_ptr<TComHashMapDynamic<OccupancyInfo>>(new TComHashMapDynamic<OccupancyInfo>);
  }

  Void clear() {
    m_map.release();
  }

  unique_ptr<TComHashMapDynamic<OccupancyInfo>>& getMap() {
    return m_map;
  }

  Bool getChildOccupied(const Int32 x, const Int32 y, const Int32 z) const;

  Bool getChildOccupied(const Int32 x, const Int32 y, const Int32 z,
                        const UInt occupancySkipParent) const;

  Bool getChildOccupied(const Int32 x, const Int32 y, const Int32 z, const Int shiftX,
                        const Int shiftY, const Int shiftZ) const;

  UInt8 getOccupancy(const Int32 x, const Int32 y, const Int32 z) const;

  Void insert(const Int32 x, const Int32 y, const Int32 z, const UInt8 occupancyCode = 0);

private:
  Int getBitIndex(const Int32 x, const Int32 y, const Int32 z) const;

};  ///< END CLASS TComOccupancyMap

//////////////////////////////////////////////////////////////////////////
// Inline functions
//////////////////////////////////////////////////////////////////////////

inline Void getContextInfor(const unique_ptr<TComOccupancyMap>* occupancyMap,
                            const TComOctreePartitionParams& params,
                            const TComOctreeNode& currentNode, TComGeomContext& geomCtx,
                            const UInt contextMode) {
  UInt depth = params.depth;
  UInt occupancySkip = params.occupancySkip;
  UInt occupancySkipParent = params.occupancySkipParent;
  V3<UInt> nodeSizeLog2 = params.nodeSizeLog2;

  //< get context infor from neighboring parent nodes
  UInt* ctxParent = &geomCtx.ctxParent;
  UInt* ctxParentAdv = geomCtx.ctxParentAdv;
  UInt* ctxChildD1 = geomCtx.ctxChildD1;
  UInt tempPlane[8] = {0};

  V3<UInt> pos = currentNode.pos;
  V3<UInt> pos_ = pos >> nodeSizeLog2;
  const Int x = pos_[0];
  const Int y = pos_[1];
  const Int z = pos_[2];

  if (depth > 0) {
    const TComOccupancyMap& om = *occupancyMap[0];

    if (contextMode == 1) {
      if (!occupancySkipParent) {
        *ctxParent |= om.getChildOccupied(x + 1, y, z) ? 4 : 0;
        *ctxParent |= om.getChildOccupied(x, y + 1, z) ? 2 : 0;
        *ctxParent |= om.getChildOccupied(x, y, z + 1) ? 1 : 0;

      } else {
        const Int shiftX = (occupancySkipParent & 4) ? 0 : 1;
        const Int shiftY = (occupancySkipParent & 2) ? 0 : 1;
        const Int shiftZ = (occupancySkipParent & 1) ? 0 : 1;
        *ctxParent |= om.getChildOccupied(x + 1, y, z, shiftX, shiftY, shiftZ) ? 4 : 0;
        *ctxParent |= om.getChildOccupied(x, y + 1, z, shiftX, shiftY, shiftZ) ? 2 : 0;
        *ctxParent |= om.getChildOccupied(x, y, z + 1, shiftX, shiftY, shiftZ) ? 1 : 0;
      }
    } else {
      // get occupancy info from 18 parent-level neighbors
      Bool parentCtx27[27] = {false};
      for (Int i = 0; i < 18; i++) {
        int dx = neighborDeltaX[i];
        int dy = neighborDeltaY[i];
        int dz = neighborDeltaZ[i];
        parentCtx27[neighborIndex[i]] =
          om.getChildOccupied(x + dx, y + dy, z + dz, occupancySkipParent);
      }

      for (Int iChild = 0; iChild < 8; iChild++) {
        if (occupancySkip != 0) {
          if ((occupancySkip & 1) && (iChild & 1))  ///< skip when z = 1
            continue;
          if ((occupancySkip & 2) && (iChild & 2))  ///< skip when y = 1
            continue;
          if ((occupancySkip & 4) && (iChild & 4))  ///< skip when x = 1
            continue;
        }
        // check 3 nearest parent-level neighbors sharing a face with current child node
        for (Int i = 0; i < 3; i++) {
          ctxParentAdv[iChild] |= parentCtx27[childNeighborIdx[iChild][i]] << i;
        }
        // check 3 nearest parent-level neighbors sharing a edge with current child node
        ctxParentAdv[iChild] <<= 2;
        for (Int i = 3; i < 6; i++) {
          ctxParentAdv[iChild] += parentCtx27[childNeighborIdx[iChild][i]];
        }
      }
    }
  }

  //< get context infor from neighboring child nodes
  TComOccupancyMap& om = *occupancyMap[1];
  UInt8 occNei;

  if (contextMode == 1) {
    if (om.getMap()->isAtBoundaryCheckPoint(pos_)) {
      om.getMap()->shrinkSize();
      om.getMap()->setBoundaryCheckPoint(pos_);
    }
  }

  if (contextMode) {
    tempPlane[0] = om.getOccupancy(x - 1, y, z);
    tempPlane[1] = om.getOccupancy(x, y - 1, z);
    tempPlane[2] = om.getOccupancy(x, y, z - 1);
    tempPlane[3] = om.getOccupancy(x - 1, y - 1, z);
    tempPlane[4] = om.getOccupancy(x - 1, y, z - 1);
    tempPlane[5] = om.getOccupancy(x, y - 1, z - 1);
    tempPlane[6] = om.getOccupancy(x - 1, y - 1, z - 1);

    tempPlane[0] = (tempPlane[0] & 0xF0) >> 4;
    geomCtx.ctxPlane[0][4] = tempPlane[0] & 1;
    geomCtx.ctxPlane[0][5] = (tempPlane[0] & 2) ? 1 : 0;
    geomCtx.ctxPlane[0][6] = (tempPlane[0] & 4) ? 1 : 0;
    geomCtx.ctxPlane[0][7] = (tempPlane[0] & 8) ? 1 : 0;

    geomCtx.ctxPlane[1][1] = (tempPlane[1] & 2) ? 1 : 0;
    geomCtx.ctxPlane[1][2] = (tempPlane[1] & 4) ? 1 : 0;
    geomCtx.ctxPlane[1][3] = (tempPlane[1] & 8) ? 1 : 0;
    geomCtx.ctxPlane[1][4] = (tempPlane[1] & 16) ? 1 : 0;
    geomCtx.ctxPlane[1][6] = (tempPlane[1] & 64) ? 1 : 0;
    geomCtx.ctxPlane[1][7] = (tempPlane[1] & 128) ? 1 : 0;

    geomCtx.ctxPlane[2][1] = (tempPlane[2] & 2) ? 1 : 0;
    geomCtx.ctxPlane[2][3] = (tempPlane[2] & 8) ? 1 : 0;
    geomCtx.ctxPlane[2][5] = (tempPlane[2] & 32) ? 1 : 0;
    geomCtx.ctxPlane[2][7] = (tempPlane[2] & 128) ? 1 : 0;

    geomCtx.ctxPlane[3][4] = (tempPlane[3] & 16) ? 1 : 0;
    geomCtx.ctxPlane[3][6] = (tempPlane[3] & 64) ? 1 : 0;
    geomCtx.ctxPlane[3][7] = (tempPlane[3] & 128) ? 1 : 0;

    geomCtx.ctxPlane[4][5] = (tempPlane[4] & 32) ? 1 : 0;
    geomCtx.ctxPlane[4][7] = (tempPlane[4] & 128) ? 1 : 0;

    geomCtx.ctxPlane[5][3] = (tempPlane[5] & 8) ? 1 : 0;
    geomCtx.ctxPlane[5][5] = (tempPlane[5] & 32) ? 1 : 0;
    geomCtx.ctxPlane[5][7] = (tempPlane[5] & 128) ? 1 : 0;

    geomCtx.ctxPlane[6][7] = (tempPlane[6] & 128) ? 1 : 0;
  } else {
    ///< neighboring node (dx = -1)
    occNei = om.getOccupancy(x - 1, y, z);
    if (!(occupancySkip & 4))
      occNei = (occNei & 0xF0) >> 4;
    else
      occNei = (occNei & 0x0F);
    ctxChildD1[0] |= occNei;

    ///< neighboring node (dy = -1)
    occNei = om.getOccupancy(x, y - 1, z);
    if (!(occupancySkip & 2))
      occNei = (occNei & 0xCC) >> 2;
    else
      occNei = (occNei & 0x33);
    ctxChildD1[1] |= occNei;

    ///< neighboring node (dz = -1)
    occNei = om.getOccupancy(x, y, z - 1);
    if (!(occupancySkip & 1))
      occNei = (occNei & 0xAA) >> 1;
    else
      occNei = (occNei & 0x55);
    ctxChildD1[2] |= occNei;

    *ctxParent = 0;
    return;
  }
}

/**
  * update context infor
  */
inline Void updateContextInfor(unique_ptr<TComOccupancyMap>& occupancyMap,
                               const TComOctreePartitionParams& params,
                               const TComOctreeNode& currentNode, UInt occupancyCode) {
  V3<UInt> pos = currentNode.pos;
  const Int x = pos[0] >> params.nodeSizeLog2[0];
  const Int y = pos[1] >> params.nodeSizeLog2[1];
  const Int z = pos[2] >> params.nodeSizeLog2[2];

  occupancyMap->insert(x, y, z, occupancyCode);
}
childPatternMerge occupancy_child(const TComOctreePartitionParams& params,
                                  const TComOctreeNode& currentNode, int bitIdx, int* update_bit,
                                  const TComGeomContext& geomCtx);

///< \}
