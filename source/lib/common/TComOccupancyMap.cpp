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

#include "TComOccupancyMap.h"

///< \in TLibCommon \{

/**
 * Class TComOccupancyMap
 * occupancy map for accessing neighborings
 */

//////////////////////////////////////////////////////////////////////////
// Public class functions
//////////////////////////////////////////////////////////////////////////

Bool TComOccupancyMap::getChildOccupied(const Int32 x, const Int32 y, const Int32 z,
                                        const UInt occupancySkipParent) const {
  if (occupancySkipParent == 0) {
    return getChildOccupied(x, y, z);
  } else {
    const Int shiftX = (occupancySkipParent & 4) ? 0 : 1;
    const Int shiftY = (occupancySkipParent & 2) ? 0 : 1;
    const Int shiftZ = (occupancySkipParent & 1) ? 0 : 1;
    return getChildOccupied(x, y, z, shiftX, shiftY, shiftZ);
  }
}

Bool TComOccupancyMap::getChildOccupied(const Int32 x, const Int32 y, const Int32 z) const {
  if (x < 0 || y < 0 || z < 0)
    return false;
  OccupancyInfo hi;
  if (m_map->get(x >> 1, y >> 1, z >> 1, hi)) {
    return (hi.occupancy >> getBitIndex(x, y, z)) & 1;
  }
  return false;
}

Bool TComOccupancyMap::getChildOccupied(const Int32 x, const Int32 y, const Int32 z,
                                        const Int shiftX, const Int shiftY,
                                        const Int shiftZ) const {
  if (x < 0 || y < 0 || z < 0)
    return false;
  OccupancyInfo hi;
  if (m_map->get(x >> shiftX, y >> shiftY, z >> shiftZ, hi)) {
    return (hi.occupancy >> getBitIndex(shiftX ? x : 0, shiftY ? y : 0, shiftZ ? z : 0)) & 1;
  }
  return false;
}

UInt8 TComOccupancyMap::getOccupancy(const Int32 x, const Int32 y, const Int32 z) const {
  if (x < 0 || y < 0 || z < 0)
    return 0;
  OccupancyInfo hi;
  if (m_map->get(x, y, z, hi)) {
    return hi.occupancy;
  }
  return 0;
}

Void TComOccupancyMap::insert(const Int32 x, const Int32 y, const Int32 z,
                              const UInt8 occupancyCode) {
  assert(x >= 0 && y >= 0 && z >= 0);
  m_map->insert(x, y, z, occupancyCode);
}

Int TComOccupancyMap::getBitIndex(const Int32 x, const Int32 y, const Int32 z) const {
  return ((x & 1) << 2) | ((y & 1) << 1) | (z & 1);
}

///< \}

childPatternMerge occupancy_child(const TComOctreePartitionParams& params,
                                  const TComOctreeNode& currentNode, int bitIdx, int* update_bit,
                                  const TComGeomContext& geomCtx) {
  UInt depth = params.depth;
  UInt occupancySkip = params.occupancySkip;
  UInt occupancySkipParent = params.occupancySkipParent;
  //< get context infor from neighboring child nodes
  //the current position
  V3<UInt> pos = currentNode.pos;
  const Int x = pos[0] >> params.nodeSizeLog2[0];
  const Int y = pos[1] >> params.nodeSizeLog2[1];
  const Int z = pos[2] >> params.nodeSizeLog2[2];
  // return the every child neigh information
  childPatternMerge stroge;
  const int deleteX = (occupancySkip & 4 ? 1 : 0);
  const int deleteY = (occupancySkip & 2 ? 1 : 0);
  const int deleteZ = (occupancySkip & 1 ? 1 : 0);

  switch (bitIdx) {
  case 0:
    stroge.childX = geomCtx.ctxPlane[0][4];
    stroge.childY = geomCtx.ctxPlane[1][2];
    stroge.childZ = geomCtx.ctxPlane[2][1];
    stroge.leanZ = geomCtx.ctxPlane[3][6];
    stroge.leanY = geomCtx.ctxPlane[4][5];
    stroge.leanX = geomCtx.ctxPlane[5][3];
    stroge.Dot = geomCtx.ctxPlane[6][7];
    break;
  case 2:
    stroge.childX = geomCtx.ctxPlane[0][6];
    stroge.childZ = geomCtx.ctxPlane[2][3];
    if (deleteY) {
      stroge.leanX = geomCtx.ctxPlane[5][3];
      stroge.leanY = geomCtx.ctxPlane[4][7];
      stroge.leanZ = geomCtx.ctxPlane[3][6];
      stroge.Dot = geomCtx.ctxPlane[6][7];
      stroge.childY = geomCtx.ctxPlane[1][2];
    } else {
      stroge.leanX = geomCtx.ctxPlane[2][1];
      stroge.leanY = geomCtx.ctxPlane[4][7];
      stroge.Dot = geomCtx.ctxPlane[4][5];
      stroge.leanZ = geomCtx.ctxPlane[0][4];
      stroge.childY = update_bit[0];
    }

    break;
  case 4:
    stroge.childY = geomCtx.ctxPlane[1][6];
    stroge.childZ = geomCtx.ctxPlane[2][5];

    if (deleteX) {
      stroge.leanX = geomCtx.ctxPlane[5][7];
      stroge.leanY = geomCtx.ctxPlane[4][5];
      stroge.leanZ = geomCtx.ctxPlane[3][6];
      stroge.Dot = geomCtx.ctxPlane[6][7];
      stroge.childX = geomCtx.ctxPlane[0][4];

    } else {
      stroge.leanX = geomCtx.ctxPlane[5][7];
      stroge.Dot = geomCtx.ctxPlane[5][3];
      stroge.leanY = geomCtx.ctxPlane[1][1];
      stroge.leanZ = geomCtx.ctxPlane[1][2];
      stroge.childX = update_bit[0];
    }

    break;

  case 6:
    if (deleteX) {
      stroge.childX = geomCtx.ctxPlane[0][6];
    } else {
      stroge.childX = update_bit[2];
    }

    if (deleteY) {
      stroge.childY = geomCtx.ctxPlane[1][6];
    } else {
      stroge.childY = update_bit[4];
    }

    stroge.childZ = geomCtx.ctxPlane[2][7];

    if (deleteX && deleteY) {
      stroge.leanZ = geomCtx.ctxPlane[3][6];
      stroge.leanX = geomCtx.ctxPlane[5][7];
      stroge.leanY = geomCtx.ctxPlane[4][7];
      stroge.Dot = geomCtx.ctxPlane[6][7];
    }
    if (deleteX && (!deleteY)) {
      stroge.leanZ = geomCtx.ctxPlane[0][4];
      stroge.leanY = geomCtx.ctxPlane[4][7];
      stroge.Dot = geomCtx.ctxPlane[4][5];
      stroge.leanX = geomCtx.ctxPlane[2][5];
    }
    if (deleteY && (!deleteX)) {
      stroge.leanX = geomCtx.ctxPlane[5][5];
      stroge.Dot = geomCtx.ctxPlane[5][3];
      stroge.leanY = geomCtx.ctxPlane[2][3];
      stroge.leanZ = geomCtx.ctxPlane[1][2];
    }
    if ((!deleteX) && (!deleteY)) {
      stroge.leanX = geomCtx.ctxPlane[2][5];
      stroge.leanY = geomCtx.ctxPlane[2][3];
      stroge.Dot = geomCtx.ctxPlane[2][1];
      stroge.leanZ = update_bit[0];
    }
    break;
  case 1:
    stroge.childX = geomCtx.ctxPlane[0][5];
    stroge.childY = geomCtx.ctxPlane[1][3];
    stroge.childZ = geomCtx.ctxPlane[2][1];

    if (deleteZ) {
      stroge.leanZ = geomCtx.ctxPlane[3][7];
      stroge.leanY = geomCtx.ctxPlane[4][5];
      stroge.leanX = geomCtx.ctxPlane[5][3];
      stroge.Dot = geomCtx.ctxPlane[6][7];
      stroge.childZ = geomCtx.ctxPlane[2][1];

    } else {
      stroge.leanX = geomCtx.ctxPlane[1][2];
      stroge.leanY = geomCtx.ctxPlane[0][4];
      stroge.leanZ = geomCtx.ctxPlane[3][7];
      stroge.Dot = geomCtx.ctxPlane[3][6];
      stroge.childZ = update_bit[0];
    }
    break;
  case 3:
    stroge.childX = geomCtx.ctxPlane[0][7];

    if (deleteY) {
      stroge.childY = geomCtx.ctxPlane[1][3];
    } else {
      stroge.childY = update_bit[1];
    }
    if (deleteZ) {
      stroge.childZ = geomCtx.ctxPlane[2][3];
    } else {
      stroge.childZ = update_bit[2];
    }

    if ((deleteY) && (deleteZ)) {
      stroge.leanX = geomCtx.ctxPlane[5][3];
      stroge.leanY = geomCtx.ctxPlane[4][7];
      stroge.leanZ = geomCtx.ctxPlane[3][7];
      stroge.Dot = geomCtx.ctxPlane[6][7];
    }
    if (deleteY) {
      stroge.leanX = geomCtx.ctxPlane[1][2];
      stroge.leanY = geomCtx.ctxPlane[0][6];
      stroge.leanZ = geomCtx.ctxPlane[3][7];
      stroge.Dot = geomCtx.ctxPlane[3][6];
    }
    if (deleteZ) {
      stroge.leanX = geomCtx.ctxPlane[2][1];
      stroge.leanY = geomCtx.ctxPlane[4][7];
      stroge.Dot = geomCtx.ctxPlane[4][5];
      stroge.leanZ = geomCtx.ctxPlane[0][5];
    }
    if ((!deleteY) && (!deleteZ)) {
      stroge.leanX = update_bit[0];
      stroge.leanY = geomCtx.ctxPlane[0][6];
      stroge.leanZ = geomCtx.ctxPlane[0][5];
      stroge.Dot = geomCtx.ctxPlane[0][4];
    }

    break;
  case 5:
    if (deleteX) {
      stroge.childX = geomCtx.ctxPlane[0][5];
    } else {
      stroge.childX = update_bit[1];
    }

    stroge.childY = geomCtx.ctxPlane[1][7];

    if (deleteZ) {
      stroge.childZ = geomCtx.ctxPlane[2][5];
    } else {
      stroge.childZ = update_bit[4];
    }

    if ((deleteX) && (deleteZ)) {
      stroge.leanX = geomCtx.ctxPlane[5][7];
      stroge.leanY = geomCtx.ctxPlane[4][5];
      stroge.leanZ = geomCtx.ctxPlane[3][7];
      stroge.Dot = geomCtx.ctxPlane[6][7];
    }
    if (deleteX) {
      stroge.leanX = geomCtx.ctxPlane[1][6];
      stroge.leanY = geomCtx.ctxPlane[0][4];
      stroge.leanZ = geomCtx.ctxPlane[3][7];
      stroge.Dot = geomCtx.ctxPlane[3][4];
    }
    if (deleteZ) {
      stroge.leanX = geomCtx.ctxPlane[5][7];
      stroge.Dot = geomCtx.ctxPlane[5][3];
      stroge.leanY = geomCtx.ctxPlane[2][1];
      stroge.leanZ = geomCtx.ctxPlane[1][3];
    }
    if ((!deleteZ) && (!deleteX)) {
      stroge.leanX = geomCtx.ctxPlane[1][6];
      stroge.leanY = update_bit[0];
      stroge.leanZ = geomCtx.ctxPlane[1][3];
      stroge.Dot = geomCtx.ctxPlane[1][2];
    }
    break;

  case 7:
    if (deleteX) {
      stroge.childX = geomCtx.ctxPlane[0][7];
    } else {
      stroge.childX = update_bit[3];
    }
    if (deleteY) {
      stroge.childY = geomCtx.ctxPlane[1][7];
    } else {
      stroge.childY = update_bit[5];
    }
    if (deleteZ) {
      stroge.childZ = geomCtx.ctxPlane[2][7];
    } else {
      stroge.childZ = update_bit[6];
    }

    if (deleteX && deleteY && deleteZ) {
      stroge.leanX = geomCtx.ctxPlane[5][7];
      stroge.leanY = geomCtx.ctxPlane[4][7];
      stroge.leanZ = geomCtx.ctxPlane[3][7];
      stroge.Dot = geomCtx.ctxPlane[6][7];
    }
    if (deleteX && deleteY && (!deleteZ)) {
      stroge.leanX = geomCtx.ctxPlane[1][6];
      stroge.leanY = geomCtx.ctxPlane[0][6];
      stroge.leanZ = geomCtx.ctxPlane[3][7];
      stroge.Dot = geomCtx.ctxPlane[3][6];
    }
    if (deleteX && deleteZ && (!deleteY)) {
      stroge.leanX = geomCtx.ctxPlane[1][5];
      stroge.leanY = geomCtx.ctxPlane[4][7];
      stroge.Dot = geomCtx.ctxPlane[4][5];
      stroge.leanZ = geomCtx.ctxPlane[0][5];
    }
    if ((!deleteX) && deleteY && deleteZ) {
      stroge.leanX = geomCtx.ctxPlane[5][7];
      stroge.Dot = geomCtx.ctxPlane[5][3];
      stroge.leanY = geomCtx.ctxPlane[2][3];
      stroge.leanZ = geomCtx.ctxPlane[0][3];
    }
    if (deleteX && (!deleteY) && (!deleteZ)) {
      stroge.leanX = update_bit[4];
      stroge.leanY = geomCtx.ctxPlane[0][6];
      stroge.leanZ = geomCtx.ctxPlane[0][5];
      stroge.Dot = geomCtx.ctxPlane[0][4];
    }
    if (deleteY && (!deleteX) && (!deleteZ)) {
      stroge.leanX = geomCtx.ctxPlane[1][6];
      stroge.leanZ = geomCtx.ctxPlane[1][3];
      stroge.Dot = geomCtx.ctxPlane[1][2];
      stroge.leanY = update_bit[2];
    }
    if (deleteZ && (!deleteY) && (!deleteX)) {
      stroge.leanX = geomCtx.ctxPlane[2][5];
      stroge.leanY = geomCtx.ctxPlane[2][3];
      stroge.Dot = geomCtx.ctxPlane[2][1];
      stroge.leanZ = update_bit[1];
    }
    if ((!deleteX) && (!deleteY) && (!deleteZ)) {
      stroge.leanX = update_bit[4];
      stroge.leanY = update_bit[2];
      stroge.leanZ = update_bit[1];
      stroge.Dot = update_bit[0];
    }
    break;
  }

  return stroge;
}
