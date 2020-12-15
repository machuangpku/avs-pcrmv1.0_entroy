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

#include "TComOctree.h"
#include "TComPointCloud.h"
#include "TComVector.h"
#include "TypeDef.h"
#include "rhHash.h"
#include <tuple>

using namespace std;

///< \in TLibCommon \{

class IdenticalHasher {
public:
  UInt64 operator()(UInt64 const& key) const {
    return key;
  }
};

/*
 Hash Map with Morton code as key
*/
template<typename T> class TComHashMap {
protected:
  robin_hood::unordered_map<UInt64, T, IdenticalHasher> m_map;

public:
  TComHashMap<T>() = default;

  ~TComHashMap<T>() {
    m_map.clear();
  }

  size_t getSize() const {
    return m_map.size();
  }

  bool get(const UInt64 morton, T& val) const {
    auto it = m_map.find(morton);
    if (it != m_map.end()) {
      val = it->second;
      return true;
    } else {
      return false;
    }
  }

  bool get(const Int32 x, const Int32 y, const Int32 z, T& val) const {
    assert(x >= 0 && y >= 0 && z >= 0);
    return get(mortonAddr(x, y, z), val);
  }

  T& getR(const Int32 x, const Int32 y, const Int32 z) {
    // get an element by key, if it's not existed, create a new one and return it
    assert(x >= 0 && y >= 0 && z >= 0);
    const auto key = mortonAddr(x, y, z);
    const auto& it = m_map.find(key);
    if (it == m_map.end()) {  ///< doesn't exist in the table
      return (m_map.emplace(std::piecewise_construct, std::forward_as_tuple(key),
                            std::forward_as_tuple(0)))
        .first->second;
    }
    return it->second;
  }

  Void insert(const Int32 x, const Int32 y, const Int32 z, const T val = 0) {
    assert(x >= 0 && y >= 0 && z >= 0);
    const auto key = mortonAddr(x, y, z);
    m_map.emplace(std::piecewise_construct, std::forward_as_tuple(key), std::forward_as_tuple(val));
  }

  Void insert(const UInt64 morton, const T val = 0) {
    m_map.emplace(std::piecewise_construct, std::forward_as_tuple(morton),
                  std::forward_as_tuple(val));
  }

};  ///< END CLASS TComHashMap

/*
 Dynamic Hash Map with Morton code as key
 Shrinking itself when reaching the capacity
*/
template<typename T> class TComHashMapDynamic : public TComHashMap<std::pair<T, bool>> {
private:
  const int m_boundarySizeLog2;
  const int m_max_size_to_shrink;
  V3<UInt> m_nextBoundaryCheckPoint;

public:
  TComHashMapDynamic()
    : m_boundarySizeLog2(8)
    , m_max_size_to_shrink(1 << 19)
    , m_nextBoundaryCheckPoint(0) {}

  TComHashMapDynamic(const int boundarySizeLog2)
    : m_boundarySizeLog2(std::max(boundarySizeLog2, 8))
    , m_max_size_to_shrink(1 << 19)
    , m_nextBoundaryCheckPoint(0) {}

  Bool isAtBoundaryCheckPoint(const V3<UInt>& nodePos) {
    return (nodePos >> m_boundarySizeLog2) != m_nextBoundaryCheckPoint;
  }

  Void setBoundaryCheckPoint(const V3<UInt>& nodePos) {
    m_nextBoundaryCheckPoint = nodePos >> m_boundarySizeLog2;
  }

  Void shrinkSize() {
    if (this->m_map.size() <= m_max_size_to_shrink) {
      // don't shrink until reaching the max size
      return;
    }
    for (auto it = this->m_map.begin(); it != this->m_map.end();) {
      if (!it->second.second) {
        it = this->m_map.erase(it);
      } else {
        it++;
      }
    }
  }

  Void insert(const Int32 x, const Int32 y, const Int32 z, const T val) {
    assert(x >= 0 && y >= 0 && z >= 0);
    const auto key = mortonAddr(x, y, z);
    const int boundaryCheckMask = (1 << m_boundarySizeLog2) - 1;
    // mark each element whether it is on a boundary
    bool isBoundary = (x & boundaryCheckMask) == boundaryCheckMask ||
      (y & boundaryCheckMask) == boundaryCheckMask || (z & boundaryCheckMask) == boundaryCheckMask;
    this->m_map.emplace(std::piecewise_construct, std::forward_as_tuple(key),
                        std::forward_as_tuple(val, isBoundary));
  }

  bool get(const UInt64 morton, T& val) const {
    auto it = this->m_map.find(morton);
    if (it != this->m_map.end()) {
      val = it->second.first;
      return true;
    } else {
      return false;
    }
  }

  bool get(const Int32 x, const Int32 y, const Int32 z, T& val) const {
    assert(x >= 0 && y >= 0 && z >= 0);
    return get(mortonAddr(x, y, z), val);
  }
};  ///< END CLASS TComHashMapDynamic
