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

#ifndef _PCEM_KDTREE_H_
#define _PCEM_KDTREE_H_

#include "common/TComPointCloud.h"
#include "nanoflann.hpp"
#include <array>
#include <vector>

namespace pc_evalue {
typedef uint32_t index_type;
typedef Double value_type;
typedef PC_POS point_type;
typedef std::vector<point_type> pointset_type;
typedef double distance_type;

class NNResult {
public:
  NNResult() {
    m_size = 0;
    m_capacity = 0;
  }
  ~NNResult() {
    m_indexes.clear();
    m_distances.clear();
  }
  inline void reserve(const index_type size) {
    m_capacity = size;
    m_indexes.reserve(size);
    m_distances.reserve(size);
    m_indexes.resize(size);
    m_distances.resize(size);
  }
  inline void resize(const index_type size) {
    m_size = size;
    m_indexes.resize(size);
    m_distances.resize(size);
  }
  inline index_type size() const {
    return m_size;
  }
  inline index_type& capacity() {
    return m_capacity;
  }
  inline index_type& indices(index_type index) {
    return m_indexes[index];
  }
  inline distance_type& dist(index_type index) {
    return m_distances[index];
  }
  inline index_type* indices() {
    return m_indexes.data();
  }
  inline distance_type* dist() {
    return m_distances.data();
  }

  index_type m_size;
  index_type m_capacity;
  std::vector<index_type> m_indexes;
  std::vector<distance_type> m_distances;
};

class KdTree {
public:
  typedef typename nanoflann::L2_Adaptor<value_type, KdTree, double>::DistanceType DistanceType;
  typedef nanoflann::KDTreeSingleIndexAdaptor<nanoflann::L2_Adaptor<value_type, KdTree, double>,
                                              KdTree, 3 /*DIM*/, index_type>
    index_t;

  KdTree(const int dim, const pointset_type& positions, const int maxLeafSize = 10)
    : m_positions(positions) {
    assert(m_positions.size() != 0 && dim == 3);
    m_indices = new index_t(dim, *this, nanoflann::KDTreeSingleIndexAdaptorParams(maxLeafSize));
    m_indices->buildIndex();
  }

  KdTree(const pointset_type& positions, const int maxLeafSize = 10)
    : m_positions(positions) {
    assert(m_positions.size() != 0);
    m_indices = new index_t(3, *this, nanoflann::KDTreeSingleIndexAdaptorParams(maxLeafSize));
    m_indices->buildIndex();
  }

  ~KdTree() {
    delete m_indices;
  }

  bool search(const value_type* queryPoint, const size_t NNNumber, index_type* NNIndices,
              DistanceType* NNDistances) const {
    nanoflann::KNNResultSet<DistanceType, index_type> resultSet(NNNumber);
    resultSet.init(NNIndices, NNDistances);
    return m_indices->findNeighbors(resultSet, queryPoint, nanoflann::SearchParams());
  }

  bool search(const point_type queryPoint, const index_type NNNumber, NNResult& result) const {
    result.reserve(NNNumber);
    nanoflann::KNNResultSet<DistanceType, index_type> resultSet(NNNumber);
    resultSet.init(result.indices(), result.dist());
    if (m_indices->findNeighbors(resultSet, &queryPoint[0], nanoflann::SearchParams())) {
      result.resize(index_type(resultSet.size()));
      return true;
    } else
      return false;
  }

  inline size_t kdtree_get_point_count() const {
    return m_positions.size();
  }

  inline value_type kdtree_get_pt(const size_t idx, int dim) const {
    return m_positions[idx][dim];
  }

  template<class BBOX> bool kdtree_get_bbox(BBOX&) const {
    return false;
  }

public:
  index_t* m_indices;
  const pointset_type& m_positions;
};
}  // namespace pc_evalue
#endif
