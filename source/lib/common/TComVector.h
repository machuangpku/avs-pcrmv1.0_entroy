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
#include "contributors.h"
#include <algorithm>
#include <ostream>

using namespace std;

///< \in TLibCommon \{

/**
 * Class TComVector
 * N-dimensional vectors of type T
 */

template<typename T, TSize N> class TComVector {
private:
  T m_data[N];

public:
  TSize getVectorDimension();
  T* getData();
  T min() const;
  T max() const;
  TComVector();

  TComVector(const T& v) {
    for (Int i = 0; i < N; i++)
      m_data[i] = v;
  }

  TComVector(const initializer_list<T>& list);

  ///< operator overwrite
public:
  T& operator[](TSize i) {
    assert(i < N);
    return m_data[i];
  }

  const T& operator[](TSize i) const {
    assert(i < N);
    return m_data[i];
  }

  T getNorm1() {
    T ret = 0;
    for (const auto& d : m_data)
      ret += abs(d);
    return ret;
  }

  T getNorm2() {
    T ret = 0;
    for (const auto& d : m_data)
      ret += d * d;
    return ret;
  }

  TComVector& operator=(const initializer_list<T>& list) {
    assert(N == list.size());
    Int i = 0;
    for (const auto& v : list)
      m_data[i++] = v;
    return *this;
  }

  TComVector& operator=(const TComVector& vec) {
    memcpy(m_data, vec.m_data, sizeof(TComVector<T, N>));
    return *this;
  }

  TComVector& operator=(const T& v) {
    for (auto& d : m_data)
      d = v;
    return *this;
  }

  Bool operator==(const TComVector& vec) const {
    for (TSize i = 0; i < N; i++)
      if (m_data[i] != vec[i])
        return false;
    return true;
  }

  Bool operator!=(const TComVector& vec) const {
    for (TSize i = 0; i < N; i++)
      if (m_data[i] != vec[i])
        return true;
    return false;
  }

  Bool operator<(const TComVector& vec) const {
    for (TSize i = 0; i < N; i++)
      if (m_data[i] != vec[i])
        return m_data[i] < vec[i];
    return false;
  }

  Bool operator>(const TComVector& vec) const {
    for (TSize i = 0; i < N; i++)
      if (m_data[i] != vec[i])
        return m_data[i] > vec[i];
    return false;
  }

  TComVector& operator/=(const T& v) {
    for (auto& d : m_data)
      d /= v;
    return *this;
  }

  TComVector& operator+=(const TComVector& vec) {
    for (Int i = 0; i < N; i++)
      m_data[i] += vec[i];
    return *this;
  }

  TComVector& operator-=(const TComVector& vec) {
    for (Int i = 0; i < N; i++)
      m_data[i] -= vec[i];
    return *this;
  }

  friend ostream& operator<<(ostream& os, const TComVector& v) {
    os << v[0] << " " << v[1] << " " << v[2];
    return os;
  }

  friend TComVector operator+(const TComVector& lhs, const TComVector& rhs) {
    TComVector<T, N> temp;
    for (Int i = 0; i < N; i++)
      temp.m_data[i] = lhs[i] + rhs[i];
    return temp;
  }

  friend TComVector operator+(const T lhs, const TComVector& rhs) {
    TComVector<T, N> temp;
    for (Int i = 0; i < N; i++)
      temp.m_data[i] = lhs + rhs[i];
    return temp;
  }

  friend TComVector operator+(const TComVector& lhs, const T rhs) {
    TComVector<T, N> temp;
    for (Int i = 0; i < N; i++)
      temp.m_data[i] = lhs[i] + rhs;
    return temp;
  }

  friend TComVector operator-(const TComVector& lhs, const TComVector& rhs) {
    TComVector<T, N> temp;
    for (Int i = 0; i < N; i++)
      temp.m_data[i] = lhs[i] - rhs[i];
    return temp;
  }

  friend TComVector operator-(const T lhs, const TComVector& rhs) {
    TComVector<T, N> temp;
    for (Int i = 0; i < N; i++)
      temp.m_data[i] = lhs - rhs[i];
    return temp;
  }

  friend TComVector operator-(const TComVector& lhs, const T rhs) {
    TComVector<T, N> temp;
    for (Int i = 0; i < N; i++)
      temp.m_data[i] = lhs[i] - rhs;
    return temp;
  }

  friend TComVector operator*(const TComVector& lhs, const TComVector& rhs) {
    TComVector<T, N> temp;
    for (Int i = 0; i < N; i++)
      temp.m_data[i] = lhs[i] * rhs[i];
    return temp;
  }

  friend TComVector operator*(const T lhs, const TComVector& rhs) {
    TComVector<T, N> temp;
    for (Int i = 0; i < N; i++)
      temp.m_data[i] = lhs * rhs[i];
    return temp;
  }

  friend TComVector operator*(const TComVector& lhs, const T rhs) {
    TComVector<T, N> temp;
    for (Int i = 0; i < N; i++)
      temp.m_data[i] = lhs[i] * rhs;
    return temp;
  }

  friend TComVector operator<<(const TComVector& lhs, const TComVector& rhs) {
    TComVector<T, N> temp;
    for (Int i = 0; i < N; i++)
      temp.m_data[i] = lhs[i] << rhs[i];
    return temp;
  }

  friend TComVector operator<<(const T lhs, const TComVector& rhs) {
    TComVector<T, N> temp;
    for (Int i = 0; i < N; i++)
      temp.m_data[i] = lhs << rhs[i];
    return temp;
  }

  friend TComVector operator<<(const TComVector& lhs, const T rhs) {
    TComVector<T, N> temp;
    for (Int i = 0; i < N; i++)
      temp.m_data[i] = lhs[i] << rhs;
    return temp;
  }

  friend TComVector operator>>(const TComVector& lhs, const TComVector& rhs) {
    TComVector<T, N> temp;
    for (Int i = 0; i < N; i++)
      temp.m_data[i] = lhs[i] >> rhs[i];
    return temp;
  }

  friend TComVector operator>>(const T lhs, const TComVector& rhs) {
    TComVector<T, N> temp;
    for (Int i = 0; i < N; i++)
      temp.m_data[i] = lhs >> rhs[i];
    return temp;
  }

  friend TComVector operator>>(const TComVector& lhs, const T rhs) {
    TComVector<T, N> temp;
    for (Int i = 0; i < N; i++)
      temp.m_data[i] = lhs[i] >> rhs;
    return temp;
  }

};  ///< END CLASS TComVector

template<class T> using V3 = TComVector<T, 3>;

//////////////////////////////////////////////////////////////////////////
// Public class functions
//////////////////////////////////////////////////////////////////////////

template<typename T, TSize N> TSize TComVector<T, N>::getVectorDimension() {
  return N;
}

template<typename T, TSize N> T* TComVector<T, N>::getData() {
  return m_data;
}

template<typename T, TSize N> T TComVector<T, N>::min() const {
  assert(N > 0);
  return *std::min_element(m_data, m_data + N);
}

template<typename T, TSize N> T TComVector<T, N>::max() const {
  assert(N > 0);
  return *std::max_element(m_data, m_data + N);
}

template<typename T, TSize N> TComVector<T, N>::TComVector() {
  *this = 0;
}

template<typename T, TSize N> TComVector<T, N>::TComVector(const initializer_list<T>& list) {
  *this = list;
}

///< \}
