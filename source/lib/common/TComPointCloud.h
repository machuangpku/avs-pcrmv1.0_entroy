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

#include "HighLevelSyntax.h"
#include "PointCloudMortonTable.h"
#include "TComMath.h"
#include "TComVector.h"
#include "contributors.h"
#include <string>
#include <vector>

using namespace std;

///< \in TLibCommon \{

typedef V3<Double> PC_POS;
typedef V3<UInt8> PC_COL;
typedef Int16 PC_REFL;
typedef V3<Double> PC_VC3;
typedef V3<Int64> PC_POS_INT;

const uint8_t maxNeighborsNum = 6;

/**
 * Class TComPointCloud
 * point cloud class
 */

class TComPointCloud {
  friend class AttributeReader;

private:
  vector<PC_POS> m_pos;    ///< geometry position in (x, y, z)
  vector<PC_COL> m_color;  ///< 8bit color in (r, g, b)
  vector<PC_REFL> m_refl;  ///< 16bit reflectance
  TSize m_numPoint;        ///< total number of points

public:
  TComPointCloud() = default;
  ~TComPointCloud() = default;

  PC_POS& operator[](const TSize i);
  const PC_POS& operator[](const TSize i) const;
  const TSize& getNumPoint() const;
  Void setNumPoint(TSize numPoint);

  const vector<PC_POS>& positions() const;
  vector<PC_POS>& positions();
  PC_COL getColor(const TSize index) const;
  PC_COL& getColor(const TSize index);
  const vector<PC_COL>& getColors() const;
  vector<PC_COL>& getColors();
  Void setColor(const TSize index, const V3<UInt8> color);
  Void setColors(vector<PC_COL> colors);
  PC_REFL getReflectance(const TSize index) const;
  PC_REFL& getReflectance(const TSize index);
  vector<PC_REFL>& getReflectances();
  Void setReflectance(const TSize index, const UInt16 reflectance);

  Bool hasReflectances() const;
  Void addReflectances();
  Void removeReflectances();

  bool hasColors() const;
  void addColors();
  void removeColors();

  Void init(const size_t size, bool WithColor, bool WithRef);
  Void convertRGBToYUV();
  Void convertYUVToRGB();

  Bool readFromFile(const string& fileName, const Bool geomOnly = false);
  Bool writeToFile(const string& fileName, const Bool isAscii = true) const;
  Void clear();
  Void swapPoints(const TSize& idx1, const TSize& idx2);

  Void computeBoundingBox(PC_POS& boxMin, PC_POS& boxMax) const;

};  ///< END CLASS TComPointCloud

template<typename T> class TComRingBuffer {
public:
  TComRingBuffer(std::size_t _capacity)
    : capacity(_capacity)
    , buffer(_capacity) {}

  bool empty() const {
    return size == 0;
  }

  size_t getSize() const {
    return size;
  }

  bool full() const {
    return size == capacity;
  }

  T& operator[](std::size_t pos) {
    auto p = (read_idx + pos) % capacity;
    return buffer[p];
  }

  //if there is no duplicate data in the buffer, insert the buffer at the end
  //if the buffer reach the capacity, remove the data at the beginning
  void insertion(T data, bool allowDuplicate = true) {
    //chck if there is duplicate data in the buffer
    if (!allowDuplicate) {
      for (int i = 0; i < size; i++) {
        auto p = (read_idx + i) % capacity;
        if (data.second == buffer[p].second) {
          std::swap(buffer[p], buffer[read_idx]);
          return;
        }
      }
    }

    //insert the data if not in the ring buffer
    if (write_idx >= capacity) {
      write_idx = 0;
    }
    buffer[write_idx++] = data;
    if (full()) {
      read_idx = (read_idx + 1) % capacity;
    } else {
      size++;
    }
  }

private:
  std::vector<T> buffer;
  std::size_t read_idx = 0;
  std::size_t write_idx = 0;
  std::size_t size = 0;
  std::size_t capacity = 0;
};

/**
 * Class AttributeReader
 * implement the utility of reading attribute data to TComPointCloud
 */
class AttributeReader {
public:
  enum DataType {
    DT_FLOAT64,
    DT_FLOAT32,
    DT_INT64,
    DT_INT32,
    DT_INT16,
    DT_INT8,
    DT_UINT64,
    DT_UINT32,
    DT_UINT16,
    DT_UINT8,
  };
  enum AttributeType {
    AT_UNDEF = 0,
    AT_POS_X = 1 << 0,
    AT_POS_Y = 1 << 1,
    AT_POS_Z = 1 << 2,
    AT_COL_R = 1 << 3,
    AT_COL_G = 1 << 4,
    AT_COL_B = 1 << 5,
    AT_REFLE = 1 << 6,
    AT_NOR_X = 1 << 7,
    AT_NOR_Y = 1 << 8,
    AT_NOR_Z = 1 << 9,
    AT_POS = 0b0000000111,
    AT_COL = 0b0000111000,
    AT_NOR = 0b1110000000,
  };

private:
  AttributeType m_attrType;
  Void* m_target;
  Int m_dataTypeTarget;
  UInt8 m_bytesTarget;
  Int m_dataTypeSource;
  UInt8 m_bytesSource;
  TComPointCloud* m_pc;

public:
  AttributeReader(TComPointCloud* pc)
    : m_pc(pc)
    , m_target(nullptr) {}

  AttributeType setAttrType(const string& attrName) {
    if (attrName == "x")
      m_attrType = AttributeType::AT_POS_X;
    else if (attrName == "y")
      m_attrType = AttributeType::AT_POS_Y;
    else if (attrName == "z")
      m_attrType = AttributeType::AT_POS_Z;
    else if (attrName == "red" || attrName == "r")
      m_attrType = AttributeType::AT_COL_R;
    else if (attrName == "green" || attrName == "g")
      m_attrType = AttributeType::AT_COL_G;
    else if (attrName == "blue" || attrName == "b")
      m_attrType = AttributeType::AT_COL_B;
    else if (attrName == "reflectance" || attrName == "refl" || attrName == "refc")
      m_attrType = AttributeType::AT_REFLE;
    else
      m_attrType = AttributeType::AT_UNDEF;
    return m_attrType;
  }

  Void setTarget() {
    switch (m_attrType) {
    case AttributeReader::AT_POS_X:
      m_target = m_pc->m_pos.size() > 0 ? &((*m_pc)[0][0]) : nullptr;
      break;
    case AttributeReader::AT_POS_Y:
      m_target = m_pc->m_pos.size() > 0 ? &((*m_pc)[0][1]) : nullptr;
      break;
    case AttributeReader::AT_POS_Z:
      m_target = m_pc->m_pos.size() > 0 ? &((*m_pc)[0][2]) : nullptr;
      break;
    case AttributeReader::AT_COL_R:
      m_target = m_pc->m_color.size() > 0 ? &(m_pc->m_color[0][0]) : nullptr;
      break;
    case AttributeReader::AT_COL_G:
      m_target = m_pc->m_color.size() > 0 ? &(m_pc->m_color[0][1]) : nullptr;
      break;
    case AttributeReader::AT_COL_B:
      m_target = m_pc->m_color.size() > 0 ? &(m_pc->m_color[0][2]) : nullptr;
      break;
    case AttributeReader::AT_REFLE:
      m_target = m_pc->m_refl.size() > 0 ? &(m_pc->m_refl[0]) : nullptr;
      break;
    case AttributeReader::AT_UNDEF:
    default:
      m_target = nullptr;  ///< unsupported type
      break;
    }
  }

  template<typename T> Void setDataToTarget(const T& data) {
    if (m_target == nullptr)
      return;

    switch (m_dataTypeTarget)  ///< target can only be double, uint16 or uint8 for now.
    {
    case DT_FLOAT64: {
      *(Double*)(m_target) = (Double)data;
      break;
    }
    case DT_INT16: {
      *(Int16*)(m_target) = (Int16)data;
      break;
    }
    case DT_UINT16: {
      *(UInt16*)(m_target) = (UInt16)data;
      break;
    }
    case DT_UINT8: {
      *(UInt8*)(m_target) = (UInt8)data;
      break;
    }
    default:
      assert(0);
      break;
    }
  }

  UInt setTargetDataType(const string& attrName) {
    if (attrName == "x" || attrName == "y" || attrName == "z") {
      m_bytesTarget = sizeof(PC_POS);
      m_dataTypeTarget = DT_FLOAT64;
    } else if (attrName == "red" || attrName == "green" || attrName == "blue" || attrName == "r" ||
               attrName == "g" || attrName == "b") {
      m_bytesTarget = sizeof(PC_COL);
      m_dataTypeTarget = DT_UINT8;
    } else if (attrName == "reflectance" || attrName == "refl" || attrName == "refc") {
      m_bytesTarget = sizeof(PC_REFL);
      m_dataTypeTarget = DT_INT16;
    }
    return m_bytesTarget;
  }

  UInt setSourceDataType(const string& attrType) {
    if (attrType == "float32" || attrType == "float") {
      m_dataTypeSource = DataType::DT_FLOAT32;
      m_bytesSource = 4;
    } else if (attrType == "float64" || attrType == "double") {
      m_dataTypeSource = DataType::DT_FLOAT64;
      m_bytesSource = 8;
    } else if (attrType == "uint64" || attrType == "ulong") {
      m_dataTypeSource = DataType::DT_UINT64;
      m_bytesSource = 8;
    } else if (attrType == "uint32" || attrType == "uint") {
      m_dataTypeSource = DataType::DT_UINT32;
      m_bytesSource = 4;
    } else if (attrType == "uint16" || attrType == "ushort") {
      m_dataTypeSource = DataType::DT_UINT16;
      m_bytesSource = 2;
    } else if (attrType == "uint8" || attrType == "uchar") {
      m_dataTypeSource = DataType::DT_UINT8;
      m_bytesSource = 1;
    } else if (attrType == "int64" || attrType == "long") {
      m_dataTypeSource = DataType::DT_INT64;
      m_bytesSource = 8;
    } else if (attrType == "int32" || attrType == "int") {
      m_dataTypeSource = DataType::DT_INT32;
      m_bytesSource = 4;
    } else if (attrType == "int16" || attrType == "short") {
      m_dataTypeSource = DataType::DT_INT16;
      m_bytesSource = 2;
    } else if (attrType == "int8" || attrType == "char") {
      m_dataTypeSource = DataType::DT_INT8;
      m_bytesSource = 1;
    }
    return m_bytesSource;
  }

  Void readFromAscii(const string& token) {
    if (!m_target)  ///skip the empty attribute
      return;
    switch (m_dataTypeTarget) {
    case DT_FLOAT64:
    case DT_FLOAT32: {
      Double t = atof(token.c_str());
      setDataToTarget(t);
      break;
    }
    case DT_UINT64:
    case DT_UINT32:
    case DT_UINT16:
    case DT_UINT8: {
      UInt64 t = atoi(token.c_str());
      setDataToTarget(t);
      break;
    }
    case DT_INT64:
    case DT_INT32:
    case DT_INT16:
    case DT_INT8: {
      Int64 t = atoi(token.c_str());
      setDataToTarget(t);
      break;
    }
    default:
      assert(false);
      break;
    }
    m_target = (TChar*)m_target + m_bytesTarget;  ///< pointer move forward to next
  }

  Void readFromBinary(istream& plyFile) {
    switch (m_dataTypeSource) {
    case DT_FLOAT64: {
      Double t;
      plyFile.read((TChar*)(&t), m_bytesSource);
      setDataToTarget(t);
      break;
    }
    case DT_FLOAT32: {
      Float t;
      plyFile.read((TChar*)(&t), m_bytesSource);
      setDataToTarget(t);
      break;
    }
    case DT_INT64: {
      Int64 t;
      plyFile.read((TChar*)(&t), m_bytesSource);
      setDataToTarget(t);
      break;
    }
    case DT_INT32: {
      Int32 t;
      plyFile.read((TChar*)(&t), m_bytesSource);
      setDataToTarget(t);
      break;
    }
    case DT_INT16: {
      Int16 t;
      plyFile.read((TChar*)(&t), m_bytesSource);
      setDataToTarget(t);
      break;
    }
    case DT_INT8: {
      Int8 t;
      plyFile.read((TChar*)(&t), m_bytesSource);
      setDataToTarget(t);
      break;
    }
    case DT_UINT64: {
      UInt64 t;
      plyFile.read((TChar*)(&t), m_bytesSource);
      setDataToTarget(t);
      break;
    }
    case DT_UINT32: {
      UInt32 t;
      plyFile.read((TChar*)(&t), m_bytesSource);
      setDataToTarget(t);
      break;
    }
    case DT_UINT16: {
      UInt16 t;
      plyFile.read((TChar*)(&t), m_bytesSource);
      setDataToTarget(t);
      break;
    }
    case DT_UINT8: {
      UInt8 t;
      plyFile.read((TChar*)(&t), m_bytesSource);
      setDataToTarget(t);
      break;
    }
    default:
      assert(false);
      break;
    }
    if (m_target != nullptr)
      m_target = (TChar*)m_target + m_bytesTarget;  ///< pointer move forward to next
  }
};

//////////////////////////////////////////////////////////////////////////
// Inline functions
//////////////////////////////////////////////////////////////////////////

///< get tokens from string by splitting with delimiter
static inline Void getTokens(string& str, vector<string>& tokens) {
  tokens.clear();

  const string delimiter = " \n\r\t\0";
  str += ' ';  ///< add a delimiter at the end
  string token = "";
  for (const auto& ch : str) {
    if (delimiter.find(ch) != string::npos) {
      if (token.length() > 0)
        tokens.push_back(token);
      token = "";
    } else
      token += ch;
  }
}

///< check the system endianness, return true: little endian or false: big endian
static inline Bool isLittleEndian() {
  UInt i = 1;
  return (*(reinterpret_cast<char*>(&i)) == 1);
}

///< Determine color values from a reference point cloud.
bool TComColorTransfer(const TComPointCloud& pointCloudOrg, double geomQuanStep, PC_VC3 quanOffset,
                       TComPointCloud& pointCloudRec);

///< Determine reflectance values from a reference point cloud.
bool TComReflectanceTransfer(const TComPointCloud& pointCloudOrg, double geomQuanStep,
                             PC_VC3 quanOffset, TComPointCloud& pointCloudRec);

///< Recolour attributes based on a source/reference point cloud.
int recolour(const TComPointCloud& pointCloudOrg, float geomQuanStep, V3<int> quanOffset,
             TComPointCloud* pointCloudRec);

inline UInt64 QuantizaResidual(const UInt64 residual, const UInt8 QP) {
  const int index = QP;
  const int offset = 1 << (encoderShiftBit - 1);
  const uint32_t quantstepSize = DriveQuantstepSize[index];
  return (residual * quantstepSize + offset) >> encoderShiftBit;
}

inline UInt64 InverseQuantizeResidual(const UInt64 residual, const UInt8 QP) {
  const int index = QP;
  const int offset = 1 << (decoderShiftBit - 1);
  uint32_t quantstepSize = DriveInerseQuantstepSize[index];
  return (residual * quantstepSize + offset) >> decoderShiftBit;
}

inline uint64_t IntToUInt(int64_t value) {
  return (value < 0) ? static_cast<uint64_t>(-1 - (2 * value)) : static_cast<uint64_t>(2 * value);
}

inline int64_t UIntToInt(uint64_t uiValue) {
  return (uiValue & 1) ? -(static_cast<int64_t>((uiValue + 1) >> 1))
                       : (static_cast<int64_t>(uiValue >> 1));
}

inline UInt64 mortonAddr(const Int32 x, const Int32 y, const Int32 z) {
  assert(x >= 0 && y >= 0 && z >= 0);
  int64_t answer = pointCloudMortonCode256X[(x >> 16) & 0xFF] |
    pointCloudMortonCode256Y[(y >> 16) & 0xFF] | pointCloudMortonCode256Z[(z >> 16) & 0xFF];
  answer = answer << 24 | pointCloudMortonCode256X[(x >> 8) & 0xFF] |
    pointCloudMortonCode256Y[(y >> 8) & 0xFF] | pointCloudMortonCode256Z[(z >> 8) & 0xFF];
  answer = answer << 24 | pointCloudMortonCode256X[x & 0xFF] | pointCloudMortonCode256Y[y & 0xFF] |
    pointCloudMortonCode256Z[z & 0xFF];
  return answer;
}

inline void HilbertAddr(const uint32_t& x, const uint32_t& y, const uint32_t& z,
                        int64_t& hilbertCode) {
  static uint64_t pos;
  static uint64_t state;
  state = 4;
  hilbertCode = 0;

  hilbertCode <<= 6;
  pos = ((x & 0xC0000) >> 14) | ((y & 0xC0000) >> 16) | ((z & 0xC0000) >> 18);
  hilbertCode += HilbertTable[state][pos][1];
  state = HilbertTable[state][pos][0];

  hilbertCode <<= 6;
  pos = ((x & 0x30000) >> 12) | ((y & 0x30000) >> 14) | ((z & 0x30000) >> 16);
  hilbertCode += HilbertTable[state][pos][1];
  state = HilbertTable[state][pos][0];

  hilbertCode <<= 6;
  pos = ((x & 0xC000) >> 10) | ((y & 0xC000) >> 12) | ((z & 0xC000) >> 14);
  hilbertCode += HilbertTable[state][pos][1];
  state = HilbertTable[state][pos][0];

  hilbertCode <<= 6;
  pos = ((x & 0x3000) >> 8) | ((y & 0x3000) >> 10) | ((z & 0x3000) >> 12);
  hilbertCode += HilbertTable[state][pos][1];
  state = HilbertTable[state][pos][0];

  hilbertCode <<= 6;
  pos = ((x & 0xC00) >> 6) | ((y & 0xC00) >> 8) | ((z & 0xC00) >> 10);
  hilbertCode += HilbertTable[state][pos][1];
  state = HilbertTable[state][pos][0];

  hilbertCode <<= 6;
  pos = ((x & 0x300) >> 4) | ((y & 0x300) >> 6) | ((z & 0x300) >> 8);
  hilbertCode += HilbertTable[state][pos][1];
  state = HilbertTable[state][pos][0];

  hilbertCode <<= 6;
  pos = ((x & 0xC0) >> 2) | ((y & 0xC0) >> 4) | ((z & 0xC0) >> 6);
  hilbertCode += HilbertTable[state][pos][1];
  state = HilbertTable[state][pos][0];

  hilbertCode <<= 6;
  pos = (x & 0x30) | ((y & 0x30) >> 2) | ((z & 0x30) >> 4);
  hilbertCode += HilbertTable[state][pos][1];
  state = HilbertTable[state][pos][0];

  hilbertCode <<= 6;
  pos = ((x & 0xC) << 2) | (y & 0xC) | ((z & 0xC) >> 2);
  hilbertCode += HilbertTable[state][pos][1];
  state = HilbertTable[state][pos][0];

  hilbertCode <<= 6;
  pos = ((x & 0x3) << 4) | ((y & 0x3) << 2) | (z & 0x3);
  hilbertCode += HilbertTable[state][pos][1];
  state = HilbertTable[state][pos][0];
}

inline UInt32 mortonAddr(const Int8 x, const Int8 y, const Int8 z) {
  assert(x >= 0 && x <= 255 && y >= 0 && y <= 255 && z >= 0 && z <= 255);
  return pointCloudMortonCode256X[x] | pointCloudMortonCode256X[y] | pointCloudMortonCode256Z[z];
}

inline uint64_t morton3dAdd(uint64_t a, uint64_t b) {
  uint64_t mask = 0x9249249249249249llu;
  uint64_t val = 0;

  for (int i = 0; i < 3; i++) {
    val |= (a | ~mask) + (b & mask) & mask;
    mask <<= 1;
  }

  return val;
}

struct pointMortonCodeWithIndex {
  int64_t mortonCode;
  int32_t index;
  bool operator<(const pointMortonCodeWithIndex& rhs) const {
    // NB: index used to maintain stable sort
    if (mortonCode == rhs.mortonCode)
      return index < rhs.index;
    return mortonCode < rhs.mortonCode;
  }
  bool operator==(const int64_t& pointMorton) const {
    // NB: index used to maintain stable sort
    return this->mortonCode == pointMorton;
  }
  bool operator<(const int64_t& pointMorton) const {
    // NB: index used to maintain stable sort
    return this->mortonCode < pointMorton;
  }
};

class PredictedNeighborInfo {
public:
  PredictedNeighborInfo(const uint32_t& pointIndex = 0, const uint64_t& w = 0)
    : predictorIndex(pointIndex)
    , weight(w) {}
  uint64_t weight;
  uint32_t predictorIndex;
  bool operator<(const PredictedNeighborInfo& rhs) const {
    return (weight == rhs.weight) ? predictorIndex < rhs.predictorIndex : weight < rhs.weight;
  }
  bool operator==(const PredictedNeighborInfo& rhs) const {
    return (weight == rhs.weight && predictorIndex == rhs.predictorIndex);
  }
  bool operator==(const uint32_t& index) const {
    return predictorIndex == index;
  }
};
class PredictedNearsetNeighbors {
public:
  void init(const int& neighborNum) {
    neighborsCount = 0;
    maxNeighborCount = neighborNum;
    preditedNeighbors = new PredictedNeighborInfo[maxNeighborCount];
    memset(preditedNeighbors, 0, sizeof(PredictedNeighborInfo) * neighborNum);
  }
  void insertNeighor(const uint32_t predictIndex, const uint64_t weight,
                     const bool needSort = true) {
    if (needSort) {
      bool sort = false;

      if (neighborsCount < maxNeighborCount) {
        PredictedNeighborInfo& neighborInfo = preditedNeighbors[neighborsCount];
        neighborInfo.weight = weight;
        neighborInfo.predictorIndex = predictIndex;
        sort = true;
      } else {
        PredictedNeighborInfo& neighborInfo = preditedNeighbors[maxNeighborCount - 1];
        if (weight < neighborInfo.weight ||
            (weight == neighborInfo.weight && predictIndex < neighborInfo.predictorIndex)) {
          neighborInfo.weight = weight;
          neighborInfo.predictorIndex = predictIndex;
          sort = true;
        }
      }
      for (int32_t k = neighborsCount - 1; k > 0 && sort; --k) {
        if (preditedNeighbors[k] < preditedNeighbors[k - 1])
          std::swap(preditedNeighbors[k], preditedNeighbors[k - 1]);
        else
          break;
      }
      if (neighborsCount < maxNeighborCount)
        ++neighborsCount;
    } else {
      PredictedNeighborInfo& neighborInfo = preditedNeighbors[neighborsCount];
      neighborInfo.weight = weight;
      neighborInfo.predictorIndex = predictIndex;
      ++neighborsCount;
    }
  }
  PredictedNeighborInfo* begin() const {
    return preditedNeighbors;
  }
  PredictedNeighborInfo* end() const {
    return preditedNeighbors + neighborsCount;
  }
  PredictedNeighborInfo* getNeigbors() const {
    return preditedNeighbors;
  }
  uint32_t getNeighCount() const {
    return neighborsCount;
  }
  ~PredictedNearsetNeighbors() {
    delete[] preditedNeighbors;
  }

private:
  uint32_t neighborsCount;
  uint32_t maxNeighborCount;
  PredictedNeighborInfo* preditedNeighbors;
};
void sortReconPoints(const TComPointCloud& pointCloudRecon, UInt lastNumReconPoints,
                     UInt currNumReconPoints, std::vector<pointMortonCodeWithIndex>& mortonOrder);

void sortReconPointsShift(const TComPointCloud& pointCloudRecon, const Int shift,
                          UInt lastNumReconPoints, UInt currNumReconPoints,
                          std::vector<pointMortonCodeWithIndex>& mortonOrder,
                          std::vector<int>& mortonToindex, std::vector<int>& mortonTomorton2);
int32_t determineInitShiftBits(const uint32_t voxelCount,
                               const std::vector<pointMortonCodeWithIndex>& pointCloudMorton);
Void levelOfDetailLayeringStructure(const AttributeParameterSet aps,
                                    const TComPointCloud& pointCloudOrg,
                                    std::vector<uint32_t>& layerToindex,
                                    std::vector<uint32_t>& numberOfPointsPerLayer,
                                    std::vector<pointMortonCodeWithIndex>& pointCloudMorton);

///< \}
