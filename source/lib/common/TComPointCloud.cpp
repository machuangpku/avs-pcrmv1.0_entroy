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

#include "TComPointCloud.h"
#include "HighLevelSyntax.h"
#include "TComKdTree.h"
#include "contributors.h"
#include <algorithm>
#include <fstream>
#include <iomanip>
#include <limits>

///< \in TLibCommon \{

/**
 * Class TComPointCloud
 * point cloud class
 */

//////////////////////////////////////////////////////////////////////////
// Public class functions
//////////////////////////////////////////////////////////////////////////

Void TComPointCloud::clear() {
  m_pos.clear();
  m_color.clear();
  m_refl.clear();
  m_numPoint = 0;
}

Void TComPointCloud::swapPoints(const TSize& idx1, const TSize& idx2) {
  assert(idx1 < m_numPoint && idx2 < m_numPoint);
  if (idx1 == idx2)
    return;
  swap(m_pos[idx1], m_pos[idx2]);
  if (hasColors())
    swap(m_color[idx1], m_color[idx2]);
  if (hasReflectances())
    swap(m_refl[idx1], m_refl[idx2]);
}

Bool TComPointCloud::readFromFile(const string& fileName, const Bool geomOnly) {
  this->clear();  ///< clear the point cloud data

  ifstream plyFile(fileName, ios::in | ios::binary);

  if (!checkCond(plyFile.is_open(), "Error: failed to read ply file!"))
    return false;

  string line;
  vector<string> tokens;
  Bool isStart = false;
  Bool isVertex = false;
  Bool isAscii = true;
  vector<AttributeReader> attributes;
  Int attrType = 0;

  ///< read ply header
  while (true) {
    if (!checkCond(!plyFile.eof(), "Error: invalid ply header!"))
      return false;

    getline(plyFile, line);
    getTokens(line, tokens);

    if (tokens.empty() || tokens[0] == "comment")  ///< empty line or comment line
      continue;
    if (!isStart) {
      if (!checkCond(tokens[0] == "ply", "Error: invalid ply header!"))
        return false;
      isStart = true;
    } else if (tokens[0] == "format") {
      if (!checkCond(tokens.size() == 3, "Error: invalid ply format!"))
        return false;
      if (tokens[1] == "ascii")
        isAscii = true;
      else if (tokens[1].find("binary") != string::npos)
        isAscii = false;
      else
        return checkCond(false, "Error: invalid ply format!");
    } else if (tokens[0] == "element") {
      if (!checkCond(tokens.size() == 3, "Error: invalid ply element!"))
        return false;
      if (tokens[1] == "vertex") {
        m_numPoint = atoi(tokens[2].data());
        assert(m_numPoint > 0);
        isVertex = true;
      } else if (tokens[1] == "face")
        isVertex = false;
      else
        return checkCond(false, "Error: invalid ply element!");
    } else if (tokens[0] == "property" && isVertex) {
      if (!checkCond(tokens.size() == 3, "Error: invalid ply property!"))
        return false;
      string propType = tokens[1];
      string propName = tokens[2];

      AttributeReader attr(this);
      attr.setSourceDataType(propType);
      attr.setTargetDataType(propName);
      attrType |= attr.setAttrType(propName);
      attributes.push_back(attr);
    } else if (tokens[0] == "end_header") {
      break;  ///< finish reading ply header
    }
  }  ///< end while

  ///< prepare attribute memories
  if ((attrType & AttributeReader::AttributeType::AT_POS) ==
      AttributeReader::AttributeType::AT_POS) {
    m_pos.resize(m_numPoint);
  } else {
    return checkCond(false, "Error: position is incomplete!");
  }

  if (geomOnly)
    attrType = AttributeReader::AttributeType::AT_POS;
  if ((attrType & AttributeReader::AttributeType::AT_COL) ==
      AttributeReader::AttributeType::AT_COL) {
    this->addColors();
  }
  if ((attrType & AttributeReader::AttributeType::AT_REFLE) ==
      AttributeReader::AttributeType::AT_REFLE) {
    this->addReflectances();
  }
  for (auto& attr : attributes) {
    attr.setTarget();
  }
  ///< read ply data
  TSize numPointFromeFile = 0;
  TSize numAttributes = attributes.size();
  while (!plyFile.eof() && numPointFromeFile < m_numPoint) {
    if (isAscii) {
      getline(plyFile, line);
      getTokens(line, tokens);
      if (tokens.empty())  ///< empty line
        continue;
      if (!checkCond(tokens.size() >= numAttributes, "Error: insufficient ply data!"))
        return false;
      for (Int i = 0; i < numAttributes; i++)
        attributes[i].readFromAscii(tokens[i]);
    } else {
      for (Int i = 0; i < numAttributes; i++)
        attributes[i].readFromBinary(plyFile);
    }

    numPointFromeFile++;
  }
  setNumPoint(numPointFromeFile);

  plyFile.close();
  return true;
}

Bool TComPointCloud::writeToFile(const string& fileName, const Bool isAscii) const {
  std::ofstream plyFile(fileName, ofstream::out | ofstream::binary);

  if (!checkCond(plyFile.is_open(), "Error: failed to write ply file!"))
    return EXIT_FAILURE;

  ///< write ply header
  plyFile << "ply" << endl;
  if (isAscii)
    plyFile << "format ascii 1.0" << endl;
  else {
    if (isLittleEndian())
      plyFile << "format binary_little_endian 1.0" << endl;
    else
      plyFile << "format binary_big_endian 1.0" << endl;
  }
  plyFile << "element vertex " << m_numPoint << endl;
  assert(m_pos.size() == m_numPoint);
  plyFile << "property float64 x" << endl;
  plyFile << "property float64 y" << endl;
  plyFile << "property float64 z" << endl;
  if (m_color.size() > 0) {
    assert(m_color.size() == m_numPoint);
    plyFile << "property uchar red" << endl;
    plyFile << "property uchar green" << endl;
    plyFile << "property uchar blue" << endl;
  }
  if (m_refl.size() > 0) {
    assert(m_refl.size() == m_numPoint);
    plyFile << "property int16 reflectance" << endl;
  }
  plyFile << "end_header" << endl;

  ///< write ply data
  if (isAscii) {
    plyFile << fixed << setprecision(6);
    for (TSize i = 0; i < m_numPoint; i++) {
      plyFile << m_pos[i];
      if (m_color.size() > 0)
        plyFile << ' ' << static_cast<int>(m_color[i][0]) << ' ' << static_cast<int>(m_color[i][1])
                << ' ' << static_cast<int>(m_color[i][2]);
      if (m_refl.size() > 0)
        plyFile << ' ' << m_refl[i];
      plyFile << endl;
    }
  } else {
    for (TSize i = 0; i < m_numPoint; i++) {
      plyFile.write((TChar*)&(m_pos[i][0]), sizeof(PC_POS));
      if (m_color.size() > 0)
        plyFile.write((TChar*)&(m_color[i][0]), sizeof(PC_COL));
      if (m_refl.size() > 0)
        plyFile.write((TChar*)&(m_refl[i]), sizeof(PC_REFL));
    }
  }

  plyFile.close();
  return EXIT_SUCCESS;
}

Void TComPointCloud::computeBoundingBox(PC_POS& boxMin, PC_POS& boxMax) const {
  boxMin = std::numeric_limits<Double>::max();
  boxMax = std::numeric_limits<Double>::min();
  for (const auto& pos : m_pos) {
    for (Int i = 0; i < 3; i++) {
      if (boxMin[i] > pos[i])
        boxMin[i] = pos[i];
      if (boxMax[i] < pos[i])
        boxMax[i] = pos[i];
    }
  }
}

PC_POS& TComPointCloud::operator[](const TSize i) {
  assert(i < m_numPoint);
  return m_pos[i];
}

const PC_POS& TComPointCloud::operator[](const TSize i) const {
  assert(i < m_numPoint);
  return m_pos[i];
}

const TSize& TComPointCloud::getNumPoint() const {
  return m_numPoint;
}

const vector<PC_POS>& TComPointCloud::positions() const {
  return m_pos;
}

vector<PC_POS>& TComPointCloud::positions() {
  return m_pos;
}

Void TComPointCloud::setNumPoint(TSize numPoint) {
  m_numPoint = numPoint;
  m_pos.resize(numPoint);
  if (m_color.size() > 0)
    m_color.resize(numPoint);
  if (m_refl.size() > 0)
    m_refl.resize(numPoint);
}

PC_COL TComPointCloud::getColor(const TSize index) const {
  assert(index < m_color.size());
  return m_color[index];
}
PC_COL& TComPointCloud::getColor(const TSize index) {
  assert(index < m_color.size());
  return m_color[index];
}
const vector<PC_COL>& TComPointCloud::getColors() const {
  return m_color;
}
vector<PC_COL>& TComPointCloud::getColors() {
  return m_color;
}
Void TComPointCloud::setColor(const TSize index, const V3<UInt8> color) {
  assert(index < m_color.size());
  m_color[index] = color;
}
Void TComPointCloud::setColors(vector<PC_COL> colors) {
  m_color = colors;
}
PC_REFL TComPointCloud::getReflectance(const TSize index) const {
  assert(index < m_refl.size());
  return m_refl[index];
}
PC_REFL& TComPointCloud::getReflectance(const TSize index) {
  assert(index < m_refl.size());
  return m_refl[index];
}
vector<PC_REFL>& TComPointCloud::getReflectances() {
  return m_refl;
}
Void TComPointCloud::setReflectance(const TSize index, const UInt16 reflectance) {
  assert(index < m_refl.size());
  m_refl[index] = reflectance;
}

Bool TComPointCloud::hasReflectances() const {
  return m_refl.size() > 0;
}
Void TComPointCloud::addReflectances() {
  m_refl.resize(m_numPoint);
}
Void TComPointCloud::removeReflectances() {
  m_refl.resize(0);
}

Bool TComPointCloud::hasColors() const {
  return m_color.size() > 0;
}
Void TComPointCloud::addColors() {
  m_color.resize(m_numPoint);
}
Void TComPointCloud::removeColors() {
  m_color.resize(0);
}
Void TComPointCloud::init(const size_t size, bool WithColor, bool WithRef) {
  setNumPoint(size);
  if (WithColor == true) {
    addColors();
  } else {
    removeColors();
  }
  if (WithRef == true) {
    addReflectances();
  } else {
    removeReflectances();
  }
}

Void TComPointCloud::convertRGBToYUV() {
  for (auto& color : m_color) {
    const uint8_t r = color[0];
    const uint8_t g = color[1];
    const uint8_t b = color[2];
    const double y = TComClip(0., 255., std::round(0.212600 * r + 0.715200 * g + 0.072200 * b));
    const double u = TComClip(0., 255., std::round(-0.114572 * r - 0.385428 * g + 0.5 * b + 128.0));
    const double v = TComClip(0., 255., std::round(0.5 * r - 0.454153 * g - 0.045847 * b + 128.0));
    color[0] = static_cast<uint8_t>(y);
    color[1] = static_cast<uint8_t>(u);
    color[2] = static_cast<uint8_t>(v);
  }
}

Void TComPointCloud::convertYUVToRGB() {
  for (auto& color : m_color) {
    const double y1 = color[0];
    const double u1 = color[1] - 128.0;
    const double v1 = color[2] - 128.0;
    const double r = TComClip(0.0, 255.0, round(y1 + 1.57480 * v1));
    const double g = TComClip(0.0, 255.0, round(y1 - 0.18733 * u1 - 0.46813 * v1));
    const double b = TComClip(0.0, 255.0, round(y1 + 1.85563 * u1));
    color[0] = static_cast<uint8_t>(r);
    color[1] = static_cast<uint8_t>(g);
    color[2] = static_cast<uint8_t>(b);
  }
}

bool TComColorTransfer(const TComPointCloud& pointCloudOrg, double geomQuanStep, PC_VC3 quanOffset,
                       TComPointCloud& pointCloudRec) {
  const size_t orgPointCount = pointCloudOrg.getNumPoint();
  const size_t recPointCount = pointCloudRec.getNumPoint();
  if (!orgPointCount || !recPointCount || !pointCloudOrg.hasColors()) {
    return false;
  }

  pointCloudRec.addColors();
  pc_evalue::KdTree kdtreeOrg(pointCloudOrg.positions(), 10);
  pc_evalue::KdTree kdtreeRec(pointCloudRec.positions(), 10);

  std::vector<std::vector<PC_COL>> referColors1;
  referColors1.resize(recPointCount);

  // For each point of the origin point cloud,
  // find its nearest neighbor in the reconstructed cloud
  std::vector<size_t> indices;
  std::vector<double> sqrDist;
  //This will used a original point to chongjian points
  for (Int i = 0; i < orgPointCount; i++) {
    const PC_COL curColor = pointCloudOrg.getColor(i);
    PC_VC3 QuanPos = (pointCloudOrg[i] - quanOffset) * geomQuanStep;
    int resultMax1 = 30;
    int resultNum1 = 0;
    do {
      resultNum1 += 5;
      indices.resize(resultNum1);
      sqrDist.resize(resultNum1);
      nanoflann::KNNResultSet<double> resultSet1(resultNum1);
      resultSet1.init(&indices[0], &sqrDist[0]);
      kdtreeRec.m_indices->findNeighbors(resultSet1, &QuanPos[0], nanoflann::SearchParams(10));
    } while (sqrDist[0] == sqrDist[resultNum1 - 1] && resultNum1 + 5 <= resultMax1);
    //! same distance points
    referColors1[indices[0]].push_back(curColor);
    for (int j = 1; j < resultNum1; j++) {
      if (abs(sqrDist[j] - sqrDist[0]) < 1e-8) {
        referColors1[indices[j]].push_back(curColor);
      } else
        break;
    }
  }
  // -First project the points of the origin cloud to the reconstructed cloud.
  //  In case multiple origin points map to a single reconstructed point, the
  //  mean value is used.
  // -For the remained uncolored points, use their nearest neighbor attribute
  //  found above as their new attribute value
  for (Int i = 0; i < recPointCount; i++) {
    if (referColors1[i].empty()) {
      // if using Orignal KD-Tree not found nearset,will used Reconstruct KD-Tree
      double InverseQuanStep = 1.0 / geomQuanStep;
      PC_VC3 InverQuanPos = pointCloudRec[i] * InverseQuanStep + quanOffset;

      int resultMax = 30;
      int resultNum2 = 0;
      do {
        resultNum2 += 5;
        indices.resize(resultNum2);
        sqrDist.resize(resultNum2);
        nanoflann::KNNResultSet<double> resultSet2(resultNum2);
        resultSet2.init(&indices[0], &sqrDist[0]);
        kdtreeOrg.m_indices->findNeighbors(resultSet2, &InverQuanPos[0],
                                           nanoflann::SearchParams(10));
      } while (sqrDist[0] == sqrDist[resultNum2 - 1] && resultNum2 + 5 <= resultMax);

      size_t idx = indices[0];
      assert(idx >= 0);

      //! same distance points
      std::vector<size_t> multineighbour;
      multineighbour.push_back(idx);
      for (int j = 1; j < resultNum2; j++) {
        if (abs(sqrDist[j] - sqrDist[0]) < 1e-8) {
          multineighbour.push_back(indices[j]);
        } else
          break;
      }

      PC_VC3 colorAvg(0.0);
      for (const auto idx : multineighbour) {
        const auto color = pointCloudOrg.getColor(idx);
        for (size_t k = 0; k < 3; k++) {
          colorAvg[k] += color[k];
        }
      }
      colorAvg /= (double)multineighbour.size();
      PC_COL refColor;
      for (size_t k = 0; k < 3; k++) {
        refColor[k] = uint8_t(TComClip(0.0, 255.0, std::round(colorAvg[k])));
      }
      pointCloudRec.setColor(i, refColor);
    } else {
      PC_VC3 avgAttr(0.0);
      for (const auto color : referColors1[i]) {
        for (size_t k = 0; k < 3; k++) {
          avgAttr[k] += color[k];
        }
      }
      avgAttr /= double(referColors1[i].size());
      PC_COL avgColor;
      for (size_t k = 0; k < 3; k++) {
        avgColor[k] = uint8_t(TComClip(0.0, 255.0, std::round(avgAttr[k])));
      }
      pointCloudRec.setColor(i, avgColor);
    }
  }
  return true;
}

bool TComReflectanceTransfer(const TComPointCloud& pointCloudOrg, double geomQuanStep,
                             PC_VC3 quanOffset, TComPointCloud& pointCloudRec) {
  const size_t orgPointCount = pointCloudOrg.getNumPoint();
  const size_t recPointCount = pointCloudRec.getNumPoint();
  if (!orgPointCount || !recPointCount || !pointCloudOrg.hasReflectances()) {
    return false;
  }

  pointCloudRec.addReflectances();
  pc_evalue::KdTree kdtreeOrg(pointCloudOrg.positions(), 10);
  pc_evalue::KdTree kdtreeRec(pointCloudRec.positions(), 10);

  std::vector<std::vector<PC_REFL>> referReflectances1;
  referReflectances1.resize(recPointCount);

  // For each point of the origin point cloud,
  // find its nearest neighbor in the reconstructed cloud
  std::vector<size_t> indices;
  std::vector<double> sqrDist;
  //This will used a original point to chongjian points
  for (Int i = 0; i < orgPointCount; i++) {
    const PC_REFL curRefl = pointCloudOrg.getReflectance(i);
    PC_VC3 QuanPos = (pointCloudOrg[i] - quanOffset) * geomQuanStep;
    int resultMax1 = 30;
    int resultNum1 = 0;
    do {
      resultNum1 += 5;
      indices.resize(resultNum1);
      sqrDist.resize(resultNum1);
      nanoflann::KNNResultSet<double> resultSet1(resultNum1);
      resultSet1.init(&indices[0], &sqrDist[0]);
      kdtreeRec.m_indices->findNeighbors(resultSet1, &QuanPos[0], nanoflann::SearchParams(10));
    } while (sqrDist[0] == sqrDist[resultNum1 - 1] && resultNum1 + 5 <= resultMax1);
    //! same distance points
    referReflectances1[indices[0]].push_back(curRefl);
    for (int j = 1; j < resultNum1; j++) {
      if (abs(sqrDist[j] - sqrDist[0]) < 1e-8) {
        referReflectances1[indices[j]].push_back(curRefl);
      } else
        break;
    }
  }
  // -First project the points of the origin cloud to the reconstructed cloud.
  //  In case multiple origin points map to a single reconstructed point, the
  //  mean value is used.
  // -For the remained uncolored points, use their nearest neighbor attribute
  //  found above as their new attribute value
  for (Int i = 0; i < recPointCount; i++) {
    if (referReflectances1[i].empty()) {
      double InverseQuanStep = 1.0 / geomQuanStep;
      PC_VC3 InverQuanPos = pointCloudRec[i] * InverseQuanStep + quanOffset;

      int resultMax = 30;
      int resultNum2 = 0;
      do {
        resultNum2 += 5;
        indices.resize(resultNum2);
        sqrDist.resize(resultNum2);
        nanoflann::KNNResultSet<double> resultSet2(resultNum2);
        resultSet2.init(&indices[0], &sqrDist[0]);
        kdtreeOrg.m_indices->findNeighbors(resultSet2, &InverQuanPos[0],
                                           nanoflann::SearchParams(10));
      } while (sqrDist[0] == sqrDist[resultNum2 - 1] && resultNum2 + 5 <= resultMax);

      size_t idx = indices[0];
      assert(idx >= 0);

      //! same distance points
      std::vector<size_t> multineighbour;
      multineighbour.push_back(idx);
      for (int j = 1; j < resultNum2; j++) {
        if (abs(sqrDist[j] - sqrDist[0]) < 1e-8) {
          multineighbour.push_back(indices[j]);
        } else
          break;
      }

      double reflectanceAvg = 0.0;
      for (int i = 0; i < multineighbour.size(); i++) {
        reflectanceAvg += (double)pointCloudOrg.getReflectance(multineighbour[i]);
      }
      reflectanceAvg = TComClip(double(std::numeric_limits<PC_REFL>::min()),
                                double(std::numeric_limits<PC_REFL>::max()),
                                std::round(reflectanceAvg / double(multineighbour.size())));
      pointCloudRec.setReflectance(i, reflectanceAvg);
    } else {
      double avgAttr = 0.0;
      for (const auto reflectance : referReflectances1[i]) {
        avgAttr += reflectance;
      }
      avgAttr = TComClip(double(std::numeric_limits<PC_REFL>::min()),
                         double(std::numeric_limits<PC_REFL>::max()),
                         std::round(avgAttr / double(referReflectances1[i].size())));
      pointCloudRec.setReflectance(i, uint16_t(avgAttr));
    }
  }
  return true;
}

int recolour(const TComPointCloud& pointCloudOrg, float geomQuanStep, V3<int> quanOffset,
             TComPointCloud* pointCloudRec) {
  PC_VC3 doubleQuanOffset;
  for (int k = 0; k < 3; k++)
    doubleQuanOffset[k] = double(quanOffset[k]);

  if (pointCloudOrg.hasColors()) {
    bool ok = TComColorTransfer(pointCloudOrg, geomQuanStep, doubleQuanOffset, *pointCloudRec);

    if (!ok) {
      std::cout << "Error: can't transfer colors!" << std::endl;
      return -1;
    }
  }

  if (pointCloudOrg.hasReflectances()) {
    bool ok =
      TComReflectanceTransfer(pointCloudOrg, geomQuanStep, doubleQuanOffset, *pointCloudRec);

    if (!ok) {
      std::cout << "Error: can't transfer reflectance!" << std::endl;
      return -1;
    }
  }
  return 0;
}

void sortReconPoints(const TComPointCloud& pointCloudRecon, UInt lastNumReconPoints,
                     UInt currNumReconPoints, std::vector<pointMortonCodeWithIndex>& mortonOrder) {
  Int count = 0;
  for (UInt idx = lastNumReconPoints; idx < currNumReconPoints; idx++) {
    const PC_POS& point = pointCloudRecon[idx];
    mortonOrder[count].index = idx;
    mortonOrder[count].mortonCode = mortonAddr((Int32)point[0], (Int32)point[1], (Int32)point[2]);
    count++;
  }
  std::sort(mortonOrder.begin(), mortonOrder.end());
}

void sortReconPointsShift(const TComPointCloud& pointCloudRecon, const Int shift,
                          UInt lastNumReconPoints, UInt currNumReconPoints,
                          std::vector<pointMortonCodeWithIndex>& mortonOrder,
                          std::vector<int>& mortonToindex, std::vector<int>& mortonTomorton2) {
  Int count = 0;
  for (UInt idx = lastNumReconPoints; idx < currNumReconPoints; idx++) {
    auto& index = mortonOrder[count].index;
    mortonToindex[count] = index;
    const PC_POS& point = pointCloudRecon[index];
    mortonOrder[count].mortonCode =
      mortonAddr((Int32)point[0] + shift, (Int32)point[1] + shift, (Int32)point[2] + shift);
    mortonOrder[count].index = count;
    count++;
  }
  std::sort(mortonOrder.begin(), mortonOrder.end());
  count = 0;
  for (UInt idx = lastNumReconPoints; idx < currNumReconPoints; idx++) {
    mortonTomorton2[mortonOrder[count].index] = count;
    count++;
  }
}
int32_t determineInitShiftBits(const uint32_t voxelCount,
                               const std::vector<pointMortonCodeWithIndex>& pointCloudMorton) {
  int initShiftBits = -3;
  int recycleTime = 0;
  int neighborFlag = 0;
  int neighborFlagCount = 0;
  float Ratio = 0;
  float neighborFlagRatio = 0;
  while (Ratio < 1) {
    initShiftBits += 3;
    recycleTime = 0;
    neighborFlagCount = 0;
    int neighborCount = 0;
    for (int i = (voxelCount / 100); i < voxelCount; i += (voxelCount / 100)) {
      ++recycleTime;
      neighborFlag = 0;
      for (int j = i + 1; j < voxelCount; ++j) {
        if (pointCloudMorton[j].mortonCode >> (initShiftBits + 3) !=
            pointCloudMorton[i].mortonCode >> (initShiftBits + 3))
          break;
        else {
          ++neighborCount;
          neighborFlag = 1;
        }
      }
      for (int j = i - 1; j > 0; --j) {
        if (pointCloudMorton[j].mortonCode >> (initShiftBits + 3) !=
            pointCloudMorton[i].mortonCode >> (initShiftBits + 3))
          break;
        else {
          ++neighborCount;
          neighborFlag = 1;
        }
      }
      neighborFlagCount += neighborFlag;
    }
    Ratio = (float)neighborCount / recycleTime;
    neighborFlagRatio = (float)neighborFlagRatio / recycleTime;
  }
  if (neighborFlagRatio < 0.6)
    initShiftBits += 3;

  return initShiftBits;
}

Void levelOfDetailLayeringStructure(const AttributeParameterSet aps,
                                    const TComPointCloud& pointCloudOrg,
                                    std::vector<uint32_t>& layerToindex,
                                    std::vector<uint32_t>& numberOfPointsPerLayer,
                                    std::vector<pointMortonCodeWithIndex>& pointCloudMorton) {
  const int voxelCount = int(pointCloudOrg.getNumPoint());
  std::vector<int64_t> inputLayerMorton;
  std::vector<int64_t> nextLayerMorton;
  std::vector<uint32_t> nextLayer, inputLayer;

  numberOfPointsPerLayer.push_back(0);
  //determine initial shift bits
  //determin shift
  int32_t ShiftBits = determineInitShiftBits(voxelCount, pointCloudMorton);

  inputLayer.resize(voxelCount);
  for (uint32_t i = 0; i < voxelCount; ++i)
    inputLayer[i] = i;

  if (aps.numOflevelOfDetail) {
    nextLayer.reserve(voxelCount);
    nextLayerMorton.reserve(voxelCount);
    inputLayerMorton.resize(voxelCount);
    for (uint32_t i = 0; i < voxelCount; ++i)
      inputLayerMorton[i] = pointCloudMorton[i].mortonCode >> ShiftBits;
  }

  int32_t predIndex = -1;
  int32_t startIndex, endIndex;
  // xun huan LOD
  for (uint32_t lodIndex = 0; !inputLayer.empty() && lodIndex <= aps.numOflevelOfDetail;
       ++lodIndex) {
    startIndex = layerToindex.size();
    if (aps.numOflevelOfDetail == 0) {
      layerToindex = inputLayer;
    } else if (lodIndex == aps.numOflevelOfDetail) {
      for (const auto index : inputLayer) {
        layerToindex.push_back(index);
      }
    } else {
      //these neighbor offsets are relative to base_pos;
      static const uint8_t neighOffset[20] = {7,  35, 21, 14, 17, 10, 33, 49, 34, 42,
                                              12, 20, 28, 32, 40, 8,  16, 48, 56, 24};
      std::vector<int64_t> neighbor_pos(20);
      int inputindex = 0;
      if (inputLayer.size() == 1) {
        layerToindex.push_back(inputLayer[0]);
      } else {
        for (auto index : inputLayer) {
          if (index > voxelCount) {
            layerToindex.push_back(index - voxelCount);
            ++inputindex;
            continue;
          }
          nextLayer.push_back(index);
          nextLayerMorton.push_back(pointCloudMorton[index].mortonCode >> (ShiftBits + 3));

          int64_t cur_pos = pointCloudMorton[index].mortonCode >> ShiftBits;
          int64_t base_pos = morton3dAdd(cur_pos, -1ll);
          for (int i = 0; i < 20; i++) {
            neighbor_pos[i] = morton3dAdd(base_pos, neighOffset[i]);
            if (neighbor_pos[i] < neighbor_pos[0])
              continue;
            auto rangeBound = std::equal_range(inputLayerMorton.begin() + inputindex + 1,
                                               inputLayerMorton.end(), neighbor_pos[i]);
            auto begin = rangeBound.first - inputLayerMorton.begin();
            auto end = rangeBound.second - inputLayerMorton.begin();
            for (int32_t k = begin; k < end; ++k) {
              if (inputLayer[k] > voxelCount)
                continue;
              inputLayer[k] += voxelCount;
            }
          }
          inputindex++;
        }
      }
    }
    if (layerToindex.size())
      ShiftBits += 3;
    endIndex = layerToindex.size();

    numberOfPointsPerLayer.push_back(endIndex - startIndex);
    inputLayer.resize(0);
    std::swap(nextLayer, inputLayer);
    inputLayerMorton.resize(0);
    std::swap(nextLayerMorton, inputLayerMorton);
  }
}
///< \}
