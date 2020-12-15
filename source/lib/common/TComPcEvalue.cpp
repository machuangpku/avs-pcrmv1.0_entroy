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

#include "TComPcEvalue.h"

namespace pc_evalue {
TMetricCfg::TMetricCfg() {
  m_srcFile = "";
  m_disFile = "";
  m_calColor = false;
  m_calReflectance = false;
  m_peakValue = 0.0;
  m_duplicateMode = 1;
  m_multiNeighbourMode = 1;
  m_symmetry = true;
  m_showHausdorff = true;
}
bool TMetricCfg::parseCfg(Int argc, TChar* argv[]) {
  Bool isHelp = false;
  Bool ret = 0;

  // clang-format off
  TComParamParser parser;
  parser.addParameter() ///< data, default value, short option, long option, description
    (isHelp,               false,      "h",   "help",                "help")
    (ConfigParser,                     "c",   "config",              "config file")
    (m_srcFile,            string(""), "f1",  "file1",               "file name of file1")
    (m_disFile,            string(""), "f2",  "file2",               "file name of file2")
    (m_symmetry,           true,       "sy",  "symmetry",            "if calculate symmetry metrics")
    (m_calColor,           false,      "cc",  "cal_color",           "if calculate color metrics. 1: on, 0 : off")
    (m_calReflectance,     false,      "cr",  "cal_reflectance",     "if calculate reflectance metrics. 1: on, 0 : off")
    (m_peakValue,          0.0f,       "pk",  "peakvalue",           "peak value of Geometry PSNR(default = 0)")
    (m_duplicateMode,      true,       "dp",  "duplicate_mode",      "process duplicated points. 1: average 0 : no process")
    (m_multiNeighbourMode, true,       "ne",  "multineighbour_mode", "process same distance neighbours. 1: average 0 : no process")
    (m_showHausdorff,      true,       "hau", "show_hausdorff",      "if show hausdorff and hausdorffPSNR. 1: on, 0: off")
  ;
  // clang-format on

  parser.initParameters();                            ///< set the default parameters
  parser.parseParameters(argc, (const TChar**)argv);  ///< parsing
  parser.printInvalidParameters(cout);                ///< print warnings

  if (isHelp || argc == 1) {  ///< print help info
    parser.printHelp(cout);
    return false;
  }

  ret = checkParameter();  ///< check the validity of parameters

  parser.printParameters(cout);  ///< print parameter values

  return ret;
}

bool TMetricCfg::checkParameter() {
  bool isFailed = false;

  isFailed |= checkCond(m_srcFile.length() > 0, "Error: file1 must be specified.");
  isFailed |= checkCond(m_disFile.length() > 0, "Error: file2 must be specified.");

  return isFailed;
}

TMetricRes::TMetricRes() {
  m_geometryMSE = 0.0;
  m_geometryHausdorff = 0.0;
  m_geometryPSNR = 0.0;
  m_geometryHausdorffPSNR = 0.0;

  m_colorMSE[0] = m_colorMSE[1] = m_colorMSE[2] = 0.0;
  m_colorHausdorff[0] = m_colorHausdorff[1] = m_colorHausdorff[2] = 0.0;
  m_colorPSNR[0] = m_colorPSNR[1] = m_colorPSNR[2] = 0.0;
  m_colorHausdorffPSNR[0] = m_colorHausdorffPSNR[1] = m_colorHausdorffPSNR[2] = 0.0;

  m_reflectanceMSE = 0.0;
  m_reflectanceHausdorff = 0.0;
  m_reflectancePSNR = 0.0;
  m_reflectanceHausdorffPSNR = 0.0;

  m_peakValue = 0.0;
}

void TMetricRes::print(TMetricCfg para) {
  cout.setf(ios::fixed);
  cout.precision(6);
  std::cout << "All frames metric result" << std::endl;
  std::cout << "   D1_PSNR_Ave          : " << m_geometryPSNR << std::endl;
  if (para.m_showHausdorff) {
    std::cout << "   D1_HausdorffPSNR : " << m_geometryHausdorffPSNR << std::endl;
  }
  if (para.m_calColor) {
    std::cout << "   c[0]_PSNR_Ave           : " << m_colorPSNR[0] << std::endl;
    std::cout << "   c[1]_PSNR_Ave           : " << m_colorPSNR[1] << std::endl;
    std::cout << "   c[2]_PSNR_Ave           : " << m_colorPSNR[2] << std::endl;
    if (para.m_showHausdorff) {
      std::cout << "   c[0]_HausdorffPSNR_Ave  : " << m_colorHausdorffPSNR[0] << std::endl;
      std::cout << "   c[1]_HausdorffPSNR_Ave  : " << m_colorHausdorffPSNR[1] << std::endl;
      std::cout << "   c[2]_HausdorffPSNR_Ave  : " << m_colorHausdorffPSNR[2] << std::endl;
    }
  }
  if (para.m_calReflectance) {
    std::cout << "   rel_PSNR_Ave            : " << m_reflectancePSNR << std::endl;
    if (para.m_showHausdorff) {
      std::cout << "   rel_HausdroffPSNR_Ave   : " << m_reflectanceHausdorffPSNR << std::endl;
    }
  }
  std::cout << std::endl;
  cout.unsetf(ios::fixed);
}
TMetricRes TMetricRes::operator+(TMetricRes r) {
  TMetricRes res;
  res.m_geometryPSNR = m_geometryPSNR + r.m_geometryPSNR;
  res.m_geometryHausdorffPSNR = m_geometryHausdorffPSNR + r.m_geometryHausdorffPSNR;
  res.m_colorPSNR[0] = m_colorPSNR[0] + r.m_colorPSNR[0];
  res.m_colorPSNR[1] = m_colorPSNR[1] + r.m_colorPSNR[1];
  res.m_colorPSNR[2] = m_colorPSNR[2] + r.m_colorPSNR[2];
  res.m_colorHausdorffPSNR[0] = m_colorHausdorffPSNR[0] + r.m_colorHausdorffPSNR[0];
  res.m_colorHausdorffPSNR[1] = m_colorHausdorffPSNR[1] + r.m_colorHausdorffPSNR[1];
  res.m_colorHausdorffPSNR[2] = m_colorHausdorffPSNR[2] + r.m_colorHausdorffPSNR[2];
  res.m_reflectancePSNR = m_reflectancePSNR + r.m_reflectancePSNR;
  res.m_reflectanceHausdorffPSNR = m_reflectanceHausdorffPSNR + r.m_reflectanceHausdorffPSNR;
  return res;
}

TMetricRes TMetricRes::operator/(unsigned int numOfFrames) {
  TMetricRes res;
  res.m_geometryPSNR = m_geometryPSNR / numOfFrames;
  res.m_geometryHausdorffPSNR = m_geometryHausdorffPSNR / numOfFrames;
  res.m_colorPSNR[0] = m_colorPSNR[0] / numOfFrames;
  res.m_colorPSNR[1] = m_colorPSNR[1] / numOfFrames;
  res.m_colorPSNR[2] = m_colorPSNR[2] / numOfFrames;
  res.m_colorHausdorffPSNR[0] = m_colorHausdorffPSNR[0] / numOfFrames;
  res.m_colorHausdorffPSNR[1] = m_colorHausdorffPSNR[1] / numOfFrames;
  res.m_colorHausdorffPSNR[2] = m_colorHausdorffPSNR[2] / numOfFrames;
  res.m_reflectancePSNR = m_reflectancePSNR / numOfFrames;
  res.m_reflectanceHausdorffPSNR = m_reflectanceHausdorffPSNR / numOfFrames;
  return res;
}

struct hash_name {
  size_t operator()(const PC_POS& p) const {
    return hash<Double>()(p[0]) ^ hash<Double>()(p[1]) ^ hash<Double>()(p[2]);
  }
};
struct equalTo {
  bool operator()(const PC_POS& a, const PC_POS& b) const {
    return abs(a[0] - b[0]) < 1e-8 && abs(a[1] - b[1]) < 1e-8 && abs(a[2] - b[2]) < 1e-8;
  }
};

void RGB2YUV_BT709(const PC_COL& rgb, PC_COL& yuv) {
  //! convert RGB to YUV by BT709 (HDTV)
  /* yuv[0] = float((0.2126 * rgb[0] + 0.7152 * rgb[1] + 0.0722 * rgb[2]) / 255.0);
  yuv[1] = float((-0.1146 * rgb[0] - 0.3854 * rgb[1] + 0.5000 * rgb[2]) / 255.0 + 0.5000);
  yuv[2] = float((0.5000 * rgb[0] - 0.4542 * rgb[1] - 0.0458 * rgb[2]) / 255.0 + 0.5000);*/
  const double y =
    TComClip(0., 255., std::round(0.212600 * rgb[0] + 0.715200 * rgb[1] + 0.072200 * rgb[2]));
  const double u =
    TComClip(0., 255., std::round(-0.114572 * rgb[0] - 0.385428 * rgb[1] + 0.5 * rgb[2] + 128.0));
  const double v =
    TComClip(0., 255., std::round(0.5 * rgb[0] - 0.454153 * rgb[1] - 0.045847 * rgb[2] + 128.0));
  yuv[0] = static_cast<uint8_t>(y);
  yuv[1] = static_cast<uint8_t>(u);
  yuv[2] = static_cast<uint8_t>(v);
}

float getPSNR(float MSE, float peakValue, float factor) {
  double maxEnergy = double(peakValue) * double(peakValue);
  float PSNR = float(10.0 * std::log10((double(factor) * maxEnergy) / double(MSE)));
  return PSNR;
}

double dist2(float dist) {
  return dist * dist;
}

bool checkPointCloud(const TComPointCloud& in, TComPointCloud& out, TMetricCfg& para) {
  if (!in.getNumPoint()) {
    cout << "ERROR: point cloud is empty!" << endl;
    return false;
  }
  if (!in.hasColors() && para.m_calColor) {
    cout << "WARNING: point cloud has no color!" << endl;
    para.m_calColor = false;
  }

  if (!in.hasReflectances() && para.m_calReflectance) {
    cout << "WARNING: point cloud has no reflectance!" << endl;
    para.m_calReflectance = false;
  }

  out = in;
  if (para.m_duplicateMode) {
    std::unordered_map<PC_POS, std::vector<size_t>, hash_name, equalTo> dic;
    for (size_t i = 0; i < in.getNumPoint(); i++) {
      if (dic.find(in[i]) != dic.end())
        dic[in[i]].push_back(i);
      else
        dic.insert(pair<PC_POS, vector<size_t>>(in[i], {i}));
    }

    out.setNumPoint(dic.size());
    size_t count = 0;
    for (auto ele : dic) {
      assert(count < dic.size());
      double colorSum[3] = {0, 0, 0};
      double reflSum = 0;
      for (size_t i = 0; i < ele.second.size(); i++) {
        if (para.m_calColor) {
          colorSum[0] += in.getColor(ele.second[i])[0];
          colorSum[1] += in.getColor(ele.second[i])[1];
          colorSum[2] += in.getColor(ele.second[i])[2];
        }
        if (para.m_calReflectance) {
          reflSum += in.getReflectance(ele.second[i]);
        }
      }
      out[count] = ele.first;
      if (para.m_calColor) {
        out.getColor(count)[0] = UInt8(colorSum[0] / ele.second.size());
        out.getColor(count)[1] = UInt8(colorSum[1] / ele.second.size());
        out.getColor(count)[2] = UInt8(colorSum[2] / ele.second.size());
      }
      if (para.m_calReflectance) {
        out.getReflectance(count) = UInt16(reflSum / ele.second.size());
      }
      count++;
    }
  }
  size_t numin = in.getNumPoint();
  size_t numout = out.getNumPoint();
  if (numin - numout > 0) {
    if (!para.m_duplicateMode)
      cout << " Same coordinates points size: " << numin - numout << endl;
    else
      cout << " Same coordinates points size (averaged): " << numin - numout << endl;
    cout << " points remained:  " << numout << endl;
  } else {
    cout << " point cloud size: " << numin << endl;
  }
  return true;
}

float computePeakValue(const TComPointCloud& src) {
  float maxDistance = 0;
  KdTree kdtree = KdTree(src.positions(), 10);
  NNResult result;
  for (int i = 0; i < src.getNumPoint(); i++) {
    kdtree.search(src[i], 3, result);
    if (result.indices(0) != i || result.dist(1) < 1e-10) {
      //cout << "WARNING:  Some points are duplicated!" << endl;
    } else {
      if (result.dist(1) > maxDistance)
        maxDistance = float(result.dist(1));
    }
  }
  return sqrt(maxDistance);
}

void computeEValue(const TComPointCloud& src, const TComPointCloud& dis, TMetricCfg& para,
                   TMetricRes& res) {
  double geoMaxSquareErr = 0;
  double geoSumSquareErr = 0;
  double colorMaxSquareErr[3] = {0, 0, 0};
  double colorSumSquareErr[3] = {0, 0, 0};
  double reflMaxSquareErr = 0;
  double reflSumSquareErr = 0;
  size_t resultmax = 30 > src.getNumPoint() ? src.getNumPoint() : 30;
  KdTree kdtree = KdTree(dis.positions(), 10);
  NNResult result;
  for (int i = 0; i < src.getNumPoint(); i++) {
    int resultnum = 0;
    do {
      resultnum += 5;
      resultnum = resultnum > resultmax ? resultmax : resultnum;
      if (!kdtree.search(src[i], resultnum, result)) {
        cout << " WARNING: requested neighbors could not be found " << endl;
      }
    } while (result.dist(0) == result.dist(resultnum - 1) && (resultnum + 1) <= resultmax);

    int idx = result.indices(0);
    assert(idx >= 0);

    //! same distance points
    std::vector<size_t> multineighbour;
    multineighbour.push_back(idx);
    for (int j = 1; j < resultnum; j++) {
      if (abs(result.dist(j) - result.dist(0)) < 1e-8) {
        multineighbour.push_back(result.indices(j));
      } else
        break;
    }

    //! compute geometry EValue
    double geoSquareErr = result.dist(0);
    geoSumSquareErr += geoSquareErr;
    geoMaxSquareErr = std::max(geoMaxSquareErr, geoSquareErr);

    //! compute color EValue
    double colorSquareErr[3] = {0, 0, 0};
    if (para.m_calColor) {
      PC_COL src_yuv, dis_yuv;
      RGB2YUV_BT709(src.getColor(i), src_yuv);
      if (para.m_multiNeighbourMode) {
        //! average
        PC_COL colorTemp;
        double temp[3] = {0, 0, 0};
        for (auto& pnt : multineighbour) {
          temp[0] += dis.getColor(pnt)[0];
          temp[1] += dis.getColor(pnt)[1];
          temp[2] += dis.getColor(pnt)[2];
        }
        colorTemp[0] = (unsigned char)round(temp[0] / multineighbour.size());
        colorTemp[1] = (unsigned char)round(temp[1] / multineighbour.size());
        colorTemp[2] = (unsigned char)round(temp[2] / multineighbour.size());
        RGB2YUV_BT709(colorTemp, dis_yuv);
        colorMaxSquareErr[0] =
          std::max(colorMaxSquareErr[0], dist2(float(src.getColor(i)[0] - colorTemp[0])));
        colorMaxSquareErr[1] =
          std::max(colorMaxSquareErr[1], dist2(float(src.getColor(i)[1] - colorTemp[1])));
        colorMaxSquareErr[2] =
          std::max(colorMaxSquareErr[2], dist2(float(src.getColor(i)[2] - colorTemp[2])));
      } else {
        RGB2YUV_BT709(dis.getColor(idx), dis_yuv);
        colorMaxSquareErr[0] =
          std::max(colorMaxSquareErr[0], dist2(float(src.getColor(i)[0] - dis.getColor(idx)[0])));
        colorMaxSquareErr[1] =
          std::max(colorMaxSquareErr[1], dist2(float(src.getColor(i)[1] - dis.getColor(idx)[1])));
        colorMaxSquareErr[2] =
          std::max(colorMaxSquareErr[2], dist2(float(src.getColor(i)[2] - dis.getColor(idx)[2])));
      }
      colorSquareErr[0] = dist2(float(src_yuv[0] - dis_yuv[0]));
      colorSquareErr[1] = dist2(float(src_yuv[1] - dis_yuv[1]));
      colorSquareErr[2] = dist2(float(src_yuv[2] - dis_yuv[2]));
      colorSumSquareErr[0] += colorSquareErr[0];
      colorSumSquareErr[1] += colorSquareErr[1];
      colorSumSquareErr[2] += colorSquareErr[2];
    }
    //! compute reflectance EValue
    double reflSquareErr = 0.0;
    if (para.m_calReflectance) {
      double reflTemp = 0;
      if (para.m_multiNeighbourMode) {
        for (auto& pnt : multineighbour) {
          reflTemp += dis.getReflectance(pnt);
        }
      }
      reflSquareErr = dist2(float(src.getReflectance(i) - round(reflTemp / multineighbour.size())));
      //reflSquareErr = dist2(src.refl(i) - dis.refl(idx));
      reflSumSquareErr += reflSquareErr;
      reflMaxSquareErr = std::max(reflMaxSquareErr, reflSquareErr);
    }
  }
  //! compute geometry MSE and PSNR
  res.m_geometryMSE = float(geoSumSquareErr / src.getNumPoint());
  res.m_geometryHausdorff = float(geoMaxSquareErr);
  res.m_geometryPSNR = getPSNR(res.m_geometryMSE, res.m_peakValue, 3.0);
  res.m_geometryHausdorffPSNR = getPSNR(res.m_geometryHausdorff, res.m_peakValue, 3.0);
  //! compute colors MSE and PSNR
  if (para.m_calColor) {
    res.m_colorMSE[0] = float(colorSumSquareErr[0] / src.getNumPoint());
    res.m_colorMSE[1] = float(colorSumSquareErr[1] / src.getNumPoint());
    res.m_colorMSE[2] = float(colorSumSquareErr[2] / src.getNumPoint());
    res.m_colorPSNR[0] = getPSNR(res.m_colorMSE[0], float(std::numeric_limits<UInt8>::max()), 1.0);
    res.m_colorPSNR[1] = getPSNR(res.m_colorMSE[1], float(std::numeric_limits<UInt8>::max()), 1.0);
    res.m_colorPSNR[2] = getPSNR(res.m_colorMSE[2], float(std::numeric_limits<UInt8>::max()), 1.0);
    res.m_colorHausdorff[0] = float(colorMaxSquareErr[0]);
    res.m_colorHausdorff[1] = float(colorMaxSquareErr[1]);
    res.m_colorHausdorff[2] = float(colorMaxSquareErr[2]);
    res.m_colorHausdorffPSNR[0] =
      getPSNR(res.m_colorHausdorff[0], float(std::numeric_limits<UInt8>::max()), 1.0);
    res.m_colorHausdorffPSNR[1] =
      getPSNR(res.m_colorHausdorff[1], float(std::numeric_limits<UInt8>::max()), 1.0);
    res.m_colorHausdorffPSNR[2] =
      getPSNR(res.m_colorHausdorff[2], float(std::numeric_limits<UInt8>::max()), 1.0);
  }
  //! compute reflectance MSE and PSNR
  if (para.m_calReflectance) {
    res.m_reflectanceMSE = float(reflSumSquareErr / src.getNumPoint());
    res.m_reflectancePSNR =
      getPSNR(res.m_reflectanceMSE, float(std::numeric_limits<UInt16>::max()), 1.0);
    res.m_reflectanceHausdorff = float(reflMaxSquareErr);
    res.m_reflectanceHausdorffPSNR =
      getPSNR(res.m_reflectanceHausdorff, float(std::numeric_limits<UInt16>::max()), 1.0);
  }
}

bool computeMetric(const TComPointCloud& src, TComPointCloud& dis, TMetricCfg& para,
                   TMetricRes& res) {
  clock_t pc_evalue_TimeBegin = clock();
  cout << "Checking original point cloud..." << endl;
  TComPointCloud srcout;
  if (!checkPointCloud(src, srcout, para)) {
    cout << "ERROR: Load reference point cloud failed!" << endl;
    return false;
  }
  cout << "Checking reconstruct point cloud..." << endl;
  TComPointCloud disout;
  if (!checkPointCloud(dis, disout, para)) {
    cout << "ERROR: Load distortion point cloud failed!" << endl;
    return false;
  }

  TMetricRes resA;
  if (!para.m_peakValue) {
    resA.m_peakValue = computePeakValue(src);
    cout << "peakValue is set " << resA.m_peakValue << endl;
  } else {
    resA.m_peakValue = para.m_peakValue;
    cout << "peakValue: " << resA.m_peakValue << endl;
  }
  computeEValue(srcout, disout, para, resA);

  cout.setf(ios::fixed);
  cout.precision(6);
  cout << endl << "1. Take original point cloud as reference:" << endl;
  cout << "   D1_MSE_1           : " << resA.m_geometryMSE << endl;
  cout << "   D1_PSNR_1          : " << resA.m_geometryPSNR << endl;
  if (para.m_showHausdorff) {
    cout << "   D1_Hausdorff_1     : " << resA.m_geometryHausdorff << endl;
    cout << "   D1_HausdorffPSNR_1 : " << resA.m_geometryHausdorffPSNR << endl;
  }
  if (para.m_calColor) {
    cout << "   c[0]_MSE_1            : " << resA.m_colorMSE[0] << endl;
    cout << "   c[1]_MSE_1            : " << resA.m_colorMSE[1] << endl;
    cout << "   c[2]_MSE_1            : " << resA.m_colorMSE[2] << endl;
    cout << "   c[0]_PSNR_1           : " << resA.m_colorPSNR[0] << endl;
    cout << "   c[1]_PSNR_1           : " << resA.m_colorPSNR[1] << endl;
    cout << "   c[2]_PSNR_1           : " << resA.m_colorPSNR[2] << endl;
    if (para.m_showHausdorff) {
      cout << "   c[0]_Hausdorff_1      : " << resA.m_colorHausdorff[0] << endl;
      cout << "   c[1]_Hausdorff_1      : " << resA.m_colorHausdorff[1] << endl;
      cout << "   c[2]_Hausdorff_1      : " << resA.m_colorHausdorff[2] << endl;
      cout << "   c[0]_HausdorffPSNR_1  : " << resA.m_colorHausdorffPSNR[0] << endl;
      cout << "   c[1]_HausdorffPSNR_1  : " << resA.m_colorHausdorffPSNR[1] << endl;
      cout << "   c[2]_HausdorffPSNR_1  : " << resA.m_colorHausdorffPSNR[2] << endl;
    }
  }
  if (para.m_calReflectance) {
    cout << "   rel_MSE_1             : " << resA.m_reflectanceMSE << endl;
    cout << "   rel_PSNR_1            : " << resA.m_reflectancePSNR << endl;
    if (para.m_showHausdorff) {
      cout << "   rel_Hausdroff_1       : " << resA.m_reflectanceHausdorff << endl;
      cout << "   rel_HausdroffPSNR_1   : " << resA.m_reflectanceHausdorffPSNR << endl;
    }
  }

  if (para.m_symmetry) {
    TMetricRes resB;
    if (!para.m_peakValue)
      resB.m_peakValue = computePeakValue(src);
    else
      resB.m_peakValue = para.m_peakValue;
    computeEValue(disout, srcout, para, resB);
    cout << endl << "2. Take reconstruct point cloud as reference:" << endl;
    cout << "   D1_MSE_2           : " << resB.m_geometryMSE << endl;
    cout << "   D1_PSNR_2          : " << resB.m_geometryPSNR << endl;
    if (para.m_showHausdorff) {
      cout << "   D1_Hausdorff_2     : " << resB.m_geometryHausdorff << endl;
      cout << "   D1_HausdorffPSNR_2 : " << resB.m_geometryHausdorffPSNR << endl;
    }

    if (para.m_calColor) {
      cout << "   c[0]_MSE_2            : " << resB.m_colorMSE[0] << endl;
      cout << "   c[1]_MSE_2            : " << resB.m_colorMSE[1] << endl;
      cout << "   c[2]_MSE_2            : " << resB.m_colorMSE[2] << endl;
      cout << "   c[0]_PSNR_2           : " << resB.m_colorPSNR[0] << endl;
      cout << "   c[1]_PSNR_2           : " << resB.m_colorPSNR[1] << endl;
      cout << "   c[2]_PSNR_2           : " << resB.m_colorPSNR[2] << endl;
      if (para.m_showHausdorff) {
        cout << "   c[0]_Hausdorff_2      : " << resB.m_colorHausdorff[0] << endl;
        cout << "   c[1]_Hausdorff_2      : " << resB.m_colorHausdorff[1] << endl;
        cout << "   c[2]_Hausdorff_2      : " << resB.m_colorHausdorff[2] << endl;
        cout << "   c[0]_HausdorffPSNR_2  : " << resB.m_colorHausdorffPSNR[0] << endl;
        cout << "   c[1]_HausdorffPSNR_2  : " << resB.m_colorHausdorffPSNR[1] << endl;
        cout << "   c[2]_HausdorffPSNR_2  : " << resB.m_colorHausdorffPSNR[2] << endl;
      }
    }
    if (para.m_calReflectance) {
      cout << "   rel_MSE_2             : " << resB.m_reflectanceMSE << endl;
      cout << "   rel_PSNR_2            : " << resB.m_reflectancePSNR << endl;
      if (para.m_showHausdorff) {
        cout << "   rel_Hausdroff_2       : " << resB.m_reflectanceHausdorff << endl;
        cout << "   rel_HausdroffPSNR_2   : " << resB.m_reflectanceHausdorffPSNR << endl;
      }
    }
    res.m_geometryMSE = max(resA.m_geometryMSE, resB.m_geometryMSE);
    res.m_geometryPSNR = min(resA.m_geometryPSNR, resB.m_geometryPSNR);
    res.m_geometryHausdorff = max(resA.m_geometryHausdorff, resB.m_geometryHausdorff);
    res.m_geometryHausdorffPSNR = min(resA.m_geometryHausdorffPSNR, resB.m_geometryHausdorffPSNR);
    if (para.m_calColor) {
      res.m_colorMSE[0] = max(resA.m_colorMSE[0], resB.m_colorMSE[0]);
      res.m_colorMSE[1] = max(resA.m_colorMSE[1], resB.m_colorMSE[1]);
      res.m_colorMSE[2] = max(resA.m_colorMSE[2], resB.m_colorMSE[2]);
      res.m_colorPSNR[0] = min(resA.m_colorPSNR[0], resB.m_colorPSNR[0]);
      res.m_colorPSNR[1] = min(resA.m_colorPSNR[1], resB.m_colorPSNR[1]);
      res.m_colorPSNR[2] = min(resA.m_colorPSNR[2], resB.m_colorPSNR[2]);
      res.m_colorHausdorff[0] = max(resA.m_colorHausdorff[0], resB.m_colorHausdorff[0]);
      res.m_colorHausdorff[1] = max(resA.m_colorHausdorff[1], resB.m_colorHausdorff[1]);
      res.m_colorHausdorff[2] = max(resA.m_colorHausdorff[2], resB.m_colorHausdorff[2]);
      res.m_colorHausdorffPSNR[0] = min(resA.m_colorHausdorffPSNR[0], resB.m_colorHausdorffPSNR[0]);
      res.m_colorHausdorffPSNR[1] = min(resA.m_colorHausdorffPSNR[1], resB.m_colorHausdorffPSNR[1]);
      res.m_colorHausdorffPSNR[2] = min(resA.m_colorHausdorffPSNR[2], resB.m_colorHausdorffPSNR[2]);
    }
    if (para.m_calReflectance) {
      res.m_reflectanceMSE = max(resA.m_reflectanceMSE, resB.m_reflectanceMSE);
      res.m_reflectancePSNR = min(resA.m_reflectancePSNR, resB.m_reflectancePSNR);
      res.m_reflectanceHausdorff = max(resA.m_reflectanceHausdorff, resB.m_reflectanceHausdorff);
      res.m_reflectanceHausdorffPSNR =
        min(resA.m_reflectanceHausdorffPSNR, resB.m_reflectanceHausdorffPSNR);
    }

    cout << endl << "3. Symmetric result:" << endl;
    cout << "   D1_MSE_F           : " << res.m_geometryMSE << endl;
    cout << "   D1_PSNR_F          : " << res.m_geometryPSNR << endl;
    if (para.m_showHausdorff) {
      cout << "   D1_Hausdorff_F     : " << res.m_geometryHausdorff << endl;
      cout << "   D1_HausdorffPSNR_F : " << res.m_geometryHausdorffPSNR << endl;
    }
    if (para.m_calColor) {
      cout << "   c[0]_MSE_F            : " << res.m_colorMSE[0] << endl;
      cout << "   c[1]_MSE_F            : " << res.m_colorMSE[1] << endl;
      cout << "   c[2]_MSE_F            : " << res.m_colorMSE[2] << endl;
      cout << "   c[0]_PSNR_F           : " << res.m_colorPSNR[0] << endl;
      cout << "   c[1]_PSNR_F           : " << res.m_colorPSNR[1] << endl;
      cout << "   c[2]_PSNR_F           : " << res.m_colorPSNR[2] << endl;
      if (para.m_showHausdorff) {
        cout << "   c[0]_Hausdorff_F      : " << res.m_colorHausdorff[0] << endl;
        cout << "   c[1]_Hausdorff_F      : " << res.m_colorHausdorff[1] << endl;
        cout << "   c[2]_Hausdorff_F      : " << res.m_colorHausdorff[2] << endl;
        cout << "   c[0]_HausdorffPSNR_F  : " << res.m_colorHausdorffPSNR[0] << endl;
        cout << "   c[1]_HausdorffPSNR_F  : " << res.m_colorHausdorffPSNR[1] << endl;
        cout << "   c[2]_HausdorffPSNR_F  : " << res.m_colorHausdorffPSNR[2] << endl;
      }
    }
    if (para.m_calReflectance) {
      cout << "   rel_MSE_F             : " << res.m_reflectanceMSE << endl;
      cout << "   rel_PSNR_F            : " << res.m_reflectancePSNR << endl;
      if (para.m_showHausdorff) {
        cout << "   rel_Hausdroff_F       : " << res.m_reflectanceHausdorff << endl;
        cout << "   rel_HausdroffPSNR_F   : " << res.m_reflectanceHausdorffPSNR << endl;
      }
    }
  }
  cout.unsetf(ios::fixed);
  Double userTime = (Double)(clock() - pc_evalue_TimeBegin) / CLOCKS_PER_SEC;
  cout << "Point cloud evalue processing time (user): " << userTime << " sec." << endl << endl;
  return true;
}
}  // namespace pc_evalue
