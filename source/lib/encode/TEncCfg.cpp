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

#include "TEncCfg.h"
#include "common/contributors.h"

///< \in TLibEncoder \{

/**
 * Implementation of TEncCfg
 * encoder configuration class
 */

//////////////////////////////////////////////////////////////////////////
// Public class functions
//////////////////////////////////////////////////////////////////////////

Bool TEncCfg::parseCfg(Int argc, TChar* argv[]) {
  Bool isHelp = false;
  Bool ret = 0;

  TComParamParser parser;
  // clang-format off
  parser.addParameter() ///< data, default value, short option, long option, description
    ("Generic")
    (isHelp,                             false,       "h",     "help",                          "help")
    (ConfigParser,                                    "c",     "config",                        "config file")
    (m_inputFileName,                    string(""),  "i",     "input",                         "original PLY input file name")
    (m_bitstreamFileName,                string(""),  "b",     "bitstream",                     "bitstream output file name")
    (m_reconFileName,                    string(""),  "r",     "recon",                         "reconstructed PLY output file name")
    (m_numOfFrames,                      (UInt)1,     "ftbc",  "frames_to_be_coded",            "number of frames to be coded. Default: 1")
    (m_geomOnlyFlag,                     false,       "gof",   "geom_only_flag",                "geometry-only coding mode")
    (m_colorTransformFlag,               false,       "ctf",   "color_transform_flag",          "apply color transform method. 1: on, 0: off")
    (m_writePlyInAsciiFlag,              true,        "awf",   "ascii_write_flag",              "recon ply write mode. 1: ascii, 0: binary")
    (m_MD5FileName,                      string(""),  "mdf",   "md5_file_name",                 "name of MD5 file")
    (m_numOfSlices,                      (UInt)1,      "s",    "slice_num",                     "number of slice to be division")
    ("Geometry")
    (m_hls.sps.geomQuantStep,            1.0f,        "gqs",   "geom_quant_step",               "geometry quantization step")
    (m_hls.sps.geomRemoveDuplicateFlag,  true,        "grdf",  "geom_remove_dup_flag",          "manipulating duplicate points. 1: remove, 0: keep")
    (m_hls.gps.depthFirstNodeSizeLog2,   (UInt)0,     "df",    "geom_depthfirst_nzlg2",         "geometry depth first partition nodeSizeLog2. 0: disabled, >0: enabled")
    (m_hls.gps.im_qtbt_flag,             true,        "ipf",   "geom_im_qtbt_flag",             "control flag for geometry implicit QTBT partition")
    (m_hls.gps.im_qtbt_num_before_ot,    (UInt)0,     "ipk",   "geom_im_qtbt_k",                "max num of OT before implicit QTBT")
    (m_hls.gps.im_qtbt_min_size,         (UInt)0,     "ipm",   "geom_im_qtbt_m",                "min size of implicit QTBT")
    (m_hls.gps.singleModeFlag,           (UInt)1,     "smf",   "geom_single_mode_flag",         "control flag for single point encode mode")
    (m_hls.gps.geom_context_mode,        (UInt)0,     "gcm",   "geom_context_mode",             "choose geometry context numbers (0:64 contexts,1:191contexts)")
    ("Attribute")
    (m_hls.sps.recolorMode,              (UInt)1,     "rm",    "recolor_mode",                  "select recolor mode: 0: regular recolor 1: fast recolor")
    (m_hls.sps.attrAdaptPred,            false,       "adp",   "attr_adapt_pred",               "predictor adaptation methods: 0: no adaptation, 1: allow change from position based to attribute based with index")
    (m_hls.sps.attrQuantParam,           (UInt)1,     "aqp",   "attr_quant_param",              "attribute quantization Parameter")
    (m_hls.aps.maxNumOfNeighbours,       (UInt)128,   "mnon",  "max_num_of_neighbours",         "search range of predicted neighbor")
    (m_hls.aps.numOflevelOfDetail,       (UInt)0,     "lodn",  "num_of_lod",                    "number of level of detail")
    (m_hls.aps.crossComponentPred,       false,       "ccp",   "cross_comp_pred",               "predict residuals of other components from the residual of the first component")
    (m_hls.aps.intraLodFlag,             (bool)1,     "ilodf", "intra_lod_flag",                "control flag for using points in intra LoD")
    (m_hls.aps.maxNumOfPredictNeighbours,(UInt)3,     "mnopn", "max_num_of_predict_neighbours", "number of neighbours used in prediction")
    ("DMetric")
    (m_metricsEnable,                    true,        "m",     "metrics_enable",                "calculate metrics flag. 1: on, 0: off")
    (m_symmetry,                         true,        "sy",    "symmetry",                      "if calculate symmetry metrics")
    (m_calColor,                         false,       "cc",    "cal_color",                     "if calculate color metrics. 1: on, 0 : off")
    (m_calReflectance,                   false,       "cr",    "cal_reflectance",               "if calculate reflectance metrics. 1: on, 0 : off")
    (m_peakValue,                        0.0f,        "pk",    "peakvalue",                     "peak value of Geometry PSNR(default = 0)")
    (m_duplicateMode,                    true,        "dp",    "duplicate_mode",                "process duplicated points. 1: average 0 : no process")
    (m_multiNeighbourMode,               true,        "ne",    "multineighbour_mode",           "process same distance neighbours. 1: average 0 : no process")
    (m_showHausdorff,                    true,        "hau",   "show_hausdorff",                "if show hausdorff and hausdorffPSNR. 1: on, 0: off")
    (m_PeakMemoryFlag,                   false,       "pmf",   "peak_memory_flag",              "output peak memory info. 1: on, 0: off")
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

//////////////////////////////////////////////////////////////////////////
// Private class functions
//////////////////////////////////////////////////////////////////////////

Bool TEncCfg::checkParameter() {
  Bool isFailed = false;

  isFailed |=
    checkCond(m_bitstreamFileName.length() > 0, "Error: bitstream file must be specified.");
  isFailed |= checkCond(m_inputFileName.length() > 0, "Error: input ply file must be specified.");
  isFailed |= checkCond(m_hls.sps.geomQuantStep > 0, "Error: invalid geometry quantization step.");
  isFailed |=
    checkCond(m_hls.sps.attrQuantParam >= 0, "Error: invalid attribute quantization step.");

  ///< depthFirstNodeSizeLog2 == 1 is equivilant to disable it
  if (m_hls.gps.depthFirstNodeSizeLog2 == 1)
    m_hls.gps.depthFirstNodeSizeLog2 = 0;

  ///< apply cross-component-prediction only when color transform is not used.
  if (!m_colorTransformFlag) {
    m_hls.aps.crossComponentPred = true;
  }

  return isFailed;
}

///< \}
