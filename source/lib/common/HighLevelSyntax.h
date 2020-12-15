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
#include "TComVector.h"
#include "contributors.h"

/**
 * Sequence Parameter Set
 */
struct SequenceParameterSet {
  UInt spsID;
  UInt level;

  V3<Int> geomBoundingBoxOrigin;  ///< geometry origin
  V3<UInt> geomBoundingBoxSize;   ///< geometry bounding box size
  UInt geomNumPoints;             ///< total number of points
  Float geomQuantStep;            ///< geometry quantization stepsize (in voxelization)
  Bool geomRemoveDuplicateFlag;   ///< remove duplicate points (=1) or keep duplicate points (=0)
  UInt recolorMode;               ///< 0: regular recolor, 1: fast recolor
  Bool attrAdaptPred;             ///< predictor adaptation methods:
                                  ///<   0: no adaptation,
    ///<   1: allow change from position based to attribute based with index
  UInt attrQuantParam;  ///< attribute quantization stepsize (attribute resudial value)
};

/**
 * Geometry Parameter Set
 */
struct GeometryParameterSet {
  UInt gpsID;
  UInt spsID;

  UInt depthFirstNodeSizeLog2;  ///< geometry level from which the depth-first partition starts
  Bool im_qtbt_flag;            ///< control flag for implicit QTBT partition
  UInt im_qtbt_num_before_ot;   ///< maximum number of implicit QTBT before OT
  UInt im_qtbt_min_size;        ///< minimum size in log2 of implicit QTBT
  UInt singleModeFlag;          ///< control flag for single point encode mode
  UInt geom_context_mode;       ///< geometry context reduction at parent level
};

/**
 * Geometry Brick Set
 */
struct GeometryBrickHeader {
  UInt gbhID;
  UInt sliceID;
  V3<Int> geomBoundingBoxOrigin;  ///< slice geometry origin
  V3<UInt> nodeSizeLog2;          ///< slice  node size (log2) for xyz dimensions
  UInt geomNumPoints;             ///< number of points in a slice
};

/**
* Attribute Parameter Set
*/
struct AttributeParameterSet {
  UInt apsID;
  UInt spsID;
  UInt maxNumOfNeighbours;
  UInt numOflevelOfDetail;
  UInt maxNumOfPredictNeighbours;
  bool intraLodFlag;
  Bool crossComponentPred;
  bool withColor = false;
  bool withRef = false;
};

/**
 * Attribute Brick Header
 */
struct AttributeBrickHeader {
  UInt abhID;
  UInt sliceID;
};

/**
 * High-Level Syntax
 * contains SPS, GPS, ...
 */
struct HighLevelSyntax {
  SequenceParameterSet sps;
  GeometryParameterSet gps;
  AttributeParameterSet aps;
  GeometryBrickHeader gbh;
  AttributeBrickHeader abh;
};
