/* The copyright in this software is being made available under the BSD
 * License, included below. This software may be subject to other third party
 * and contributor rights, including patent rights, and no such rights are
 * granted under this license.
 *
 * <OWNER> = Apple Inc.
 * <ORGANIZATION> = Apple Inc.
 * <YEAR> = 2014
 *
 * Copyright (c) 2014, Apple Inc.
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
 *  * Neither the name of the <ORGANIZATION> nor the names of its contributors may
 *    be used to endorse or promote products derived from this software without
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

/*!
 *************************************************************************************
 * \file ColorTransform.cpp
 *
 * \brief
 *    Base Class for color transform application
 *
 * \author
 *     - Alexis Michael Tourapis         <atourapis@apple.com>
 *
 *************************************************************************************
 */

//-----------------------------------------------------------------------------
// Include headers
//-----------------------------------------------------------------------------

#include "Global.H"
#include "ColorTransform.H"
#include "ColorTransformNull.H"
#include "ColorTransformXYZ2YUpVp.H"
#include "ColorTransformGeneric.H"
#include "ColorTransformClosedLoop.H"
#include "ColorTransformClosedLoopY.H"
#include "ColorTransformClosedLoopCr.H"
#include "ColorTransformClosedLoopRGB.H"
#include "ColorTransformYAdjust.H"
#include "ColorTransformYAdjustAlt.H"
#include "ColorTransformYAdjustFast.H"
#include "ColorTransformYAdjustTele.H"
#include "ColorTransformYAdjustXYZ.H"
#include "ColorTransformFVDO.H"
#include "ColorTransformCL.H"

//-----------------------------------------------------------------------------
// Private methods
//-----------------------------------------------------------------------------

//-----------------------------------------------------------------------------
// Public methods
//-----------------------------------------------------------------------------
ColorTransform *ColorTransform::create( 
                                       ColorSpace        iColorSpace, 
                                       ColorPrimaries    iColorPrimaries, 
                                       ColorSpace        oColorSpace, 
                                       ColorPrimaries    oColorPrimaries, 
                                       bool              transformPrecision, 
                                       int               useHighPrecision,
                                       ClosedLoopTrans   closedLoopTransform, 
                                       int               iConstantLuminance, 
                                       int               oConstantLuminance, 
                                       TransferFunctions transferFunction,
                                       int               bitDepth,
                                       SampleRange       range,
                                       int               downMethod,
                                       int               upMethod,
                                       bool              useAdaptiveDownsampler,
                                       bool              useAdaptiveUpsampler,
                                       int               useMinMax,
                                       int               maxIterations,
                                       ChromaFormat      oChromaFormat,
                                       ChromaLocation    *oChromaLocationType,
                                       bool              useFloatPrecision
                                       ) {
  ColorTransform *result = NULL;
  
  if ((iColorSpace == oColorSpace) && (iColorPrimaries == oColorPrimaries) && ((iConstantLuminance == oConstantLuminance) || (iColorSpace != CM_YCbCr)))
    result = new ColorTransformNull();
  else if ((iColorSpace == CM_XYZ && oColorSpace == CM_YUpVp) || (iColorSpace == CM_YUpVp && oColorSpace == CM_XYZ)) 
    result = new ColorTransformXYZ2YUpVp();
  else if ((iColorSpace >= CM_YFBFRV1 && iColorSpace <= CM_YFBFRV2) || (oColorSpace >= CM_YFBFRV1 && oColorSpace <= CM_YFBFRV2))
    result = new ColorTransformFVDO(iColorSpace, oColorSpace);
  else if ((iColorPrimaries == oColorPrimaries) && 
           (((oColorSpace == CM_YCbCr) && (oConstantLuminance == 1) && (iColorSpace == CM_RGB)) ||
            ((iColorSpace == CM_YCbCr) && (iConstantLuminance == 1) && (oColorSpace == CM_RGB)))){
             result = new ColorTransformCL(iColorSpace, iColorPrimaries, oColorSpace, oColorPrimaries, useHighPrecision, transferFunction, transferFunction, TRUE);
           }
  else if ((iColorPrimaries == oColorPrimaries) && 
           (((oColorSpace == CM_YCbCr) && (oConstantLuminance == 2) && (iColorSpace == CM_RGB)) ||
            ((iColorSpace == CM_YCbCr) && (iConstantLuminance == 2) && (oColorSpace == CM_RGB)))){
             result = new ColorTransformCL(iColorSpace, iColorPrimaries, oColorSpace, oColorPrimaries, useHighPrecision, transferFunction, transferFunction, FALSE);
           }
  else if (closedLoopTransform == CLT_NULL)
    result = new ColorTransformGeneric(iColorSpace, iColorPrimaries, oColorSpace, oColorPrimaries, transformPrecision, useHighPrecision);
  else if (closedLoopTransform == CLT_YADJ)
    result = new ColorTransformYAdjust(iColorSpace, iColorPrimaries, oColorSpace, oColorPrimaries, useHighPrecision, transferFunction, downMethod, upMethod, useAdaptiveDownsampler, useAdaptiveUpsampler, useMinMax, bitDepth, range, maxIterations, oChromaFormat, oChromaLocationType, useFloatPrecision);
  else if (closedLoopTransform == CLT_YADJALT)
    result = new ColorTransformYAdjustAlt(iColorSpace, iColorPrimaries, oColorSpace, oColorPrimaries, useHighPrecision, transferFunction, downMethod, upMethod, useAdaptiveDownsampler, useAdaptiveUpsampler, useMinMax, bitDepth, range, maxIterations, oChromaFormat, oChromaLocationType, useFloatPrecision);
  else if (closedLoopTransform == CLT_CHROMA)
    result = new ColorTransformClosedLoopCr(iColorSpace, iColorPrimaries, oColorSpace, oColorPrimaries, useHighPrecision, transferFunction, downMethod, upMethod, useAdaptiveDownsampler, useAdaptiveUpsampler, useMinMax, bitDepth, range, maxIterations, oChromaFormat, oChromaLocationType, useFloatPrecision);
  else if (closedLoopTransform == CLT_RGB)
    result = new ColorTransformClosedLoopRGB(iColorSpace, iColorPrimaries, oColorSpace, oColorPrimaries, useHighPrecision, transferFunction, downMethod, upMethod, useAdaptiveDownsampler, useAdaptiveUpsampler, useMinMax, bitDepth, range, maxIterations, oChromaFormat, oChromaLocationType, useFloatPrecision);
  else if (closedLoopTransform == CLT_Y)
    result = new ColorTransformClosedLoopY(iColorSpace, iColorPrimaries, oColorSpace, oColorPrimaries, useHighPrecision, transferFunction, downMethod, upMethod, useAdaptiveDownsampler, useAdaptiveUpsampler, useMinMax, bitDepth, range, maxIterations, oChromaFormat, oChromaLocationType, useFloatPrecision);
  else if (closedLoopTransform == CLT_XYZ)
    result = new ColorTransformYAdjustXYZ(iColorSpace, iColorPrimaries, oColorSpace, oColorPrimaries, useHighPrecision, transferFunction, downMethod, upMethod, useAdaptiveDownsampler, useAdaptiveUpsampler, useMinMax, bitDepth, range, maxIterations, oChromaFormat, oChromaLocationType, useFloatPrecision);
  else if (closedLoopTransform == CLT_YADJFST)
    result = new ColorTransformYAdjustFast(iColorSpace, iColorPrimaries, oColorSpace, oColorPrimaries, useHighPrecision, transferFunction, downMethod, upMethod, useAdaptiveDownsampler, useAdaptiveUpsampler, useMinMax, bitDepth, range, maxIterations, oChromaFormat, oChromaLocationType, useFloatPrecision);
  else if (closedLoopTransform == CLT_YADJTELE)
    result = new ColorTransformYAdjustTele(iColorSpace, iColorPrimaries, oColorSpace, oColorPrimaries, useHighPrecision, transferFunction, downMethod, upMethod, useAdaptiveDownsampler, useAdaptiveUpsampler, useMinMax, bitDepth, range, maxIterations, oChromaFormat, oChromaLocationType, useFloatPrecision);
  else
    result = new ColorTransformClosedLoop(iColorSpace, iColorPrimaries, oColorSpace, oColorPrimaries, transformPrecision, useHighPrecision, closedLoopTransform, bitDepth, range);
  
  return result;
}

//-----------------------------------------------------------------------------
// End of file
//-----------------------------------------------------------------------------
