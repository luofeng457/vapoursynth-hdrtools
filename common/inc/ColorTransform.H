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
 * \file ColorTransform.H
 *
 * \brief
 *    Base Class Header for the application of a Color Transform
 *
 * \author
 *     - Alexis Michael Tourapis         <atourapis@apple.com>
 *
 *************************************************************************************
 */


#ifndef __ColorTransform_H__
#define __ColorTransform_H__

#include "Global.H"
#include "Parameters.H"
#include "Frame.H"


class ColorTransform {
public:
  // Construct/Deconstruct
  virtual ~ColorTransform() {};
  static ColorTransform *create( 
                                ColorSpace        iColorTransform, 
                                ColorPrimaries    iColorPrimaries, 
                                ColorSpace        oColorTransform, 
                                ColorPrimaries    oColorPrimaries, 
                                bool              transformPrecision,
                                int               useHighPrecision,
                                ClosedLoopTrans   closedLoopTransform = CLT_NULL, 
                                int               iConstantLuminance = 0, 
                                int               oConstantLuminance = 0,
                                TransferFunctions transferFunction = TF_PQ,
                                int               bitDepth = 10,
                                SampleRange       range = SR_STANDARD,
                                int               downMethod = 2,
                                int               upMethod = 1,
                                bool              useAdaptiveDownsampler = FALSE,
                                bool              useAdaptiveUpsampler = FALSE,
                                int               useMinMax = 0,
                                int               maxIterations = 20,
                                ChromaFormat      oChromaFormat = CF_420,
                                ChromaLocation    *oChromaLocationType = NULL,
                                bool              useFloatPrecision = TRUE
                                );
  
  virtual void process(Frame *out,  const Frame *inp) = 0;
};

#endif
