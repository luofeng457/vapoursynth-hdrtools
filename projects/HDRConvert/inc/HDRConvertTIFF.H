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
 * \file HDRConvertTIFF.H
 *
 * \brief
 *    HDRConvertTIFF class (main) definition for converting TIFF files
 *
 * \author
 *     - Alexis Michael Tourapis         <atourapis@apple.com>
 *
 *************************************************************************************
 */

#ifndef __HDRConvertTIFF_H__
#define __HDRConvertTIFF_H__

#include "Global.H"
#include "HDRConvert.H"
#include "Input.H"
#include "Output.H"
#include "Frame.H"
#include "IOFunctions.H"
#include "Convert.H"
#include "ConvertColorFormat.H"
#include "ColorTransform.H"
#include "TransferFunction.H"
#include "FrameFilter.H"
#include "AddNoise.H"
#include "DisplayGammaAdjust.H"


class HDRConvertTIFF : public HDRConvert {
private:
  bool                  m_useSingleTransferStep;
  int                   m_useMinMax;
  bool                  m_filterInFloat;
  int                   m_startFrame;
  int                   m_noOfFrameStores;

  Input                *m_inputFrame;             // input frames
  Output               *m_outputFrame;            // output frames
  Frame                *m_oFrameStore;            // picture storage for output frames
  Frame                *m_iFrameStore;            // picture storage for input frames
  Frame                *m_dFrameStore;              // picture storage for linear downconversion
 
  Frame                *m_pFrameStore[7];         // picture storage for processing frames
  Frame                *m_pDFrameStore[7];          // picture storage for processing frames
  Frame                *m_croppedFrameStore;      // cropped frame store
  
  //Frame                *m_colorSpaceFrame;
  Frame                *m_scaledFrame;
  
  FrameFilter          *m_frameFilterNoise0;
  FrameFilter          *m_frameFilterNoise1;
  FrameFilter          *m_frameFilterNoise2;
  
  Convert              *m_convertProcess;         // Conversion process
  Convert              *m_convertIQuantize;       // Inverse Quantize TIFF files
  
  ColorTransform      *m_colorTransform;         // Color space conversion
  ColorTransform      *m_colorSpaceConvert;      // Alter color space in linear space, if for example in XYZ or in different RGB space
  
  ConvertColorFormat  *m_convertTo420;           // Chroma subsampling
  
  DisplayGammaAdjust   *m_srcDisplayGammaAdjust;    // Source Display Gamma adjustment (for the HLG TF)
  DisplayGammaAdjust   *m_outDisplayGammaAdjust;    // Output Display Gamma adjustment (for the HLG TF)

  AddNoise             *m_addNoise;                 // Noise Addition

  TransferFunction    *m_normalizeFunction;      //! Data normalization for OpenEXR inputs with linear light data
  TransferFunction    *m_outputTransferFunction; //! Inverse Transfer function
  TransferFunction    *m_inputTransferFunction;  //! Forward Transfer function

  ToneMapping          *m_toneMapping;
  void                 allocateFrameStores (ProjectParameters *inputParams, FrameFormat   *input, FrameFormat   *output);
  void                 deleteMemory();
  
  // Cropping information
  int                  m_cropOffsetLeft;
  int                  m_cropOffsetTop;
  int                  m_cropOffsetRight;
  int                  m_cropOffsetBottom;
  bool                 m_bUseWienerFiltering;
  bool                 m_bUse2DSepFiltering;
  bool                 m_bUseNLMeansFiltering;
  bool                 m_b2DSepMode;

  
public:
  HDRConvertTIFF                           (ProjectParameters *inputParams);
  
  virtual void         outputHeader        (ProjectParameters *inputParams);
  virtual void         outputFooter        (ProjectParameters *inputParams);
  virtual void         init                (ProjectParameters *inputParams);
  virtual void         process             (ProjectParameters *inputParams);
  virtual void         destroy             ();
  
};

#endif
