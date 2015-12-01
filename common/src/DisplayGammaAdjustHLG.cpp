/* The copyright in this software is being made available under the BSD
 * License, included below. This software may be subject to other third party
 * and contributor rights, including patent rights, and no such rights are
 * granted under this license.
 *
 * <OWNER> = British Broadcasting Corporation (BBC).
 * <ORGANIZATION> = BBC.
 * <YEAR> = 2015
 *
 * Copyright (c) 2015, BBC.
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
 * \file DisplayGammaAdjustHLG.cpp
 *
 * \brief
 *    DisplayGammaAdjustHLG class 
 *    This is an implementation of the display gamma normalization process
 *    in BBC's Hybrid Log-Gamma system (HLG).
 *    A complete documentation of this process is available in 
 *    the BBC's response to the MPEG call for evidence on HDR and WCG video coding 
 *   (document m36249, available at:
 *    http://wg11.sc29.org/doc_end_user/documents/112_Warsaw/wg11/m36249-v2-m36249.zip)
 *
 * \author
 *     - Matteo Naccari         <matteo.naccari@bbc.co.uk>
 *     - Manish Pindoria        <manish.pindoria@bbc.co.uk>
 *
 *************************************************************************************
 */

#include "Global.H"
#include "DisplayGammaAdjustHLG.H"
#include "ColorTransformGeneric.H"

//-----------------------------------------------------------------------------
// Constructor / destructor implementation
//-----------------------------------------------------------------------------

DisplayGammaAdjustHLG::DisplayGammaAdjustHLG(double gamma, double scale)
{
  m_tfScale  = 12.0; //19.6829249; // transfer function scaling - assuming super whites
  m_linScale = scale;
  m_gamma    = gamma;
}

DisplayGammaAdjustHLG::~DisplayGammaAdjustHLG()
{
}

void DisplayGammaAdjustHLG::forward(Frame *frame)
{
  if (frame->m_isFloat == TRUE && frame->m_compSize[Y_COMP] == frame->m_compSize[Cb_COMP])  {    
    if (frame->m_colorSpace == CM_RGB) {
      double vComp[3];
      const double *transformY = NULL;

      ColorTransformGeneric::setYConversion(frame->m_colorPrimaries, &transformY);
      
      for (int index = 0; index < frame->m_compSize[Y_COMP]; index++) {
        for(int component = 0; component < 3; component++)  {
          vComp[component] = frame->m_floatComp[component][index] / m_tfScale;
        }
        
        double ySignal = dMax(0.000001, transformY[R_COMP] * vComp[R_COMP] + transformY[G_COMP] * vComp[G_COMP] + transformY[B_COMP] * vComp[B_COMP]);
        double yDisplay = m_linScale * pow(ySignal, m_gamma) / ySignal;
        
        for(int component = 0; component < 3; component++)  {
          frame->m_floatComp[component][index] = (float) (vComp[component] * yDisplay) ;
        }  
      }
      // reset the pointer (just for safety
      transformY = NULL;
    }
  }
}

void DisplayGammaAdjustHLG::forward(Frame *out, const Frame *inp)
{
  if (inp->m_isFloat == TRUE && out->m_isFloat == TRUE && inp->m_size == out->m_size && inp->m_compSize[Y_COMP] == inp->m_compSize[Cb_COMP])  {    
    if (inp->m_colorSpace == CM_RGB && out->m_colorSpace == CM_RGB) {
      double vComp[3];
      const double *transformY = NULL;

      ColorTransformGeneric::setYConversion(inp->m_colorPrimaries, &transformY);
      
      for (int index = 0; index < inp->m_compSize[Y_COMP]; index++) {
        for(int component = 0; component < 3; component++)  {
          vComp[component] = inp->m_floatComp[component][index] / m_tfScale;
        }
        
        double ySignal = dMax(0.000001, transformY[R_COMP] * vComp[R_COMP] + transformY[G_COMP] * vComp[G_COMP] + transformY[B_COMP] * vComp[B_COMP]);
        double yDisplay = m_linScale * pow(ySignal, m_gamma) / ySignal;
        
        for(int component = 0; component < 3; component++)  {
          out->m_floatComp[component][index] = (float) (vComp[component] * yDisplay) ;
        }      
      }
      // reset the pointer (just for safety
      transformY = NULL;
    }
    else {
      out->copy((Frame *) inp);      
    }
  }
  else if (inp->m_isFloat == FALSE && out->m_isFloat == FALSE && inp->m_size == out->m_size && inp->m_bitDepth == out->m_bitDepth) {
    out->copy((Frame *) inp);
  }
}

void DisplayGammaAdjustHLG::inverse(Frame *frame)
{
  // 1. reverse any "burnt-in" system gamma to remove any display reference and leave only scene referred linear light
  // 2. scale RGB signals by a factor of m_tfscale (12 if not using super-white, otherwise 19.68)
  // 3. apply OETF (signals now in the range of 0.0 to 1.00/1.09 (i.e. super white)
  // Step 3 is not performed in this process, but instead it is done as part of the TransferFunction Class.
  if (frame->m_isFloat == TRUE && frame->m_compSize[Y_COMP] == frame->m_compSize[Cb_COMP])  {    
    if (frame->m_colorSpace == CM_RGB) {
      double vComp[3];
      const double *transformY = NULL;

      ColorTransformGeneric::setYConversion(frame->m_colorPrimaries, &transformY);
      
      for (int index = 0; index < frame->m_compSize[Y_COMP]; index++) {
        for(int component = 0; component < 3; component++) {
          vComp[component] = dMax(0.0, (double) frame->m_floatComp[component][index] / m_linScale);
        }
                
        double yDisplay = dMax(0.000001, transformY[R_COMP] * vComp[R_COMP] + transformY[G_COMP] * vComp[G_COMP] + transformY[B_COMP] * vComp[B_COMP]);
        double yDisplayGamma = m_tfScale * pow(yDisplay,(1.0 - m_gamma) / m_gamma);
        
        for(int component = 0; component < 3; component++)  {
          frame->m_floatComp[component][index] = (float) (vComp[component] * yDisplayGamma) ;
        }
      }
      // reset the pointer (just for safety
      transformY = NULL;
    }
  }
}

void DisplayGammaAdjustHLG::inverse(Frame *out, const Frame *inp)
{
  // 1. reverse any "burnt-in" system gamma to remove any display reference and leave only scene referred linear light
  // 2. scale RGB signals by a factor of m_tfscale (12 if not using super-white, otherwise 19.68)
  // 3. apply OETF (signals now in the range of 0.0 to 1.00/1.09 (i.e. super white)
  // Step 3 is not performed in this process, but instead it is done as part of the TransferFunction Class.
  
  if (inp->m_isFloat == TRUE && out->m_isFloat == TRUE && inp->m_size == out->m_size && inp->m_compSize[Y_COMP] == inp->m_compSize[Cb_COMP])  {    
    if (inp->m_colorSpace == CM_RGB && out->m_colorSpace == CM_RGB) {
      double vComp[3];
      const double *transformY = NULL;
      ColorTransformGeneric::setYConversion(inp->m_colorPrimaries, &transformY);
      
      for (int index = 0; index < inp->m_compSize[Y_COMP]; index++) {
        for(int component = 0; component < 3; component++) {
          vComp[component] = dMax(0.0, (double) inp->m_floatComp[component][index] / m_linScale);
        }
        
        double yDisplay = dMax(0.000001, transformY[R_COMP] * vComp[R_COMP] + transformY[G_COMP] * vComp[G_COMP] + transformY[B_COMP] * vComp[B_COMP]);
        double yDisplayGamma = m_tfScale * pow(yDisplay,(1.0 - m_gamma) / m_gamma);
        
        for(int component = 0; component < 3; component++)  {
          out->m_floatComp[component][index] = (float) (vComp[component] * yDisplayGamma) ;
        }
      }
      // reset the pointer (just for safety
      transformY = NULL;
    }
  }
  else if (inp->m_isFloat == FALSE && out->m_isFloat == FALSE && inp->m_size == out->m_size && inp->m_bitDepth == out->m_bitDepth) {
    out->copy((Frame *) inp);
  }
}


//-----------------------------------------------------------------------------
// End of file
//-----------------------------------------------------------------------------

