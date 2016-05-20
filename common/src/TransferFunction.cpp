/* The copyright in this software is being made available under the BSD
 * License, included below. This software may be subject to other third party
 * and contributor rights, including patent rights, and no such rights are
 * granted under this license.
 *
 * <OWNER> = Apple Inc.
 * <ORGANIZATION> = Apple Inc.
 * <YEAR> = 2014-2016
 *
 * Copyright (c) 2014-2016, Apple Inc.
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
 * \file TransferFunction.cpp
 *
 * \brief
 *    Base Class for transfer function application
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
#include "TransferFunction.H"
#include "TransferFunctionNull.H"
#include "TransferFunctionPQ.H"
#include "TransferFunctionAPQ.H"
#include "TransferFunctionAPH.H"
#include "TransferFunctionAPQScaled.H"
#include "TransferFunctionMPQ.H"
#include "TransferFunctionAMPQ.H"
#include "TransferFunctionPH.H"
#include "TransferFunctionHG.H"
#include "TransferFunctionHLG.H"
#include "TransferFunctionNormalize.H"
#include "TransferFunctionPower.H"
#include "TransferFunctionBiasedMPQ.H"
#include "TransferFunctionHPQ.H"
#include "TransferFunctionHPQ2.H"
#ifdef __SIM2_SUPPORT_ENABLED__
#include "TransferFunctionSim2.H"
#endif
#include <time.h>


//-----------------------------------------------------------------------------
// Private methods
//-----------------------------------------------------------------------------
void TransferFunction::initLUT() {  
  if (m_enableLUT == FALSE) {
    m_binsLUT = 0;
  }
  else {
    printf("Initializing LUTs for TF computations\n");
    uint32 i, j;
    m_binsLUT = 10;
    m_elementsLUT.resize    (m_binsLUT);
    m_multiplierLUT.resize  (m_binsLUT);
    m_boundLUT.resize       (m_binsLUT + 1);
    m_invTransformLUT.resize(m_binsLUT);
    m_fwdTransformLUT.resize(m_binsLUT);
    m_boundLUT[0] = 0.0;
    for (i = 0; i < m_binsLUT; i++) {
      m_elementsLUT[i] = 10000; // Size of each bin. 
                                // Could be different for each bin, but for now lets set this to be the same.
      m_boundLUT[i + 1] = 1 / pow( 10.0 , (double) (m_binsLUT - i - 1)); // upper bin boundary
      double stepSize = (m_boundLUT[i + 1] -  m_boundLUT[i]) / (m_elementsLUT[i] - 1);
      m_multiplierLUT[i] = (double) (m_elementsLUT[i] - 1) / (m_boundLUT[i + 1] -  m_boundLUT[i]);
      
      // Now allocate memory given the specified size
      m_invTransformLUT[i].resize(m_elementsLUT[i]);
      m_fwdTransformLUT[i].resize(m_elementsLUT[i]);
      for (j = 0; j < m_elementsLUT[i] ; j++) {
        double curValue = m_boundLUT[i] + (double) j * stepSize;
        m_invTransformLUT[i][j] = inverse(curValue);
        m_fwdTransformLUT[i][j] = forward(curValue);
      }
    }
  }
}



double TransferFunction::inverseLUT(double value) {
  if (value <= 0.0)
    return m_invTransformLUT[0][0];
  else if (value >= 1.0) {
    // top value, most likely 1.0
    return m_invTransformLUT[m_binsLUT - 1][m_elementsLUT[m_binsLUT - 1] - 1];
  }
  else { // now search for value in the table
    for (uint32 i = 0; i < m_binsLUT; i++) {
      if (value < m_boundLUT[i + 1]) { // value located
        double satValue = (value - m_boundLUT[i]) * m_multiplierLUT[i];
        //return (m_invTransformMap[(int) dRound(satValue)]);
        int    valuePlus     = (int) dCeil(satValue) ;
        double distancePlus  = (double) valuePlus - satValue;
        return (m_invTransformLUT[i][valuePlus - 1] * distancePlus + m_invTransformLUT[i][valuePlus] * (1.0 - distancePlus));
      }
    }
  }
  return 0.0;
}

double TransferFunction::forwardLUT(double value) {
  if (value <= 0.0)
    return m_fwdTransformLUT[0][0];
  else if (value >= 1.0  ) {
    // top value, most likely 1.0
    return m_fwdTransformLUT[m_binsLUT - 1][m_elementsLUT[m_binsLUT - 1] - 1];
  }
  else {
    // now search for value in the table
    for (uint32 i = 0; i < m_binsLUT; i++) {
      if (value < m_boundLUT[i + 1]) { // value located
        double satValue = (value - m_boundLUT[i]) * m_multiplierLUT[i];
        //return (m_invTransformMap[(int) dRound(satValue)]);
        int    valuePlus     = (int) dCeil(satValue) ;
        double distancePlus  = (double) valuePlus - satValue;
        return (m_fwdTransformLUT[i][valuePlus - 1] * distancePlus + m_fwdTransformLUT[i][valuePlus] * (1.0 - distancePlus));
      }
    }
  }
  return 0.0;
}

//-----------------------------------------------------------------------------
// Public methods
//-----------------------------------------------------------------------------
TransferFunction *TransferFunction::create(int method, bool singleStep, float scale, float systemGamma, float minValue, float maxValue, bool enableLUT) {
  TransferFunction *result = NULL;
  
  if (singleStep == TRUE) {
    switch (method){
      case TF_NULL:
        result = new TransferFunctionNull();
        break;
      case TF_APH:
        result = new TransferFunctionAPH(scale, minValue, maxValue);
        break;
      case TF_APQ:
        result = new TransferFunctionAPQ(scale, minValue, maxValue);
        break;
      case TF_APQS:
        result = new TransferFunctionAPQScaled(scale, maxValue);
        break;
      case TF_PQ:
        result = new TransferFunctionPQ(scale);
        break;
      case TF_HPQ:
        result = new TransferFunctionHPQ(scale);
        break;
      case TF_HPQ2:
        result = new TransferFunctionHPQ2(scale);
        break;
      case TF_PH:
        result = new TransferFunctionPH(scale);
        break;
      case TF_HG:
        result = new TransferFunctionHG(systemGamma);
        break;
      case TF_HLG:
        result = new TransferFunctionHLG();
        break;
      case TF_NORMAL:
        result = new TransferFunctionNormalize(scale);
        break;
      case TF_POWER:
        result = new TransferFunctionPower(systemGamma, scale);
        break;
      case TF_MPQ:
        result = new TransferFunctionMPQ(0.1f, 1.5f, scale);
        break;
      case TF_AMPQ:
        result = new TransferFunctionAMPQ(0.1f, 1.5f, scale, minValue, maxValue);
        break;
      case TF_BiasedMPQ:
        result = new TransferFunctionBiasedMPQ(0.1f, 2.0f, scale);
        break;
#ifdef __SIM2_SUPPORT_ENABLED__
      case TF_SIM2:
        result = new TransferFunctionSim2();
        break;
#endif
      default:
        fprintf(stderr, "\nUnsupported Transfer Function %d\n", method);
        exit(EXIT_FAILURE);
    }
  }
  else {
    switch (method){
      case TF_NULL:
        result = new TransferFunctionNull();
        break;
      case TF_APH:
        result = new TransferFunctionAPH(minValue, maxValue);
        break;
      case TF_APQ:
        result = new TransferFunctionAPQ(minValue, maxValue);
        break;
      case TF_APQS:
        result = new TransferFunctionAPQScaled(maxValue);
        break;
      case TF_PQ:
        result = new TransferFunctionPQ();
        break;
      case TF_HPQ:
        result = new TransferFunctionHPQ();
        break;
      case TF_HPQ2:
        result = new TransferFunctionHPQ2();
        break;
      case TF_PH:
        result = new TransferFunctionPH();
        break;
      case TF_HG:
        result = new TransferFunctionHG(systemGamma);
        break;
      case TF_HLG:
        result = new TransferFunctionHLG();
        break;
      case TF_NORMAL:
        result = new TransferFunctionNormalize(scale);
        break;
      case TF_POWER:
        result = new TransferFunctionPower(systemGamma, scale);
        break;
      case TF_MPQ:
        result = new TransferFunctionMPQ(0.00001f, 1.5f);
        break;
      case TF_AMPQ:
        result = new TransferFunctionAMPQ(0.00001f, 1.5f, minValue, maxValue);
        break;
      case TF_BiasedMPQ:
        result = new TransferFunctionBiasedMPQ(0.00001f, 2.0f);
        break;
#ifdef __SIM2_SUPPORT_ENABLED__
      case TF_SIM2:
        result = new TransferFunctionSim2();
        break;
#endif
      default:
        fprintf(stderr, "\nUnsupported Transfer Function %d\n", method);
        exit(EXIT_FAILURE);
    }
  }
  
  result->m_enableLUT = enableLUT;
  
  result->initLUT();
  
  return result;
}


double TransferFunction::forwardDerivative(double value)
{
  double low  = value - DERIV_STEP;
  double high = value + DERIV_STEP;
  low  = ( low  > DERIV_LOWER_BOUND  ) ? low  : DERIV_LOWER_BOUND;
  high = ( high < DERIV_HIGHER_BOUND ) ? high : DERIV_HIGHER_BOUND;
  
  return (getForward(high) - getForward(low)) / (high - low) ;
}

double TransferFunction::inverseDerivative(double value)
{
  double low  = value - DERIV_STEP;
  double high = value + DERIV_STEP;
  low  = ( low  > DERIV_LOWER_BOUND  ) ? low  : DERIV_LOWER_BOUND;
  high = ( high < DERIV_HIGHER_BOUND ) ? high : DERIV_HIGHER_BOUND;
  
  return (getInverse(high) - getInverse(low)) / (high - low) ;
}


double TransferFunction::getForward(double value) {
  if (m_enableLUT == FALSE)
    return forward(value);
  else 
    return forwardLUT(value);
}

double TransferFunction::getInverse(double value) {
  if (m_enableLUT == FALSE)
    return inverse(value);
  else 
    return inverseLUT(value);
}


void TransferFunction::forward( Frame* frame, int component ) {
  if (frame->m_isFloat == TRUE) {
    if (m_enableLUT == FALSE) {
      for (int i = 0; i < frame->m_compSize[component]; i++) {
        frame->m_floatComp[component][i] = (float) forward((double) frame->m_floatComp[component][i]);
      }
    }
    else {
      for (int i = 0; i < frame->m_compSize[component]; i++) {
        frame->m_floatComp[component][i] = (float) forwardLUT((double) frame->m_floatComp[component][i]);
      }      
    }
  }
}

void TransferFunction::forward( Frame* frame ) {  
  if (frame->m_isFloat) {
    if (m_enableLUT == FALSE) {
      for (int i = 0; i < frame->m_size; i++) {
        frame->m_floatData[i] = (float) forward((double) frame->m_floatData[i]);
      }
    }
    else {
      printf("in here\n");
      for (int i = 0; i < frame->m_size; i++) {
        frame->m_floatData[i] = (float) forwardLUT((double) frame->m_floatData[i]);
      }      
    }
  }
}

void TransferFunction::inverse( Frame* frame, int component ) {
  if (frame->m_isFloat == TRUE) {
    if (m_enableLUT == FALSE) {
      for (int i = 0; i < frame->m_compSize[component]; i++) {
        frame->m_floatComp[component][i] = (float) inverse((double) frame->m_floatComp[component][i]);
      }
    }
    else {
      for (int i = 0; i < frame->m_compSize[component]; i++) {
        frame->m_floatComp[component][i] = (float) inverseLUT((double) frame->m_floatComp[component][i]);
      }      
    }
  }
}

void TransferFunction::inverse( Frame* frame ) {
  if (frame->m_isFloat) {
    if (m_enableLUT == FALSE) {
      for (int i = 0; i < frame->m_size; i++) {
        frame->m_floatData[i] = (float) inverse((double) frame->m_floatData[i]);
      }
    }
    else {
      for (int i = 0; i < frame->m_size; i++) {
        frame->m_floatData[i] = (float) inverseLUT((double) frame->m_floatData[i]);
      }      
    }
  }
}


void TransferFunction::forward( Frame* out, const Frame *inp, int component ) {
  // In this scenario, we should likely copy the frame number externally
  if (m_normalFactor == 1.0) {
    if (inp->m_isFloat == TRUE && out->m_isFloat == TRUE && inp->m_compSize[component] == out->m_compSize[component]) {
      for (int i = 0; i < inp->m_compSize[component]; i++) {
        out->m_floatComp[component][i] = (float) forward((double) inp->m_floatComp[component][i]);
      }
    }
    else if (inp->m_isFloat == FALSE && out->m_isFloat == FALSE && inp->m_size == out->m_size && inp->m_bitDepth == out->m_bitDepth) {
      out->copy((Frame *) inp, component);
    }
  }
  else {
    if (inp->m_isFloat == TRUE && out->m_isFloat == TRUE && inp->m_compSize[component] == out->m_compSize[component]) {
      for (int i = 0; i < inp->m_compSize[component]; i++) {
        out->m_floatComp[component][i] = (float) (m_normalFactor * forward((double) inp->m_floatComp[component][i]));
      }
    }
    else if (inp->m_isFloat == FALSE && out->m_isFloat == FALSE && inp->m_size == out->m_size && inp->m_bitDepth == out->m_bitDepth) {
      
      out->copy((Frame *) inp, component);
    }
  }
}

void TransferFunction::forward( Frame* out, const Frame *inp ) {
  out->m_frameNo = inp->m_frameNo;
  out->m_isAvailable = TRUE;
  
  if (m_normalFactor == 1.0) {
    if (inp->m_isFloat == TRUE && out->m_isFloat == TRUE && inp->m_size == out->m_size) {
      if (m_enableLUT == FALSE) {
        for (int i = 0; i < inp->m_size; i++) {
          out->m_floatData[i] = (float) forward((double) inp->m_floatData[i]);
        }
      }
      else {
        for (int i = 0; i < inp->m_size; i++) {
          out->m_floatData[i] = (float) forwardLUT((double) inp->m_floatData[i]);
        }        
      }
    }
    else if (inp->m_isFloat == FALSE && out->m_isFloat == FALSE && inp->m_size == out->m_size && inp->m_bitDepth == out->m_bitDepth) {
      out->copy((Frame *) inp);
    }
  }
  else {
    if (inp->m_isFloat == TRUE && out->m_isFloat == TRUE && inp->m_size == out->m_size) {
      if (m_enableLUT == FALSE) {
        for (int i = 0; i < inp->m_size; i++) {
          // ideally, we should remove the double cast. However, we are currently keeping compatibility with the old code
          //out->m_floatData[i] = (float) (m_normalFactor * forward((double) inp->m_floatData[i]));
          out->m_floatData[i] = (float) (m_normalFactor * (double) ((float) forward((double) inp->m_floatData[i])));
        }
      }
      else {
        for (int i = 0; i < inp->m_size; i++) {
          out->m_floatData[i] = (float) (m_normalFactor * (double) forwardLUT((double) inp->m_floatData[i]));
        }        
      }
    }
    else if (inp->m_isFloat == FALSE && out->m_isFloat == FALSE && inp->m_size == out->m_size && inp->m_bitDepth == out->m_bitDepth) {
      out->copy((Frame *) inp);
    }
  }
}

void TransferFunction::inverse( Frame* out, const Frame *inp, int component ) {
  // In this scenario, we should likely copy the frame number externally
  if (m_normalFactor == 1.0) {
    if (inp->m_isFloat == TRUE && out->m_isFloat == TRUE && inp->m_compSize[component] == out->m_compSize[component]) {
      for (int i = 0; i < inp->m_compSize[component]; i++) {
        out->m_floatComp[component][i] = (float) inverse((double) inp->m_floatComp[component][i]);
      }
    }
    else if (inp->m_isFloat == FALSE && out->m_isFloat == FALSE && inp->m_size == out->m_size && inp->m_bitDepth == out->m_bitDepth) {
      out->copy((Frame *) inp, component);
    }
  }
  else {
    if (inp->m_isFloat == TRUE && out->m_isFloat == TRUE && inp->m_compSize[component] == out->m_compSize[component]) {
      for (int i = 0; i < inp->m_compSize[component]; i++) {
        out->m_floatComp[component][i] = (float) inverse((double) inp->m_floatComp[component][i] / m_normalFactor);
      }
    }
    else if (inp->m_isFloat == FALSE && out->m_isFloat == FALSE && inp->m_size == out->m_size && inp->m_bitDepth == out->m_bitDepth) {
      out->copy((Frame *) inp, component);
    }
  }
}

void TransferFunction::inverse( Frame* out, const Frame *inp ) {
  out->m_frameNo = inp->m_frameNo;
  out->m_isAvailable = TRUE;
  
  if (m_normalFactor == 1.0) {
    if (inp->m_isFloat == TRUE && out->m_isFloat == TRUE && inp->m_size == out->m_size) {
      if (m_enableLUT == FALSE) {
        for (int i = 0; i < inp->m_size; i++) {
          out->m_floatData[i] = (float) inverse((double) inp->m_floatData[i]);
        }
      }
      else {
        for (int i = 0; i < inp->m_size; i++) {
          out->m_floatData[i] = (float) inverseLUT((double) inp->m_floatData[i]);
        }        
      }
    }
    else if (inp->m_isFloat == FALSE && out->m_isFloat == FALSE && inp->m_size == out->m_size && inp->m_bitDepth == out->m_bitDepth) {
      out->copy((Frame *) inp);
    }
  }
  else {
    if (inp->m_isFloat == TRUE && out->m_isFloat == TRUE && inp->m_size == out->m_size) {
      if (m_enableLUT == FALSE) {
        for (int i = 0; i < inp->m_size; i++) {
          out->m_floatData[i] = (float) inverse((double) inp->m_floatData[i] / m_normalFactor);
        }
      }
      else {
        for (int i = 0; i < inp->m_size; i++) {
          out->m_floatData[i] = (float) inverseLUT((double) inp->m_floatData[i] / m_normalFactor);
        }        
      }
    }
    else if (inp->m_isFloat == FALSE && out->m_isFloat == FALSE && inp->m_size == out->m_size && inp->m_bitDepth == out->m_bitDepth) {
      out->copy((Frame *) inp);
    }
  }
}

//-----------------------------------------------------------------------------
// End of file
//-----------------------------------------------------------------------------
