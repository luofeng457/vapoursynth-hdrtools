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
 * \file TransferFunctionHPQ.cpp
 *
 * \brief
 *    TransferFunctionHPQ (Hybrid PQ) Class
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
#include "TransferFunctionHPQ.H"

//-----------------------------------------------------------------------------
// Macros
//-----------------------------------------------------------------------------

//-----------------------------------------------------------------------------
// Constructor/destructor
//-----------------------------------------------------------------------------

TransferFunctionHPQ::TransferFunctionHPQ() {
  init(1.0);
}

TransferFunctionHPQ::TransferFunctionHPQ(float normalFactor) {
  init(normalFactor);
}

TransferFunctionHPQ::~TransferFunctionHPQ() {
}

void TransferFunctionHPQ::init(float normalFactor) {
  m1 = (2610.0        ) / (4096.0 * 4.0);
  m2 = (2523.0 * 128.0) / 4096.0;
  c1 = (3424.0        ) / 4096.0;
  c2 = (2413.0 *  32.0) / 4096.0;
  c3 = (2392.0 *  32.0) / 4096.0;
  
  m_normalFactor = normalFactor;
  m_invNormalFactor = 1.0 / m_normalFactor;
  
  m_linear = 4.5;
  m_value0 = 0.00018;
  m_value1 = 0.01;
  m_weight = 1.099;
  m_offset = 0.099;
  m_gamma = 1.0 / 0.45;
  m_inverseGamma = 0.45;
  m_scale   = inversePQ(m_value1);
  m_iValue1 = m_scale;    
  m_iValue0 = (m_value0) * (m_scale * m_linear * 100.0);    
}
//-----------------------------------------------------------------------------
// Private methods
//-----------------------------------------------------------------------------

//-----------------------------------------------------------------------------
// Public methods
//-----------------------------------------------------------------------------
double TransferFunctionHPQ::forwardPQ(double value) {
    double tempValue = pow(value, (1.0 / m2));
    return (pow(dMax(0.0, (tempValue - c1))/(c2 - c3 * tempValue),(1.0 / m1)));
}

double TransferFunctionHPQ::inversePQ(double value) {
    double tempValue = pow((double) value, m1);
    return (float) (pow(((c2 * (tempValue) + c1)/(1.0 + c3 *(tempValue))), m2));
}


double TransferFunctionHPQ::forward(double value) {
  if (value < m_iValue0) {
    return (value / (m_scale * m_linear * 100.0));
  }
  else if (value < m_iValue1) {
    return ((pow((dMax(value / m_scale, 0.0) + m_offset) / m_weight, m_gamma)) * 0.01);
  }
  else {
    return forwardPQ(value);
  }
}

double TransferFunctionHPQ::inverse(double value) {
  if (value < m_value0) {
    return ((m_linear * value * 100.0) * m_scale);
  }
  else if (value < m_value1) {
    return ((m_weight * pow(value * 100.0, m_inverseGamma) - m_offset) * m_scale);
  }
  else {
    return inversePQ(value);
  }
}


void TransferFunctionHPQ::forward( Frame* out, const Frame *inp, int component ) {
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

void TransferFunctionHPQ::forward( Frame* out, const Frame *inp ) {
  out->m_frameNo = inp->m_frameNo;
  out->m_isAvailable = TRUE;
  
  if (m_normalFactor == 1.0) {
    if (inp->m_isFloat == TRUE && out->m_isFloat == TRUE && inp->m_size == out->m_size) {
      for (int i = 0; i < inp->m_size; i++) {
        out->m_floatData[i] = (float) forward((double) inp->m_floatData[i]);
      }
    }
    else if (inp->m_isFloat == FALSE && out->m_isFloat == FALSE && inp->m_size == out->m_size && inp->m_bitDepth == out->m_bitDepth) {
      out->copy((Frame *) inp);
    }
  }
  else {
    if (inp->m_isFloat == TRUE && out->m_isFloat == TRUE && inp->m_size == out->m_size) {
      for (int i = 0; i < inp->m_size; i++) {
      // ideally, we should remove the double cast. However, we are currently keeping compatibility with the old code
        //out->m_floatData[i] = (float) (m_normalFactor * (float) forward((double) inp->m_floatData[i]));
        //out->m_floatData[i] = (float) (m_normalFactor * forward((double) inp->m_floatData[i]));
        out->m_floatData[i] = (float) (m_normalFactor * (double) ((float) forward((double) inp->m_floatData[i])));
      }
    }
    else if (inp->m_isFloat == FALSE && out->m_isFloat == FALSE && inp->m_size == out->m_size && inp->m_bitDepth == out->m_bitDepth) {
      out->copy((Frame *) inp);
    }
  }
}

void TransferFunctionHPQ::inverse( Frame* out, const Frame *inp, int component ) {
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
        out->m_floatComp[component][i] = (float) inverse(dMax(0.0, dMin((double) inp->m_floatComp[component][i] / m_normalFactor, 1.0)));
      }
    }
    else if (inp->m_isFloat == FALSE && out->m_isFloat == FALSE && inp->m_size == out->m_size && inp->m_bitDepth == out->m_bitDepth) {
      out->copy((Frame *) inp, component);
    }
  }
}

void TransferFunctionHPQ::inverse( Frame* out, const Frame *inp ) {
  out->m_frameNo = inp->m_frameNo;
  out->m_isAvailable = TRUE;
  /*
  for (int i = 0; i < 30; i++) {
    double x = 1.0 / sqrt((double) (1 << i));
    
    printf("values (%d) %10.7f %10.7f %10.7f %10.7f %10.7f %10.7f \n", i, x, x * 10000.0, inverse(x), forward(inverse(x)), forward(inverse(x))*10000.0,  (forward(inverse(x)) - x) * 10000.0);
  }
  */
  
/*
  double maxError = 0.0;
  double valError = 0.0;
  for (double dx = 99; dx < 101.0; dx +=0.01) {
    double dv = dx / 10000;
    double convDV = inverse(dv);
    double currError = dAbs(forward(convDV) - dv) / dv;
    printf("%10.8f %10.8f %10.8f, %10.8f %10.8f\n", currError, dv, convDV, round(convDV * 876) / 876, forward(round(convDV * 876) / 876));

    if ( currError> maxError) {
      maxError = currError;
      valError = dv;
    }
  }
  printf("max error %10.8f %10.8f\n", maxError, valError);
  */
  if (m_normalFactor == 1.0) {
    if (inp->m_isFloat == TRUE && out->m_isFloat == TRUE && inp->m_size == out->m_size) {
      for (int i = 0; i < inp->m_size; i++) {
        out->m_floatData[i] = (float) inverse((double) inp->m_floatData[i]);
      }
    }
    else if (inp->m_isFloat == FALSE && out->m_isFloat == FALSE && inp->m_size == out->m_size && inp->m_bitDepth == out->m_bitDepth) {
      out->copy((Frame *) inp);
    }
  }
  else {
    if (inp->m_isFloat == TRUE && out->m_isFloat == TRUE && inp->m_size == out->m_size) {
      for (int i = 0; i < inp->m_size; i++) {
        out->m_floatData[i] = (float) inverse(dMax(0.0, dMin((double) inp->m_floatData[i] / m_normalFactor, 1.0)));
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
