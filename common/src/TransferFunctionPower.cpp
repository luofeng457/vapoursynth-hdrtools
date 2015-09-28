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
 * \file TransferFunctionPower.cpp
 *
 * \brief
 *    TransferFunctionPower Class
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
#include "TransferFunctionPower.H"

//-----------------------------------------------------------------------------
// Macros
//-----------------------------------------------------------------------------

//-----------------------------------------------------------------------------
// Constructor/destructor
//-----------------------------------------------------------------------------


TransferFunctionPower::TransferFunctionPower(double gamma, double normalScale) {
  m_Lw = normalScale;
  m_Lb = 0.0;
  m_gamma = gamma;
  m_inverseGamma = 1.0 / gamma;
  m_alpha = pow(pow(m_Lw,m_inverseGamma) - pow(m_Lb,m_inverseGamma), m_gamma);
  m_beta  = pow(m_Lb, m_inverseGamma)/(pow(m_Lw,m_inverseGamma) - pow(m_Lb,m_inverseGamma));
}

TransferFunctionPower::~TransferFunctionPower() {
}

//-----------------------------------------------------------------------------
// Private methods
//-----------------------------------------------------------------------------

//-----------------------------------------------------------------------------
// Public methods
//-----------------------------------------------------------------------------
double TransferFunctionPower::forward(double value) {
  return (m_alpha * pow(dMax(value + m_beta, 0.0), m_gamma));
}

double TransferFunctionPower::inverse(double value) {
  return (pow(value / m_alpha, m_inverseGamma) - m_beta);
}

void TransferFunctionPower::forward( Frame* out, const Frame *inp, int component ) {
  if (inp->m_isFloat == TRUE && out->m_isFloat == TRUE && inp->m_compSize[component] == out->m_compSize[component]) {
    double v;
    for (int index = 0; index < inp->m_compSize[component]; index++) {
      v = (double) inp->m_floatComp[component][index];
      out->m_floatComp[component][index] = (float) (m_alpha * pow(dMax(v + m_beta, 0.0), m_gamma));
    }
  }
  else if (inp->m_isFloat == FALSE && out->m_isFloat == FALSE && inp->m_size == out->m_size && inp->m_bitDepth == out->m_bitDepth) {
    out->copy((Frame *) inp, component);
  }
}

void TransferFunctionPower::forward( Frame* out, const Frame *inp ) {
  out->m_frameNo = inp->m_frameNo;
  out->m_isAvailable = TRUE;

  if (inp->m_isFloat == TRUE && out->m_isFloat == TRUE && inp->m_size == out->m_size) {
    double v;
    for (int index = 0; index < inp->m_size; index++) {
      v = (double) inp->m_floatData[index];
      out->m_floatData[index] = (float) (m_alpha * pow(dMax(v + m_beta, 0.0), m_gamma));
    }
  }
  else if (inp->m_isFloat == FALSE && out->m_isFloat == FALSE && inp->m_size == out->m_size && inp->m_bitDepth == out->m_bitDepth) {
    out->copy((Frame *) inp);
  }
}

void TransferFunctionPower::inverse( Frame* out, const Frame *inp, int component ) {
  
  if (inp->m_isFloat == TRUE && out->m_isFloat == TRUE && inp->m_compSize[component] == out->m_compSize[component]) {
    double v;
    for (int index = 0; index < inp->m_compSize[component]; index++) {
      v = (double) inp->m_floatComp[component][index] / m_alpha;
      out->m_floatComp[component][index] = (float) (pow(v, m_inverseGamma) - m_beta);
    }
  }
  else if (inp->m_isFloat == FALSE && out->m_isFloat == FALSE && inp->m_size == out->m_size && inp->m_bitDepth == out->m_bitDepth) {
    out->copy((Frame *) inp, component);
  }
}

void TransferFunctionPower::inverse( Frame* out, const Frame *inp ) {
  out->m_frameNo = inp->m_frameNo;
  out->m_isAvailable = TRUE;

  if (inp->m_isFloat == TRUE && out->m_isFloat == TRUE && inp->m_size == out->m_size) {
    double v;
    for (int index = 0; index < inp->m_size; index++) {
      v = (double) inp->m_floatData[index]/ m_alpha;
      out->m_floatData[index] = (float) (pow(v, m_inverseGamma) - m_beta);
    }
  }
  else if (inp->m_isFloat == FALSE && out->m_isFloat == FALSE && inp->m_size == out->m_size && inp->m_bitDepth == out->m_bitDepth) {
    out->copy((Frame *) inp);
  }
}

//-----------------------------------------------------------------------------
// End of file
//-----------------------------------------------------------------------------
