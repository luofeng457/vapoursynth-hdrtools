/* The copyright in this software is being made available under the BSD
 * License, included below. This software may be subject to other third party
 * and contributor rights, including patent rights, and no such rights are
 * granted under this license.
 *
 * <OWNER> = British Broadcasting Corporation (BBC).
 * <ORGANIZATION> = BBC.
 * <YEAR> = 2014
 *
 * Copyright (c) 2014, BBC.
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
 * \file TransferFunctionHGold.cpp
 *
 * \brief
 *    TransferFunctionHGold class implementation for the BBC Hybrid Gamma (HG) Transfer Function
 *
 * \author
 *     - Matteo Naccari         <matteo.naccari@bbc.co.uk>
 *
 *************************************************************************************
 */

#include "Global.H"
#include "TransferFunctionHG.H"

//-----------------------------------------------------------------------------
// Constructor / destructor implementation
//-----------------------------------------------------------------------------

TransferFunctionHG::TransferFunctionHG(double gamma)
{
  m_mu    = 0.139401137752;
  m_eta   = sqrt(m_mu)/2.0;
  m_rho   = sqrt(m_mu)*(1.0-log(sqrt(m_mu)));
  m_xi    = sqrt(m_mu);
  m_gamma = gamma;
  
  m_normalFactor = 1.0;
}

TransferFunctionHG::TransferFunctionHG(double gamma, double normalFactor)
{
  m_mu    = 0.139401137752;
  m_eta   = sqrt(m_mu)/2.0;
  m_rho   = sqrt(m_mu)*(1.0-log(sqrt(m_mu)));
  m_xi    = sqrt(m_mu);
  m_gamma = gamma;
  
  m_normalFactor = (double) normalFactor;
}

TransferFunctionHG::~TransferFunctionHG()
{
}

double TransferFunctionHG::forward(double value) {
  return (value <= m_xi ? pow(value, 2.0 * m_gamma) : exp(m_gamma * (value - m_rho) / m_eta));
}

double TransferFunctionHG::inverse(double value) {
  return (value <= m_mu ? pow(value, 0.5) : m_eta * log(value) + m_rho);
}

void TransferFunctionHG::forward(Frame *out, const Frame *inp)
{
  if (inp->m_isFloat == TRUE && out->m_isFloat == TRUE && inp->m_size == out->m_size)  {
    if (m_normalFactor == 1.0) {
      for (int index = 0; index < inp->m_size; index++) {
        out->m_floatData[index] = (float) forward((double) inp->m_floatData[index]);
      }
    }
    else {
      for (int index = 0; index < inp->m_size; index++) {
        out->m_floatData[index] = (float) forward((double) inp->m_floatData[index] * m_normalFactor);
      }
    }
  }
  else if (inp->m_isFloat == FALSE && out->m_isFloat == FALSE && inp->m_size == out->m_size && inp->m_bitDepth == out->m_bitDepth) {
    out->copy((Frame *) inp);
  }
}

void TransferFunctionHG::forward(Frame *out, const Frame *inp, int component)
{
  if (inp->m_isFloat == TRUE && out->m_isFloat == TRUE && inp->m_compSize[component] == out->m_compSize[component]) {
    if (m_normalFactor == 1.0) {
      for (int index = 0; index < inp->m_compSize[component]; index++) {
        out->m_floatComp[component][index] = (float) forward((double) inp->m_floatComp[component][index]);
      }
    }
    else {
      for (int index = 0; index < inp->m_compSize[component]; index++) {
        out->m_floatComp[component][index] = (float) forward((double) inp->m_floatComp[component][index] * m_normalFactor);
      }      
    }
  }
  else if (inp->m_isFloat == FALSE && out->m_isFloat == FALSE && inp->m_size == out->m_size && inp->m_bitDepth == out->m_bitDepth) {
    out->copy((Frame *) inp, component);
  }
}

void TransferFunctionHG::inverse(Frame *out, const Frame *inp)
{
  if (inp->m_isFloat == TRUE && out->m_isFloat == TRUE && inp->m_size == out->m_size)  {
    if (m_normalFactor == 1.0) {
      for (int index = 0; index < inp->m_size; index++) {
        out->m_floatData[index] = (float) inverse((double) inp->m_floatData[index]);
      }
    }
    else {
      for (int index = 0; index < inp->m_size; index++) {
        out->m_floatData[index] = (float) inverse((double) inp->m_floatData[index] / m_normalFactor);
      }      
    }
  }
  else if (inp->m_isFloat == FALSE && out->m_isFloat == FALSE && inp->m_size == out->m_size && inp->m_bitDepth == out->m_bitDepth) {
    out->copy((Frame *) inp);
  }
}

void TransferFunctionHG::inverse(Frame *out, const Frame *inp, int component)
{
  if (inp->m_isFloat == TRUE && out->m_isFloat == TRUE && inp->m_compSize[component] == out->m_compSize[component]) {
    if (m_normalFactor == 1.0) {
      for (int index = 0; index < inp->m_compSize[component]; index++) {
        out->m_floatComp[component][index] = (float) inverse((double) inp->m_floatComp[component][index]);
      }
    }
    else {
      for (int index = 0; index < inp->m_compSize[component]; index++) {
        out->m_floatComp[component][index] = (float) inverse((double) inp->m_floatComp[component][index] / m_normalFactor);
      }      
    }
  }
  else if (inp->m_isFloat == FALSE && out->m_isFloat == FALSE && inp->m_size == out->m_size && inp->m_bitDepth == out->m_bitDepth) {
    out->copy((Frame *) inp, component);
  }
}

//-----------------------------------------------------------------------------
// End of file
//-----------------------------------------------------------------------------

