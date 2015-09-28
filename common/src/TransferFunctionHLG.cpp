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
 * \file TransferFunctionHLG.cpp
 *
 * \brief
 *    TransferFunctionHLG class implementation for the BBC Hybrid Log-Gamma 
 *    Transfer Function (HLG). 
 *    A complete documentation of this transfer function is available in 
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
#include "TransferFunctionHLG.H"


//-----------------------------------------------------------------------------
// Constructor / destructor implementation
//-----------------------------------------------------------------------------

TransferFunctionHLG::TransferFunctionHLG()
{
  m_a        = 0.17883277;
  m_b        = 0.28466892;
  m_c        = 0.55991073;
}


TransferFunctionHLG::~TransferFunctionHLG()
{
}

double TransferFunctionHLG::forward(double value) {
  return (value < 0.5 ? (value * value * 4.0) : (exp((value - m_c) / m_a) + m_b));
}

double TransferFunctionHLG::inverse(double value) {
  return (value < 1.0 ? 0.5 * sqrt(value) : m_a * log(value - m_b) + m_c);
}

void TransferFunctionHLG::forward(Frame *out, const Frame *inp)
{
  if (inp->m_isFloat == TRUE && out->m_isFloat == TRUE && inp->m_size == out->m_size)  {
    for (int index = 0; index < inp->m_size; index++) {
      out->m_floatData[index] = (float) forward((double) inp->m_floatData[index]);
    }
  }
  else if (inp->m_isFloat == FALSE && out->m_isFloat == FALSE && inp->m_size == out->m_size && inp->m_bitDepth == out->m_bitDepth) {
    out->copy((Frame *) inp);
  }
}

void TransferFunctionHLG::forward(Frame *out, const Frame *inp, int component)
{
  if (inp->m_isFloat == TRUE && out->m_isFloat == TRUE && inp->m_compSize[component] == out->m_compSize[component]) {
    for (int index = 0; index < inp->m_compSize[component]; index++) {
      out->m_floatComp[component][index] = (float) forward((double) inp->m_floatComp[component][index]);
    }
  }
  else if (inp->m_isFloat == FALSE && out->m_isFloat == FALSE && inp->m_size == out->m_size && inp->m_bitDepth == out->m_bitDepth) {
    out->copy((Frame *) inp, component);
  }
}

void TransferFunctionHLG::inverse(Frame *out, const Frame *inp)
{
  if (inp->m_isFloat == TRUE && out->m_isFloat == TRUE && inp->m_size == out->m_size)  {
    for (int index = 0; index < inp->m_size; index++) {
      out->m_floatData[index] = (float) inverse((double) inp->m_floatData[index]);
    }
  }
  else if (inp->m_isFloat == FALSE && out->m_isFloat == FALSE && inp->m_size == out->m_size && inp->m_bitDepth == out->m_bitDepth) {
    out->copy((Frame *) inp);
  }
}

void TransferFunctionHLG::inverse(Frame *out, const Frame *inp, int component)
{
  if (inp->m_isFloat == TRUE && out->m_isFloat == TRUE && inp->m_compSize[component] == out->m_compSize[component]) {
    for (int index = 0; index < inp->m_compSize[component]; index++) {
      out->m_floatComp[component][index] = (float) inverse((double) inp->m_floatComp[component][index]);
    }
  }
  else if (inp->m_isFloat == FALSE && out->m_isFloat == FALSE && inp->m_size == out->m_size && inp->m_bitDepth == out->m_bitDepth) {
    out->copy((Frame *) inp, component);
  }
}

//-----------------------------------------------------------------------------
// End of file
//-----------------------------------------------------------------------------

