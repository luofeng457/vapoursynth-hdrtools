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
 * \file DistortionTransferFunction.cpp
 *
 * \brief
 *    Distortion Transfer Function Class
 *
 * \author
 *     - Alexis Michael Tourapis         <atourapis@apple.com>
 *
 *************************************************************************************
 */

//-----------------------------------------------------------------------------
// Include headers
//-----------------------------------------------------------------------------

#include "DistortionTransferFunction.H"
//#include "ColorTransformGeneric.H"

//-----------------------------------------------------------------------------
// Macros
//-----------------------------------------------------------------------------

//-----------------------------------------------------------------------------
// Constructor/destructor
//-----------------------------------------------------------------------------

DistortionTransferFunction *DistortionTransferFunction::create(int method) {
  DistortionTransferFunction *result = NULL;
  
  switch (method){
    default:
    case 0:
      result = new DistortionTransferFunctionCombo();
      break;
    case 1:
      result = new DistortionTransferFunctionPQ();
      break;
    case 2:
      result = new DistortionTransferFunctionDE();
      break;
    case 3:
      result = new DistortionTransferFunctionPQNoise();
      break;
    case 4:
      result = new DistortionTransferFunctionNull();
      break;
  }
  return result;
}


double DistortionTransferFunction::computeNull(double value) {
  return (value);
}

double DistortionTransferFunction::computeDE(double value) {
  value *= 100.0;
  return ((value >= 0.008856) ? pow(value, 1.0 / 3.0) : (7.78704 * value + 0.137931)) / pow(100.0, 1.0/3.0);
}

double DistortionTransferFunction::computePQ(double value) {
  static const double m1 = (2610.0        ) / 16384.0;
  static const double m2 = (2523.0 * 128.0) /  4096.0;
  static const double c1 = (3424.0        ) /  4096.0;
  static const double c2 = (2413.0 *  32.0) /  4096.0;
  static const double c3 = (2392.0 *  32.0) /  4096.0;
  
  double tempValue = pow(dMax(0.0, dMin(value, 1.0)), m1);
  return (pow(((c2 *(tempValue) + c1)/(1 + c3 *(tempValue))), m2));
}

double DistortionTransferFunction::computePQNoise(double value) {  
  static const double minLimit = 0.0001;
  if (value < minLimit) // use a 
    return (4.5 * value);
  else {
    static const double pqLimit = computePQ(minLimit);
    static const double linLimit = 4.5 * minLimit;
    double tempValue = (((computePQ(value) - pqLimit) / ( 1.0 - pqLimit)) * (1.0 - linLimit)) + linLimit;
    return (tempValue);
  }
}

double DistortionTransferFunction::computePH10K(double value) {
  static const double rho   = 25.0;
  static const double invGamma = 1 / 2.4;
  static const double ratio = 2.0; // Philips EOTF was created with a max of 5K cd/m^2. With a ratio of 2 we now extend this to 10K.
  static const double maxValue = (log(1.0 + (rho - 1.0) * pow(ratio, invGamma)) / log(rho));
  
  return (log(1.0 + (rho - 1.0) * pow(ratio * value, invGamma)) / (log(rho) * maxValue));
}

double DistortionTransferFunctionNull::compute(double value) {
  
  double clippedValue = dMax(0.0, dMin(value, 1.0));
  
  return computeNull(clippedValue);
}


double DistortionTransferFunctionDE::compute(double value) {
  
  double clippedValue = dMax(0.0, dMin(value, 1.0));
  
  return computeDE(clippedValue);
}


double DistortionTransferFunctionPQ::compute(double value) {
  
  double clippedValue = dMax(0.0, dMin(value, 1.0));
  
  return computePQ(clippedValue);
}


double DistortionTransferFunctionPQNoise::compute(double value) {
  
  double clippedValue = dMax(0.0, dMin(value, 1.0));
  
  return computePQNoise(clippedValue);
}

double DistortionTransferFunctionCombo::compute(double value) {
  double clippedValue = dMax(0.0, dMin(value, 1.0));
  
  return (computePQ(clippedValue) + computePH10K(clippedValue)) * 0.5;
}

//-----------------------------------------------------------------------------
// End of file
//-----------------------------------------------------------------------------
