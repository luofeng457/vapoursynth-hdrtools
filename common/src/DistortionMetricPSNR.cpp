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
 * \file DistortionMetricPSNR.cpp
 *
 * \brief
 *    PSNR distortion computation Class
 *
 * \author
 *     - Alexis Michael Tourapis         <atourapis@apple.com>
 *
 *************************************************************************************
 */

//-----------------------------------------------------------------------------
// Include headers
//-----------------------------------------------------------------------------

#include "DistortionMetricPSNR.H"

//-----------------------------------------------------------------------------
// Macros
//-----------------------------------------------------------------------------

//-----------------------------------------------------------------------------
// Constructor/destructor
//-----------------------------------------------------------------------------

DistortionMetricPSNR::DistortionMetricPSNR(const FrameFormat *format, bool enableShowMSE, double maxSampleValue)
 : DistortionMetric()
{
  m_colorSpace    = format->m_colorSpace;
  m_enableShowMSE = enableShowMSE;
  for (int c = 0; c < T_COMP; c++) {
    m_mse[c] = 0.0;
    m_sse[c] = 0.0;
    m_mseStats[c].reset();
    m_sseStats[c].reset();
    m_maxValue[c] = (double) maxSampleValue;
  }
}

DistortionMetricPSNR::~DistortionMetricPSNR()
{
}

//-----------------------------------------------------------------------------
// Private methods (non-MMX)
//-----------------------------------------------------------------------------
double DistortionMetricPSNR::compute(const float *iComp0, const float *iComp1, int size, double maxValue)
{
  double diff = 0.0;
  double sum = 0.0;
  if (m_clipInputValues) { // Add clipping of distortion values as per Jacob's request
    for (int i = 0; i < size; i++) {
      diff = (dMin(iComp0[i], maxValue) - dMin(iComp1[i], maxValue));
      sum += diff * diff;
    }
  }
  else {
    for (int i = 0; i < size; i++) {
      diff = (iComp0[i] - iComp1[i]);
      sum += diff * diff;
    }
  }
  return sum;
}

uint64 DistortionMetricPSNR::compute(const uint16 *iComp0, const uint16 *iComp1, int size)
{
  int32  diff = 0;
  uint64 sum = 0;
  for (int i = 0; i < size; i++) {
    diff = (iComp0[i] - iComp1[i]);
    sum += diff * diff;
  }
  return sum;
}

uint64 DistortionMetricPSNR::compute(const uint8 *iComp0, const uint8 *iComp1, int size)
{
  int32  diff = 0;
  uint64  sum = 0;
  for (int i = 0; i < size; i++) {
    diff = (iComp0[i] - iComp1[i]);
    sum += diff * diff;
  }
  return sum;
}

//-----------------------------------------------------------------------------
// Public methods
//-----------------------------------------------------------------------------

void DistortionMetricPSNR::computeMetric (Frame* inp0, Frame* inp1)
{
  // it is assumed here that the frames are of the same type
  if (inp0->equalType(inp1)) {
    if (inp0->m_isFloat == TRUE) {    // floating point data
      for (int c = Y_COMP; c < inp0->m_noComponents; c++) {
        m_sse[c] = compute(inp0->m_floatComp[c], inp1->m_floatComp[c], inp0->m_compSize[c], m_maxValue[c]);
        m_sseStats[c].updateStats(m_sse[c]);
        m_mse[c] = m_sse[c] / (double) inp0->m_compSize[c];
        m_mseStats[c].updateStats(m_mse[c]);
        m_metric[c] = psnr(m_maxValue[c], inp0->m_compSize[c], m_sse[c]);
        m_metricStats[c].updateStats(m_metric[c]);
      }
    }
    else if (inp0->m_bitDepth == 8) {   // 8 bit data
      for (int c = Y_COMP; c < inp0->m_noComponents; c++) {
        m_sse[c] = (double) compute(inp0->m_comp[c], inp1->m_comp[c], inp0->m_compSize[c]);
        m_sseStats[c].updateStats(m_sse[c]);
        m_mse[c] = m_sse[c] / (double) inp0->m_compSize[c];
        m_mseStats[c].updateStats(m_mse[c]);
        m_metric[c] = psnr(inp0->m_maxPelValue[c], inp0->m_compSize[c], m_sse[c]);
        m_metricStats[c].updateStats(m_metric[c]);
      }
    }
    else { // 16 bit data
      for (int c = Y_COMP; c < inp0->m_noComponents; c++) {
        m_sse[c] = (double) compute(inp0->m_ui16Comp[c], inp1->m_ui16Comp[c], inp0->m_compSize[c]);
        m_sseStats[c].updateStats(m_sse[c]);
        m_mse[c] = m_sse[c] / (double) inp0->m_compSize[c];
        m_mseStats[c].updateStats(m_mse[c]);
        m_metric[c] = psnr(inp0->m_maxPelValue[c], inp0->m_compSize[c], m_sse[c]);
        m_metricStats[c].updateStats(m_metric[c]);
      }
    }
  }
  else {
    printf("Frames of different type being compared. Computation will not be performed for this frame.\n");
  }
}
void DistortionMetricPSNR::computeMetric (Frame* inp0, Frame* inp1, int component)
{
  // it is assumed here that the frames are of the same type
  if (inp0->equalType(inp1)) {
    if (inp0->m_isFloat == TRUE) {    // floating point data
        m_sse[component] = compute(inp0->m_floatComp[component], inp1->m_floatComp[component], inp0->m_compSize[component], m_maxValue[component]);
        m_sseStats[component].updateStats(m_sse[component]);
        m_mse[component] = m_sse[component] / (double) inp0->m_compSize[component];
        m_mseStats[component].updateStats(m_mse[component]);
        m_metric[component] = psnr(m_maxValue[component], inp0->m_compSize[component], m_sse[component]);
        m_metricStats[component].updateStats(m_metric[component]);
    }
    else if (inp0->m_bitDepth == 8) {   // 8 bit data
        m_sse[component] = (double) compute(inp0->m_comp[component], inp1->m_comp[component], inp0->m_compSize[component]);
        m_sseStats[component].updateStats(m_sse[component]);
        m_mse[component] = m_sse[component] / (double) inp0->m_compSize[component];
        m_mseStats[component].updateStats(m_mse[component]);
        m_metric[component] = psnr(inp0->m_maxPelValue[component], inp0->m_compSize[component], m_sse[component]);
        m_metricStats[component].updateStats(m_metric[component]);
    }
    else { // 16 bit data
        m_sse[component] = (double) compute(inp0->m_ui16Comp[component], inp1->m_ui16Comp[component], inp0->m_compSize[component]);
        m_sseStats[component].updateStats(m_sse[component]);
        m_mse[component] = m_sse[component] / (double) inp0->m_compSize[component];
        m_mseStats[component].updateStats(m_mse[component]);
        m_metric[component] = psnr(inp0->m_maxPelValue[component], inp0->m_compSize[component], m_sse[component]);
        m_metricStats[component].updateStats(m_metric[component]);
    }
  }
  else {
    printf("Frames of different type being compared. Computation will not be performed for this frame.\n");
  }
}

void DistortionMetricPSNR::reportMetric  ()
{
  printf("%7.3f %7.3f %7.3f ", m_metric[Y_COMP], m_metric[U_COMP], m_metric[V_COMP]);
  if (m_enableShowMSE == TRUE)
    printf("%10.3f %10.3f %10.3f ", m_mse[Y_COMP], m_mse[U_COMP], m_mse[V_COMP]);
}

void DistortionMetricPSNR::reportSummary  ()
{
  printf("%7.3f %7.3f %7.3f ", m_metricStats[Y_COMP].getAverage(), m_metricStats[U_COMP].getAverage(), m_metricStats[V_COMP].getAverage());
  if (m_enableShowMSE == TRUE)
    printf("%10.3f %10.3f %10.3f ", m_mseStats[Y_COMP].getAverage(), m_mseStats[U_COMP].getAverage(), m_mseStats[V_COMP].getAverage());
}

void DistortionMetricPSNR::reportMinimum  ()
{
  printf("%7.3f %7.3f %7.3f ", m_metricStats[Y_COMP].minimum, m_metricStats[U_COMP].minimum, m_metricStats[V_COMP].minimum);
  if (m_enableShowMSE == TRUE)
    printf("%10.3f %10.3f %10.3f ", m_mseStats[Y_COMP].minimum, m_mseStats[U_COMP].minimum, m_mseStats[V_COMP].minimum);
}

void DistortionMetricPSNR::reportMaximum  ()
{
  printf("%7.3f %7.3f %7.3f ", m_metricStats[Y_COMP].maximum, m_metricStats[U_COMP].maximum, m_metricStats[V_COMP].maximum);
  if (m_enableShowMSE == TRUE)
    printf("%10.3f %10.3f %10.3f ", m_mseStats[Y_COMP].maximum, m_mseStats[U_COMP].maximum, m_mseStats[V_COMP].maximum);
}

void DistortionMetricPSNR::printHeader()
{
  if (m_isWindow == FALSE ) {
  switch (m_colorSpace) {
    case CM_YCbCr:
      printf("PSNR-Y  "); // 8
      printf("PSNR-U  "); // 8
      printf("PSNR-V  "); // 8
      if (m_enableShowMSE == TRUE) {
        printf("   MSE-Y   "); // 11
        printf("   MSE-U   "); // 11
        printf("   MSE-V   "); // 11
      }
      break;
    case CM_RGB:
      printf("PSNR-R  "); // 8
      printf("PSNR-G  "); // 8
      printf("PSNR-B  "); // 8
      if (m_enableShowMSE == TRUE) {
        printf("   MSE-R   "); // 11
        printf("   MSE-G   "); // 11
        printf("   MSE-B   "); // 11
      }
      break;
    case CM_XYZ:
      printf("PSNR-X  "); // 8
      printf("PSNR-Y  "); // 8
      printf("PSNR-Z  "); // 8
      if (m_enableShowMSE == TRUE) {
        printf("   MSE-X   "); // 11
        printf("   MSE-Y   "); // 11
        printf("   MSE-Z   "); // 11
      }
      break;
    default:
      printf("PSNR-C0 "); // 8
      printf("PSNR-C1 "); // 8
      printf("PSNR-C2 "); // 8
      if (m_enableShowMSE == TRUE) {
        printf("   MSE-C0  "); // 11
        printf("   MSE-C1  "); // 11
        printf("   MSE-C2  "); // 11
      }
      break;
  }
}
  else {
    switch (m_colorSpace) {
      case CM_YCbCr:
        printf("wPSNR-Y "); // 8
        printf("wPSNR-U "); // 8
        printf("wPSNR-V "); // 8
        if (m_enableShowMSE == TRUE) {
          printf("  wMSE-Y   "); // 11
          printf("  wMSE-U   "); // 11
          printf("  wMSE-V   "); // 11
        }
        break;
      case CM_RGB:
        printf("wPSNR-R "); // 8
        printf("wPSNR-G "); // 8
        printf("wPSNR-B "); // 8
        if (m_enableShowMSE == TRUE) {
          printf("  wMSE-R   "); // 11
          printf("  wMSE-G   "); // 11
          printf("  wMSE-B   "); // 11
        }
        break;
      case CM_XYZ:
        printf("wPSNR-X "); // 8
        printf("wPSNR-Y "); // 8
        printf("wPSNR-Z "); // 8
        if (m_enableShowMSE == TRUE) {
          printf("  wMSE-X   "); // 11
          printf("  wMSE-Y   "); // 11
          printf("  wMSE-Z   "); // 11
        }
        break;
      default:
        printf("wPSNRC0 "); // 8
        printf("wPSNRC1 "); // 8
        printf("wPSNRC2 "); // 8
        if (m_enableShowMSE == TRUE) {
          printf("  wMSE-C0  "); // 11
          printf("  wMSE-C1  "); // 11
          printf("  wMSE-C2  "); // 11
        }
        break;
    }
  }
}

void DistortionMetricPSNR::printSeparator(){
  printf("--------");
  printf("--------");
  printf("--------");
  if (m_enableShowMSE == TRUE) {
    printf("-----------");
    printf("-----------");
    printf("-----------");
  }
}
//-----------------------------------------------------------------------------
// End of file
//-----------------------------------------------------------------------------
