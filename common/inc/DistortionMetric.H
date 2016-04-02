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
 * \file DistortionMetric.H
 *
 * \brief
 *    Distortion Metric parent class Header
 *
 * \author
 *     - Alexis Michael Tourapis         <atourapis@apple.com>
 *
 *************************************************************************************
 */


#ifndef __DistortionMetric_H__
#define __DistortionMetric_H__

#include "Global.H"
#include "Parameters.H"
#include "Frame.H"

enum DistortionMetrics{
  DIST_NULL         =  0,
  DIST_SSE          =  1, // SSE Metric
  DIST_MSE          =  2, // MSE Metric
  DIST_SNR          =  3, // SNR Metric
  DIST_PSNR         =  4, // PSNR Metric
  DIST_MPSNR        =  5, // mPSNR Metric
  DIST_MPSNRFAST    =  6, // mPSNRfast Metric
  DIST_TFPSNR       =  7, // TF domain PSNR Metric
  DIST_DELTAE       =  8, // DeltaE Metric
  DIST_SIGMACOMPARE =  9, // Sigma Compare Metric
  DIST_MSSSIM       = 10, // Multi-Scale Structural Similarity Index (MS-SSIM)
  DIST_RPSNR        = 11, // Region PSNR
  DIST_RTFPSNR      = 12, // TF Domain Region PSNR
  DIST_TFMSSSIM     = 13, // TF Multi-Scale Structural Similarity Index (tMS-SSIM)
  DIST_BLKJ341      = 14,
  DIST_BLK          = 15,
  DIST_VQM			= 16, //HDR Video Quality Metric
  DIST_METRICS
};

static const int NB_REF_WHITE = 3;


class MetricStatistics	{
public:
  double sum;
  double maximum;
  double minimum;
  int    counter;

  void   reset();
  void   updateStats(double value);
  double getAverage();
  MetricStatistics() {
    reset();
  }
};

class SSIMParams {
public:
  int     m_ssimBlockDistance;
  int     m_ssimBlockSizeX;
  int     m_ssimBlockSizeY;
  bool    m_useLogSSIM;
  double  m_ssimK1;
  double  m_ssimK2;
  SSIMParams () {
    m_ssimBlockDistance = 1;
    m_ssimBlockSizeX = 8;
    m_ssimBlockSizeY = 8;
    m_useLogSSIM = FALSE;
    m_ssimK1 = 0.01f;
    m_ssimK2 = 0.03f;    
  }
};

class DistortionParameters {
public:
  bool    m_enableShowMSE;
  bool    m_computePsnrInYCbCr;
  bool    m_computePsnrInRgb;
  bool    m_computePsnrInXYZ;
  bool    m_computePsnrInYUpVp;
  bool    m_enableCompmPSNR;
  bool    m_enableCompmPSNRfast;
  bool    m_clipInputValues;
  double  m_maxSampleValue;
  double  m_whitePointDeltaE[NB_REF_WHITE];
  double  m_amplitudeFactor;
  bool    m_enableSymmetry;
  
  int                m_ssimBlockDistance;
  int                m_ssimBlockSizeX;
  int                m_ssimBlockSizeY;
  bool               m_useLogSSIM;
  double             m_ssimK1;
  double             m_ssimK2;
  DistortionFunction m_tfPSNRDistortion;
  int                m_deltaEPointsEnable;
  int                m_rPSNRBlockSizeX;
  int                m_rPSNRBlockSizeY;
  int                m_rPSNROverlapX;
  int                m_rPSNROverlapY;
  //VQM parameters
  float	  m_vqmPoolingPerc;
  int     m_vqmRowsDisplay;
  int     m_vqmColsDisplay;
  int     m_vqmDisplayArea;
  float   m_vqmMaxDisplay;
  float   m_vqmMinDisplay;
  int     m_vqmViewingDistance;
  float   m_vqmFixationTime;
  int     m_vqmNumberOfScale;
  int     m_vqmNumberOfOrient;
  float   m_vqmFrameRate;
  int     m_vqm_numberofFrames;
  int	  m_vqm_displayAdapt;
  
  DistortionParameters() {
    for (int index = 0; index < NB_REF_WHITE; index ++)
      m_whitePointDeltaE[index] = 1000.0;
    m_enableShowMSE             = FALSE;
    m_computePsnrInYCbCr        = TRUE;
    m_computePsnrInRgb          = FALSE;
    m_computePsnrInXYZ          = FALSE;
    m_computePsnrInYUpVp        = FALSE;
    m_enableCompmPSNR           = FALSE;
    m_enableCompmPSNRfast       = FALSE;
    m_clipInputValues           = FALSE;
    m_maxSampleValue            = 10000.0;
    m_amplitudeFactor           = 1.0;
    m_enableSymmetry            = FALSE;
    m_ssimBlockDistance         = 1;
    m_ssimBlockSizeX            = 4;
    m_ssimBlockSizeY            = 4;
    m_useLogSSIM                = FALSE;
    m_ssimK1                    = 0.01f;
    m_ssimK2                    = 0.03f;
    m_tfPSNRDistortion          = DIF_PQPH10K;
    m_deltaEPointsEnable        = 1;
    m_rPSNRBlockSizeX           = 64;
    m_rPSNRBlockSizeY           = 64;
    m_rPSNROverlapX             = 4;
    m_rPSNROverlapY             = 4;

  //VQM parameters
    m_vqmPoolingPerc            = 0.0f;
    m_vqmRowsDisplay            = 16;
    m_vqmColsDisplay            = 16;
    m_vqmDisplayArea            = 256;
    m_vqmMaxDisplay             = 1.0f;
    m_vqmMinDisplay             = 0.0f;
    m_vqmViewingDistance        = 1;
    m_vqmFixationTime           = 0.2f;
    m_vqmNumberOfScale          = 1;
    m_vqmNumberOfOrient         = 1;
    m_vqmFrameRate              = 60.0f;
    m_vqm_numberofFrames        = 1;
    m_vqm_displayAdapt          = 1;
  }
};

class DistortionMetric {
private:
public:
  MetricStatistics     m_metricStats[T_COMP];
  double               m_metric     [T_COMP];
  int                  m_totalFrames;
  bool                 m_clipInputValues;
  bool                 m_isWindow;
  //edit by Sarvesh
  bool                 m_allFrames;		// This is required by the VQM implementation. VQM requires two passes.
                                        // This flag informs the VQM module that the first pass is over.

  // Construct/Deconstruct
  DistortionMetric();
  virtual ~DistortionMetric() {};
  static  DistortionMetric *create(const FrameFormat *format, int distortionMetric, DistortionParameters *distortionParameters, bool isWindow = FALSE);
  
  virtual void computeMetric (Frame* inp0, Frame* inp1) = 0;                // Compute metric for all components
  virtual void computeMetric (Frame* inp0, Frame* inp1, int component) = 0; // Compute metric for a single component
  virtual void reportMetric  () = 0;                                        // report results
  virtual void reportSummary () = 0;                                        // report summary results
  virtual void reportMinimum () = 0;                                        // report minimum
  virtual void reportMaximum () = 0;                                        // report maximum
  virtual void printHeader   () = 0;                                        // Print metric Header
  virtual void printSeparator() = 0;                                        // Print line separator

};

#endif
