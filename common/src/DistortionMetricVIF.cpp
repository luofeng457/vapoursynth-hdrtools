/* The copyright in this software is being made available under the BSD
 * License, included below. This software may be subject to other third party
 * and contributor rights, including patent rights, and no such rights are
 * granted under this license.
 *
 * <OWNER> = DML UBC
 * <ORGANIZATION> = DML UBC
 * <YEAR> = 2016
 *
 * Copyright (c) 2016, DML UBC
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
 * \file DistortionMetricVIF.cpp
 *
 * \brief
 *    This is a C++ implementation of the Visual Information Fidelity (VIF) measure.
 *    This implementation was prepared by the Digital Multimedia Lab (DML), 
 *    at the University of British Columbia, BC, Canada
 *    The original code in Matlab is available at:
 *          http://live.ece.utexas.edu/research/quality/VIF.htm
 *
 * \authors
 *     - Hamid Reza Tohidypour        <htohidyp@ece.ubc.ca>
 *     - Maryam Azimi                 <maryama@ece.ubc.ca>
 *     - Mahsa T. Pourazad            <mahsa.pourazad@gmail.com>
 *     - Panos Nasiopoulos            <panos.p.n@ieee.org>
 *************************************************************************************
 */


#include "DistortionMetricVIF.h"
#include "math.h"
#include "Convolve.H"
#include <vector>
#include "Eigenvalue.H"
#include "InverseGauss.H"
#include <iostream>
#include <string.h>

//#include "DistortionMetricSigmaCompare.h";

DistortionMetricVIF::DistortionMetricVIF(const FrameFormat *format, VIFParams *vifParams)
:DistortionMetric()
{
  m_vifFrame = 0; 
  m_vifFrameStats.reset();
  m_vifYBitDepth = vifParams->m_vifBitDepth;
  
}

DistortionMetricVIF::~DistortionMetricVIF()
{
}

// Private functions

//compute mean of a matrix
void DistortionMetricVIF::mean (std::vector<std::vector<double> >  &input, int w, int h, double* mcu)
{
  for (int i = 0; i < w; i++)	{
    double sum1 = 0;
    for (int j = 0; j < h; j++)
      sum1 += input[i][j] / ((double) h);
    mcu[i] = (sum1);
  }
}


void DistortionMetricVIF::reshape (std::vector<std::vector<double> >  &input, int newW, int newH ,  std::vector<std::vector<double> >  &s)
{
  int k=0;
  for (int i = 0; i < newW; i++)	{
    for (int j = 0; j < newH; j++) {
      s[i][j] = input[0][k++];
    }
  }
}


void DistortionMetricVIF::vectorMax (int* input, int size , int imax)
{
  imax=input[0];
  for (int j = 0; j < size; j++) {
    if (input[j] > imax) {
      imax = input[j];
    }
  }
}


void DistortionMetricVIF::differenceVector(std::vector<std::vector<double> >  &input1, std::vector<std::vector<double> >  &input2, int w, int h, std::vector<std::vector<double> >  &output)
{
  for (int i = 0; i < w; i++)	{
    output[i].resize(h);
    for (int j = 0; j < h; j++)	{
      output[i][j] = (input1[i][j] - input2[i][j]);
    }
  }
}



void DistortionMetricVIF::invertVector(std::vector<std::vector<double> >  &input1,int w, int h, std::vector<std::vector<double> >  &output)
{
  
  for (int j = 0; j < h; j++)	{
    output[j].resize(w);
  }
  
  for (int j = 0; j < h; j++)	{
    for (int i = 0; i < w; i++)	{	
      output[j][i]= input1[i][j];
    }
  }
}

// This code should use the already available color conversion code. Please fix

//convert RGB to R'G'B' then compute the non constant luminance 
//This function accepts r:red, g:green and b:blue of the RGB and fist it PQ codes the RGB frame 
//Then it converts the pq coded RGB to luminace based on the color primaires and bit depth of the frame.
double DistortionMetricVIF::rgbPQYuv (double r, double g, double b, int bitdepth,  ColorPrimaries cp)
{
  r = SMPTE_ST_2084(r, TRUE, 10000.0);
  g = SMPTE_ST_2084(g, TRUE, 10000.0);
  b = SMPTE_ST_2084(b, TRUE, 10000.0);
  
  double y=0;
  if (cp == CP_709)  {
    y=0.2126 * r + 0.7152 * g + 0.0722 * b;
  }
  else  {
    if (cp == CP_2020)    {
      y = 0.2627 * r + 0.6780 * g + 0.0593 * b;
    }
    else    {
      printf (" The Current version of VIF only supports BT709 and BT2020 ! ");
      exit(EXIT_FAILURE);
    }
  }
  
  // % Scaling and quantization
  //Taken from ITU. (2002). RECOMMENDATION ITU-R BT.1361.
  y = (219 * y + 16) * pow(2.0,(double) bitdepth - 8.0);
  
  return(y);
}

// This code should be removed completely. This process already exists and it is best to try and 
// reuse the existing code.
double DistortionMetricVIF::SMPTE_ST_2084(double x, bool Inverse, double MaxLum)
{
  //SMPTE_ST_2084 - implement the SMPTE ST 2084 as described in the
  // MPEG Call for Evidence in Section B.1.5.3.1
  //Inputs:
  //   - x: input image, if Inverse = 1 : RGB, otherwise R'G'B'.
  //    - Inverse: Inverse EOTF (encoding)
  //    - MaxLum: maximum luminance (default= 10,000 nits).
  //
  // Outputs:
  //    -y: output image, if Inverse = 1 : encoded image, otherwise decoded
  
  double m1 = 2610 / 4096.0 / 4;
  double m2 = 2523 / 4096.0 * 128;
  double c1 = 3424 / 4096.0;
  double c2 = 2413 / 4096.0 * 32;
  double c3 = 2392 / 4096.0 * 32;
  double y=0;
  if (Inverse)  {
    // Normalize The input by the Maximum authorized luminance
    x = x / MaxLum;
    x = dClip(x, 0.0, 1.0);
    
    // Apply the inverse EOTF on the input image
    double  y1 = ((c2 * (pow (x, m1)) + c1) / (1 + c3 * (pow (x, m1))));
    y = pow(y1, m2);
  }
  else
  {
    //Remove values outside [0-1] for example caused by overflow in coding
    x = dClip(x, 0.0, 1.0);
    // % Apply the EOTF on the input image
    double y0 = ((pow(x,(1 / m2))- c1) / (c2 - c3 * pow(x,(1 / m2))));
    double y  = pow(y0, 1/m1);
    // Scale the output to the maximum luminance
    y = y * MaxLum; 
  }
  return y;
}


//output=input1*input2
void DistortionMetricVIF::vectorMultiplication(std::vector<std::vector<double> >  &input1, int w1,int h1,std::vector<std::vector<double> >  &input2, int w2,int h2,  std::vector<std::vector<double> >  &output)
{
  for (int i = 0;i < w1; i++)	{
    output[i].resize(h2);
    for (int i1 = 0; i1 < h2; i1++)	{
      double sum=0;
      for (int j = 0; j < h1; j++) {
        sum += (input1[i][j] * input2[j][i1]);
      }
      output [i][i1]= (sum);
    }
  }
}

void DistortionMetricVIF::vectorMultiplicationInverse (std::vector<std::vector<double> >  &input1, int w1,int h1,std::vector<std::vector<double> >  &input2,  std::vector<std::vector<double> >  &output)
{
  int h2=w1;
  //	int w2=h1;
  for (int i = 0;i < w1; i++) {
    output[i].resize(h2);    
    for (int i1 = 0; i1 < h2; i1++) {
      double sum=0;
      for (int j = 0; j < h1; j++) {
        sum +=(input1[i][j] * input2[i1][j]);
      }
      output [i][i1]= (sum);
    }
  }
}

//output=input1/input2
void DistortionMetricVIF::vectorMultiplyFixNum(std::vector<std::vector<double> >  &input1, int w1, int h1,double input2,  std::vector<std::vector<double> >  &output)
{
  for (int i = 0;i < w1; i++) {
    output[i].resize(h1);
    for (int j = 0; j < h1; j++) {
      output[i][j]=(input1[i][j] * input2);
    }
  }
}

//output=input1+input2
void DistortionMetricVIF::vectorAddFixNum(std::vector<std::vector<double> >  &input1, int w1, int h1,double input2,  std::vector<std::vector<double> >  &output)
{
  for (int i = 0;i < w1; i++) {
    output[i].resize(h1);
    for (int j = 0; j < h1; j++) {
      output[i][j]=(input1[i][j]+input2);
    }
  }
}

//output=input1.*input2
void DistortionMetricVIF::vectorMultiplicationSameSize(std::vector<std::vector<double> >  &input1, int w1, int h1, std::vector<std::vector<double> >  &input2,  std::vector<std::vector<double> >  &output)
{
  for (int i = 0;i < w1; i++)	{
    output[i].resize(h1);
    for (int j = 0; j < h1; j++) {
      output[i][j]= (input1[i][j] * input2[i][j]);
    }
  }
}


double DistortionMetricVIF::sumSumLog2GGSSLambdaDivVV(std::vector<std::vector<double> >  &g, int w1, int h1, std::vector<std::vector<double> >  &ss, double lambda, double eps, std::vector<std::vector<double> >  &vv)
{
  double sum=0.0;
  for (int i = 0;i < w1; i++) {
    for (int j = 0; j < h1; j++) {
      sum+= log(1.0 + g[i][j] * g[i][j] * ss[i][j] * lambda / (vv[i][j] + eps))/log(2.0);
    }
  }
  return sum;
}


// temp2=temp2+sum(sum((log2(1+ss.*lambda(j)./(sigma_nsq))))); % reference image information in VIF matlab code
double DistortionMetricVIF::sumSumLog2SSLambdaDivSigma(std::vector<std::vector<double> > &ss, int w1, int h1,double lambda, double eps)
{
  double sum = 0.0;
  for (int i = 0 ;i < w1; i++) {
    for (int j = 0; j < h1; j++) {
      sum+= log(1.0 + ss[i][j] * lambda/(eps))/log(2.0);  //reference image information
    }
  }
  return sum;
}

//  The function is added for sumSSTempMM temp2=temp2+sum(sum((log2(1+ss.*lambda(j)./(sigma_nsq)))));
void DistortionMetricVIF::sumSSTempMM(std::vector<std::vector<double> >  &input1, int w1, int h1, std::vector<std::vector<double> >  &input2, double input3, std::vector<std::vector<double> >  &output)
{
  output[0].resize(h1);
  for (int j = 0; j < h1; j++)	{
    double sum = 0.0;
    for (int i = 0;i < w1; i++)	{
      sum+=(input1[i][j] * input2[i][j]) / input3;
    }
    output[0][j]=sum;
  }
}

//output=input1./input2
void DistortionMetricVIF::vectorDivisionSameSize(std::vector<std::vector<double> >  &input1, int w1, int h1, std::vector<std::vector<double> >  &input2, double input3, std::vector<std::vector<double> >  &output)
{
  //input3 is to avoid devision by zero
  for (int i = 0;i < w1; i++)	{
    output[i].resize(h1);
    for (int j = 0; j < h1; j++) {
      output[i][j]= (input1[i][j] / (input2[i][j] + input3));
    }
  }
}

//repmat (double* input, int w, int h, std::vector<std::vector<double> >  output)   creates a large matrix output 
//consisting of an w-by-h tiling of copies of input. 
void DistortionMetricVIF::repmat (double* input, int w, int h, std::vector<std::vector<double> >  &output)
{
  for (int i = 0; i < w; i++)	{
    output[i].resize(h);
    for (int j = 0; j < h; j++)	{
      output[i][j]= (input[i]);
    }
  }
}

//
//output (input < compVal) = setVal;
void DistortionMetricVIF::setCompare (double* input, int size, double compVal, double setVal, bool iseq, double *output )
{
  if ( iseq)	{
    for (int i = 0; i < size; i++)	{
      if (input[i] <= compVal)
        output[i] = setVal; 
    }
  }
  else	{
    for (int i = 0; i < size; i++)	{
      if (input[i] < compVal)
        output[i] = setVal;  
    }
  }
}

//output (input < compVal) = setInput (input < compVal);
void DistortionMetricVIF::setCompareWithOther (double *compInput, double *setInput, int size, double compVal, bool iseq, double *output )
{
  if (iseq == TRUE) {
    for (int i = 0; i < size; i++)	{
      if (compInput[i] <= compVal)
        output[i] = setInput[i]; 
    }
  }
  else {
    for (int i = 0; i < size; i++)	{
      if (compInput[i] < compVal)
        output[i] = setInput[i]; 
    }
  }
}

//output=input1-input2
void DistortionMetricVIF::difference (double* input1, double* input2, int size,   double* output )
{
  for (int i = 0; i < size; i++)
    output[i] = input1[i] - input2[i];
}

//output=input+num
void DistortionMetricVIF::addFix (double* input, int size, double num,   double* output )
{
  for (int i = 0; i < size; i++)
    output[i] = input[i] + num ;
}

//output=input/num
void DistortionMetricVIF::divFix (double* input, int size, double num,   double* output )
{
  for (int i = 0; i < size; i++)
    output[i] = input[i] / num ;
}

//output=input/input2
void DistortionMetricVIF::div (double* input, double* input2, int size,   double* output )
{
  for (int i = 0; i < size; i++)
    output[i] = input[i] / input2[i] ;
}

//output=input1*input2
void DistortionMetricVIF::multiply (double* input1, double* input2, int size, double num,  double* output )
{
  for (int i = 0; i < size; i++)
    output[i] = input1[i] * input2[i] * num ;
}

void  DistortionMetricVIF::convertOneRowVar ( const double** input, double * output)
{
  int xFDimInput = sizeof(input) / sizeof(*input);
  for (int i = 0; i < xFDimInput; i ++)	{
    for (int j = 0; j < xFDimInput; j ++)		{
      //reshape the filter into matrice
      output[j + i * xFDimInput] =	input[j][i] ;
    }
  }
}



//==========================================

//[ssarr, lArr, cuArr]=refParamsVecGSM(org,subands,M)
//This function computes the parameters of the reference image. This is called by vifvec.m in the matlab code.
void DistortionMetricVIF::refParamsVecGSM (std::vector<std::vector<double> > &org,  
                                           int* subbands, 
                                           int M, 
                                           std::vector<std::vector<int> > &lenWh, 
                                           int sizeSubBand, 	  
                                           std::vector<std::vector<std::vector<double> > >  &ssArr,  
                                           std::vector<std::vector<double> > &lArr, 
                                           std::vector<std::vector<std::vector<double> > >  &cuArr
                                           ) 
{
  Eigenvalue eg1;
  
  for (int subi=0;subi<sizeSubBand;subi++)	{
    int   sub = subbands[subi];
    
    //force subband size to be multiple of M
    int newSizeYWidth  = (int) ((floor(lenWh[sub][0] /((double) M))) * (double) M);
    int newSizeYHeight = (int) ((floor(lenWh[sub][1] /((double) M))) * (double) M);
    
    std::vector<std::vector<double> > yMM(newSizeYWidth);
    for (int w = 0; w < newSizeYWidth; w++) {
      yMM[w].resize(newSizeYHeight);
      for (int h = 0; h < newSizeYHeight; h++)			{
        yMM[w][h]=org[sub][w + h * lenWh[sub][0]];
      }
    }
    
    //================================
    //Collect MxM blocks. Rearrange each block into an
    //M^2 dimensional vector and collect all such vectors.
    //Collece ALL possible MXM blocks (even those overlapping) from the subband
    std::vector<std::vector<double> > temp(M * M);
    int count=0;
    int sizeTempH=0;
    
    for (int k = 0; k < M; k++)	{
      for (int j = 0; j < M; j++)	{
        int totalSize= (newSizeYWidth - (M - 1 - k) - k) * (newSizeYHeight - (M - 1 - j) - j);
        temp [count].resize(totalSize);
        sizeTempH=0;
        for (int w0 = k; w0 < newSizeYWidth - (M - 1 - k); w0++) {
          for (int h0 = j; h0 < newSizeYHeight - (M - 1 - j); h0++) {
            temp[count][sizeTempH] = (yMM[w0][h0]);
            sizeTempH++;
          }
        }
        count++;
      }
    }
    
    int totalSize= (newSizeYWidth - (M - 1)) * (newSizeYHeight - (M - 1));
    
    //estimate mean 
    std::vector<double> mcu(M * M);
    mean(temp, M * M, totalSize, &mcu[0]);
    
    //estimate covariance
    std::vector<std::vector<double> > cu1 (M * M);
    std::vector<std::vector<double> > cu2 (M * M);
    std::vector<std::vector<double> > rep1(M * M);
    
    repmat(&mcu[0], M * M, sizeTempH, rep1);
    differenceVector(temp, rep1, M * M, sizeTempH, cu1);
    vectorMultiplicationInverse (cu1, M * M, sizeTempH, cu1, cu2);
    for (int ii = 0; ii < M * M; ii++)	{
      for (int jj = 0;jj < M * M; jj++) {
        cuArr[sub] [ii][jj]=(cu2[ii][jj]/sizeTempH); //% covariance matrix for U
      }
    }
    
    //================================
    //Collect MxM blocks as above. Use ONLY non-overlapping blocks to
    //calculate the S field
    std::vector<std::vector<double> > temp1(M * M);
    
    count = 0;
    int temp1Size = 0;
    for (int k = 0; k < M; k++)		{
      for (int j = 0;j < M; j++)			{
        temp1Size=0;
        temp1[count].resize(newSizeYWidth * newSizeYHeight);
        for (int w0 = k; w0 < newSizeYWidth; w0 += M) {
          for (int h0 = j; h0 < newSizeYHeight;h0 += M) {
            temp1[count][temp1Size]=(yMM[w0][h0]);
            temp1Size++;
          }
        }
        count++;
      }
    }
    
    //Calculate the S field
    std::vector<std::vector<double> > inCuCo(M * M);    
    
    for (int i = 0; i < M * M;i++)	{
      inCuCo[i].resize(M * M); 
    }
    InverseGauss::inverseMatrix(cuArr[sub], M * M, inCuCo);
    
    std::vector<std::vector<double> > ss1(M * M);
    vectorMultiplication(inCuCo, M * M, M * M, temp1, M * M, temp1Size, ss1);
    std::vector<std::vector<double> >  ss4(1);
    
    sumSSTempMM (ss1, M * M, temp1Size, temp1, M * M, ss4);
    reshape(ss4,(int)(newSizeYWidth / (double) M), (int)(newSizeYHeight / (double) M), ssArr[sub]);
    
    //==========================
    //Eigen-decomposition
    std::vector<double> a(M * M * M * M);
    
    int k=0;
    for (int i = 0; i < M * M; i++)	{
      for (int j = 0;j < M * M; j++)	{
        a[k++] = (cuArr[sub][i][j]);
      }
    }
    int n = M * M;
    
    std::vector<double> v(M * M * M * M);
    
    int itMax = 100;
    int rotNum;
    eg1.jacobiEigenvalue ( n,  &a[0], itMax, &v[0],  &lArr[sub][0], itMax, rotNum );
  }
}
//=================================

void DistortionMetricVIF::vifSubEstM (std::vector<std::vector<double> >  &org, 
                                      std::vector<std::vector<double> >  &dist, 
                                      int* subbands, 
                                      int M, 
                                      std::vector<std::vector<int> > &lenWh, 
                                      int sizeSubBand, 
                                      std::vector<std::vector<std::vector<double> > >  &gAll, 
                                      std::vector<std::vector<std::vector<double> > >  &VvAll, 
                                      std::vector<std::vector<int> > &lenWhGAll
                                      )
{
  double tol = 1e-15; 
  for (int i=0;i<sizeSubBand;i++) {
    int sub = subbands[i];
    int sizeY = lenWh[sub][0] * lenWh[sub][1];
    std::vector<double> y (sizeY);
    std::vector<double> yn(sizeY);
    
    memcpy (&y[0], &org[sub][0],  sizeY * sizeof (double));
    memcpy (&yn[0],&dist[sub][0], sizeY * sizeof (double));
    
    //compute the size of the window used in the distortion channel estimation
    int  lev = (int) ceil((sub-1)/6.0);
    int winSize= (1 << lev) + 1; // pow(2.0, lev) + 1; 
                                 //    double offset=(winSize-1)/2.0;
    std::vector<double> win       (winSize * winSize);
    std::vector<double> winNormal(winSize * winSize);
    
    for (int j = 0; j < winSize * winSize; j++) {
      win[j]=1.0; 
      winNormal[j]=1.0 / (((double) winSize)*((double)winSize)) ;
    }
    
    //force subband size to be multiple of M
    int newSizeYWidth  = (int) (floor(lenWh[sub][0]/((double)M))*(double) M);
    int newSizeYHeight = (int) (floor(lenWh[sub][1]/((double)M))*(double) M);
    
    std::vector<double> yMM (newSizeYWidth*newSizeYHeight);
    std::vector<double> ynMM(newSizeYWidth*newSizeYHeight);
    
    for (int w=0;w<newSizeYWidth;w++) {
      for (int h=0;h<newSizeYHeight;h++) {
        yMM [w + h * newSizeYWidth] = y [w + h * lenWh[sub][0]];
        ynMM[w + h * newSizeYWidth] = yn[w + h * lenWh[sub][0]];
      }
    }
    
    //Correlation with downsampling. This is faster than downsampling after
    //computing full correlation.
    int winStepX  = M; 
    int winStepY  = M;
    int winStartX = (int) floor(M / 2.0); 
    int winStartY = winStartX;
    int winStopX  = newSizeYWidth  - (int) ceil(M / 2.0) + 1; 
    int winStopY  = newSizeYHeight - (int) ceil(M / 2.0) + 1; 
    
    //mean
    int meanXSize = (int)((double) newSizeYWidth / (double) M * (double)newSizeYHeight / (double) M);
    std::vector<double> meanX(meanXSize);
    std::vector<double> meanY((int) (newSizeYWidth / (double) M * newSizeYHeight / (double) M));
    std::vector<double> temp  (newSizeYWidth * newSizeYHeight);
    
    Convolve::internalReduce(&yMM[0], newSizeYWidth,  newSizeYHeight, 
                             &winNormal[0], &temp[0],  winSize,  winSize,
                             winStartX,  winStepX,  winStopX, 
                             winStartY, winStepY,  winStopY,
                             &meanX[0]); 
    
    Convolve::internalReduce(&ynMM[0], newSizeYWidth,  newSizeYHeight, 
                             &winNormal[0], &temp[0],  winSize,  winSize,
                             winStartX,  winStepX,  winStopX, 
                             winStartY, winStepY,  winStopY,
                             &meanY[0]); 
    
    //cov
    std::vector<double> yMMYnMM(newSizeYWidth * newSizeYHeight);
    
    multiply (&yMM[0], &ynMM[0], newSizeYWidth * newSizeYHeight, 1.0, &yMMYnMM[0]);
    
    std::vector<double> covXY ((int) (newSizeYWidth / (double) M * newSizeYHeight / (double) M));
    std::vector<double> covXY1((int) (newSizeYWidth / (double) M * newSizeYHeight / (double) M));
    std::vector<double> covXY2((int) (newSizeYWidth / (double) M * newSizeYHeight / (double) M));
    
    multiply (&meanX[0], &meanY[0], (int) (newSizeYWidth / (double) M* newSizeYHeight / (double) M) , winSize * winSize, &covXY2[0]);    
    
    Convolve::internalReduce(&yMMYnMM[0], newSizeYWidth,  newSizeYHeight, 
                             &win[0], &temp[0],  winSize,  winSize,
                             winStartX,  winStepX,  winStopX, 
                             winStartY, winStepY,  winStopY,
                             &covXY1[0]); 
    difference (&covXY1[0],&covXY2[0], (int) (newSizeYWidth / (double) M* newSizeYHeight / (double) M), &covXY[0]);    
    //varx
    std::vector<double> SsX1((int) (newSizeYWidth / (double) M * newSizeYHeight / (double) M));
    std::vector<double> SsX2((int) (newSizeYWidth / (double) M * newSizeYHeight / (double) M));
    std::vector<double> SsX ((int) (newSizeYWidth / (double) M * newSizeYHeight / (double) M));
    std::vector<double> yMM_2(newSizeYWidth * newSizeYHeight);
    
    multiply (&yMM[0], &yMM[0], newSizeYWidth * newSizeYHeight ,1.0, &yMM_2[0]);
    Convolve::internalReduce(&yMM_2[0], newSizeYWidth,  newSizeYHeight, &win[0], &temp[0], winSize, winSize,	winStartX,  winStepX,  winStopX, winStartY, winStepY,  winStopY, &SsX1[0]); 
    multiply (&meanX[0], &meanX[0],  (int) (newSizeYWidth / (double) M * newSizeYHeight / (double) M), winSize * winSize,  &SsX2[0]);
    difference (&SsX1[0], &SsX2[0],  (int) (newSizeYWidth / (double) M * newSizeYHeight / (double) M), &SsX[0]);
    
    //vary 
    std::vector<double> SsY1((int)(newSizeYWidth / (double) M * newSizeYHeight / (double)M));
    std::vector<double> SsY2((int)(newSizeYWidth / (double) M * newSizeYHeight / (double)M));
    std::vector<double> SsY ((int)(newSizeYWidth / (double) M * newSizeYHeight / (double)M));
    std::vector<double> ynMM2(newSizeYWidth * newSizeYHeight);
    
    multiply (&ynMM[0], &ynMM[0], newSizeYWidth * newSizeYHeight, 1.0, &ynMM2[0]);
    
    Convolve::internalReduce(&ynMM2[0], newSizeYWidth,  newSizeYHeight, &win[0], &temp[0],  winSize,  winSize, winStartX,  winStepX,  winStopX, winStartY, winStepY,  winStopY, &SsY1[0]); 
    multiply (&meanY[0], &meanY[0],  (int) (newSizeYWidth/(double) M * newSizeYHeight / (double) M), winSize * winSize,  &SsY2[0] );
    difference (&SsY1[0], &SsY2[0],  (int) (newSizeYWidth/(double) M * newSizeYHeight / (double) M), &SsY[0]);
    
    // get rid of numerical problems, very small negative numbers, or very
    // small positive numbers, or other theoretical impossibilities.
    setCompare(&SsX[0],  (int) (newSizeYWidth / (double) M * newSizeYHeight /(double) M), 0, 0, FALSE, &SsX[0]);
    setCompare(&SsY[0],  (int) (newSizeYWidth / (double) M * newSizeYHeight /(double) M), 0, 0, FALSE, &SsY[0]);
    
    // Regression 
    std::vector<double> gNum((int) (newSizeYWidth / (double) M * newSizeYHeight / (double) M));
    std::vector<double> g   ((int) (newSizeYWidth / (double) M * newSizeYHeight / (double) M));
    
    addFix(&SsX[0],  (int) (newSizeYWidth / (double) M * newSizeYHeight / (double) M), tol,&gNum[0]);
    div (&covXY[0], &gNum[0], (int) (newSizeYWidth / (double) M* newSizeYHeight / (double) M), &g[0]);
    
    //Variance of error in regression
    std::vector<double> vv    ((int)(newSizeYWidth / (double) M * newSizeYHeight / (double) M));
    std::vector<double> vvNum1((int)(newSizeYWidth / (double) M * newSizeYHeight / (double) M));
    std::vector<double> vvNum ((int)(newSizeYWidth / (double) M * newSizeYHeight / (double) M));
    
    multiply   (&g[0],&covXY[0],  (int) (newSizeYWidth / (double) M* newSizeYHeight / (double) M), 1.0, &vvNum1[0]);
    difference (&SsY[0], &vvNum1[0],  (int) (newSizeYWidth / (double) M* newSizeYHeight / (double) M), &vvNum[0]);
    divFix     (&vvNum[0],  (int) (newSizeYWidth / (double) M* newSizeYHeight / (double) M), winSize * winSize, &vv[0]);
    
    
    // get rid of numerical problems, very small negative numbers, or very
    // small positive numbers, or other theoretical impossibilities.
    setCompare          (&SsX[0],           (int) (newSizeYWidth / (double) M * newSizeYHeight / (double) M), tol,   0, FALSE, &g[0]);
    setCompareWithOther (&SsX[0],  &SsY[0], (int) (newSizeYWidth / (double) M * newSizeYHeight / (double) M), tol,      FALSE, &vv[0]);
    setCompare          (&SsX[0],           (int) (newSizeYWidth / (double) M * newSizeYHeight / (double) M), tol,   0, FALSE, &SsX[0]);
    setCompare          (&SsY[0],           (int) (newSizeYWidth / (double) M * newSizeYHeight / (double) M), tol,   0, FALSE, &g[0]);
    setCompare          (&SsY[0],           (int) (newSizeYWidth / (double) M * newSizeYHeight / (double) M), tol,   0, FALSE, &vv[0]);
    
    //constrain g to be non-negative. 
    setCompareWithOther (&g[0],    &SsY[0], (int) (newSizeYWidth / (double) M * newSizeYHeight / (double) M),        0, FALSE, &vv[0]);
    setCompare          (&g[0],             (int) (newSizeYWidth / (double) M * newSizeYHeight / (double) M),   0,   0, FALSE, &g[0]);
    
    //take care of numerical errors, vv could be very small negative
    setCompare          (&vv[0],            (int) (newSizeYWidth / (double) M * newSizeYHeight / (double) M), tol, tol,  TRUE, &vv[0]);
    
    for (int jj = 0; jj < newSizeYWidth  / (double) M; jj++) {
      for (int h = 0; h < newSizeYHeight / (double) M;h++) {
        gAll [sub][jj][h] = g [jj + h * (int)(newSizeYWidth / (double) M)];
        VvAll[sub][jj][h] = vv[jj + h * (int)(newSizeYWidth / (double) M)];
      }
    }
  }
}


//Construct a steerable pyramid on matrix IM.  Convolutions are
//done with spatial filters.
int DistortionMetricVIF::buildSpyrLevs(double* lo0, int ht, int w, int h, std::vector<std::vector<double> > &pyr, int max_ht, int pyrElement, std::vector<std::vector<double> >  &pyr_arr, std::vector<bool> &subbandUsed)
{
  
  //variables names were chosen similar to the Matlab code
  int bfiltsz = 0;
  int bFiltsZ2 = sizeof(bfilts) / sizeof(*bfilts); 
  int bfiltsz_1 = sizeof(*bfilts)/ sizeof(double);
  bfiltsz = (int) sqrt ((double) bfiltsz_1);
  /*
   double *bands =  new double[ w * h * bFiltsZ2 ];
   double *band  =  new double[ w * h ];
   double *bind  =  new double[ 2+bFiltsZ2 * 2];  
   double *temp  =  new double[ w * h ];
   int w_lo= ceil(w/2.0); int h_lo=ceil(h/2.0);
   double *lo  =  new double[ w_lo * h_lo ];
   double* bfilts_one_row=new double [bfiltsz*bfiltsz];
   std::vector<std::vector<double> >  bfilts_reshaped;
   bfilts_reshaped =new double * [bfiltsz];
   */
  int w_lo = (int) ceil(w / 2.0); 
  int h_lo = (int) ceil(h / 2.0);
  
  std::vector<double>  bands;
  std::vector<double>  band;
  std::vector<double>  bind;
  std::vector<double>  temp;
  std::vector<double>  lo;
  std::vector<double> bfilts_one_row;
  std::vector<std::vector <double> > bfilts_reshaped;
  
  bands.resize( w * h * bFiltsZ2 );
  band.resize( w * h );
  bind.resize( 2+bFiltsZ2 * 2);  
  temp.resize( w * h );
  lo.resize( w_lo * h_lo );
  bfilts_one_row.resize(bfiltsz * bfiltsz);
  bfilts_reshaped.resize(bfiltsz);
  
  for (int i=0;i<bfiltsz; i++)	{
    bfilts_reshaped [i].resize(bfiltsz);
  }
  
  if (ht <=0)	{
    if (subbandUsed[pyrElement]==TRUE)		{
      int k1=0;
      for (int j = 0; j < h; j++)
        for (int i = 0; i < w ; i++)				{
          pyr_arr[pyrElement][k1] = lo0[(i + j * w)]; 
          k1++;
        }
    }
  }
  else
  {			
    for ( int b = 0; b < bFiltsZ2; b ++ )		{
      int k = 0;
      
      for (int i = 0; i < bfiltsz  ; i ++)			{
        for (int j =0; j < bfiltsz ; j ++)				{
          //reshape the filter into matrice
          bfilts_reshaped[j][i] = bfilts [b][k];
          bfilts_one_row[i+j*bfiltsz]=	bfilts_reshaped[j][i] ;
          k++;
        }
      }
      
      Convolve::internalReduce(lo0, w,  h, &bfilts_one_row[0], &temp[0],  bfiltsz,  bfiltsz, 	0,  1,  w, 0,  1,  h,	&band[0]);
      
      if (subbandUsed [pyrElement]==TRUE)
      {
        int k1=0;
        for (int j = 0; j < h; j++)
          for (int i = 0; i < w ; i++)
          {
            pyr_arr[pyrElement][k1] = band [(i + j * w)]; 
            k1++;
          }
      }
      pyrElement--;
      
      for (int i = 0; i < 2 ; i++)
        if ( i == 0)
          bind[ i + b * 2] = w;
        else
          bind[ i + b * 2] = h;
    }
    
    int lioFiltSize= sizeof(lofilt) / sizeof(*lofilt);
    std::vector<double>  loFiltOneRow;
    loFiltOneRow.resize( lioFiltSize * lioFiltSize);
    for (int i = 0; i <  lioFiltSize ; i ++)
    {
      for (int j =0; j < lioFiltSize ; j ++)
      {
        //reshape the filter into matrice
        loFiltOneRow[j+i* lioFiltSize]=	lofilt[j][i] ;
      }
    }
    
    Convolve::internalReduce(lo0, w,  h, &loFiltOneRow[0], &temp[0],  lioFiltSize,  lioFiltSize, 0,  2,  w, 0,  2,  h, &lo[0]);
    
    buildSpyrLevs(&lo[0], ht - 1, (w + 1) >> 1, (h + 1) >> 1, pyr, max_ht, pyrElement, pyr_arr, subbandUsed);
    
  }
  
  return(0);
}

int DistortionMetricVIF::maxPyrHt(int imSizeW, int imSizeH,int filtsize)
{
  int height = 0;
  if (imSizeW < filtsize || imSizeH <filtsize) {
    height = 0;
  }
  else  {
    height = 1 + maxPyrHt(imSizeW >> 1, imSizeH >> 1, filtsize); 
    //height = 1 + maxPyrHt(int(imSizeW/2), int(imSizeH/2), filtsize);
  }
  return (height);
}

//This function is the main function which computes the VIF values for the input frames.
//Advanced Usage:
//  Users may want to modify the parameters in the code. 
//   (1) Modify sigma_nsq to find tune for your image dataset.
//   (2) MxM is the block size that denotes the size of a vector used in the
//   GSM model.
//   (3) subbands included in the computation, subands were modified for c++
//========================================================================
void DistortionMetricVIF::computeMetric (Frame* inp0, Frame* inp1)
{
  
  double sigma_nsq=0.4;
  int M = 3;
  //int subbands []={4 ,7, 10, 13, 16, 19, 22, 25}; //Matlab Subbands
#if spFilters5
  int subbands []={3 ,6, 9, 12, 15, 18, 21, 24}; //C++ sub bands
#else
  int subbands []={3 ,6, 9, 12, 15}; //C++ sub bands for sp1filter
#endif
  
  int sizeSubBand = sizeof(subbands )/sizeof(*subbands );
  int pYrSize = maxPyrHt(inp0->m_width[Y_COMP],inp0->m_height[Y_COMP],  sizeof(lo0filt) / sizeof(*lo0filt)   )-1;
  std::vector<double>  image_org (inp0->m_height[Y_COMP] * inp0->m_width[Y_COMP]); 
  std::vector<double>  image_dist(inp0->m_height[Y_COMP] * inp0->m_width[Y_COMP]);
  
  //compute the non-constant luminace for each frames	
  for (int i=0;i<inp0->m_height[Y_COMP] * inp0->m_width[Y_COMP];i++) {
    image_org[i]=   rgbPQYuv ( (double)inp0->m_floatComp[0][i], (double)inp0->m_floatComp[1][i],(double)inp0->m_floatComp[2][i], m_vifYBitDepth, inp0->m_colorPrimaries);
    image_dist[i]=  rgbPQYuv ( (double)inp1->m_floatComp[0][i], (double)inp1->m_floatComp[1][i],(double)inp1->m_floatComp[2][i], m_vifYBitDepth, inp1->m_colorPrimaries);
  }
  
  
  //variables names were chosen similar to the Matlab code
  std::vector<double> hi0     (inp0->m_height[Y_COMP] * inp0->m_width[Y_COMP]);
  std::vector<double> lo0_org (inp0->m_height[Y_COMP] * inp0->m_width[Y_COMP]);
  std::vector<double> lo0_dist(inp0->m_height[Y_COMP] * inp0->m_width[Y_COMP]);
  std::vector<double> temp    (inp0->m_height[Y_COMP] * inp0->m_width[Y_COMP]);
  
  int xFDimHi0 = sizeof(hi0filt) / sizeof(*hi0filt);
  int yFDimHi0 = xFDimHi0;
  int xFDimLo0 = sizeof(lo0filt) / sizeof(*lo0filt);
  int y_fdim_lo0 = xFDimLo0;
  int xStart = 0;
  int yStart = 0;
  int xStep  = 1; 
  int yStep  = 1;
  int xStop  = inp0->m_width[Y_COMP];
  int yStop  = inp0->m_height[Y_COMP];
  int xDim   = inp0->m_width[Y_COMP];
  int yDim   = inp0->m_height[Y_COMP];
  int li0FiltSize = sizeof(lo0filt) / sizeof(*lo0filt);
  int bFiltsZ2    = sizeof(bfilts)  / sizeof(*bfilts); 
  std::vector<std::vector<int> > lenWh(pYrSize * bFiltsZ2 + 2);
  
  for (int i=0;i<pYrSize*bFiltsZ2+2;i++)
    lenWh[i].resize(2);
  
  std::vector<std::vector<double> > pyrOrgArr (pYrSize * bFiltsZ2 + 2);
  std::vector<std::vector<double> > pyrDistArr(pYrSize * bFiltsZ2 + 2);
  std::vector<bool> subbandUsed(pYrSize * bFiltsZ2 + 2);
  
  for (int i=0;i<pYrSize*bFiltsZ2+2;i++)	{
    subbandUsed [i]=FALSE;
  }
  
  for (int i=0;i<sizeSubBand;i++)	{
    subbandUsed[subbands[i]]=TRUE;
  }
  
  int elementPyrOrg  = pYrSize * bFiltsZ2 + 2 - 1;
  int elementPyrDist = pYrSize * bFiltsZ2 + 2 - 1;
  
  
  lenWh[elementPyrOrg][0] = xStop;  
  lenWh[elementPyrOrg][1] = yStop;
  int k1=elementPyrOrg;
  
  for (int i = 0; i < pYrSize; i++)	{
    for (int j = 0; j < bFiltsZ2; j++)		{
      k1--;
      lenWh [k1][0] = iShiftRightRound(xStop, i); //ceil(xStop / pow(2.0, (double) i));
      lenWh [k1][1] = iShiftRightRound(yStop, i); //ceil(yStop / pow(2.0, (double) i));
    }
  }
  lenWh [k1 - 1][0] = iShiftRightRound(xStop, pYrSize); // ceil(xStop / pow(2.0, pYrSize));
  lenWh [k1 - 1][1] = iShiftRightRound(yStop, pYrSize); // ceil(yStop / pow(2.0, pYrSize));
  
  
  k1=elementPyrOrg;
  for (int i = 0; i < pYrSize * bFiltsZ2 + 2;i++)  {
    if (subbandUsed[k1]==TRUE)   {
      pyrOrgArr[k1].resize (lenWh[k1][0] * lenWh[k1][1]);
      pyrDistArr[k1].resize(lenWh[k1][0] * lenWh[k1][1]);
    }
    else {
      pyrOrgArr[k1].resize (1);
      pyrDistArr[k1].resize(1);
      
    }
    k1--;
  }
  
  std::vector<std::vector<double> > pyrOrg (pYrSize + 1);
  std::vector<std::vector<double> > pyrDist(pYrSize + 1);
  
  std::vector<double> lo0FiltOneRow(li0FiltSize * li0FiltSize);
  std::vector<double> hi0FiltOneRow(xFDimHi0 * xFDimHi0);
  
  for (int i = 0; i <  li0FiltSize ; i ++) {
    for (int j = 0; j < li0FiltSize ; j ++) {
      //reshape the filter into matrice
      lo0FiltOneRow[j + i * li0FiltSize]=	lo0filt[j][i] ;
    }
  }
  
  for (int i = 0; i <  xFDimHi0 ; i ++)	{
    for (int j = 0; j <  yFDimHi0 ; j ++) {
      hi0FiltOneRow[j + i *  xFDimHi0]=	hi0filt[j][i] ;
    }
  }
  
  // Do wavelet decomposition. This requires the Steerable Pyramid. You can
  // use your own wavelet as long as the cell arrays org and dist contain
  //corresponding subbands from the reference and the distorted images respectively.
  //for orginal
  if (subbandUsed[elementPyrOrg]) {
    Convolve::internalReduce(&image_org[0], xDim,  yDim, &hi0FiltOneRow[0], &temp[0],  xFDimHi0,  yFDimHi0,	xStart,  xStep,  xStop, yStart,  yStep,  yStop,	&pyrOrgArr[elementPyrOrg][0]); //hio0
  }
  
  Convolve::internalReduce(&image_org[0], xDim,  yDim, &lo0FiltOneRow[0], &temp[0],  xFDimLo0,  y_fdim_lo0,	xStart,  xStep,  xStop, yStart,  yStep,  yStop,	&lo0_org[0]);
  
  elementPyrOrg--;
  buildSpyrLevs(&lo0_org[0], pYrSize, inp0->m_width[Y_COMP],inp0->m_height[Y_COMP], pyrOrg, pYrSize,  elementPyrOrg, pyrOrgArr, subbandUsed);
  
  //for distorted
  if (subbandUsed[elementPyrDist]) {
    Convolve::internalReduce(&image_dist[0], xDim,  yDim, 
                             &hi0FiltOneRow[0], &temp[0],  xFDimHi0,  yFDimHi0,
                             xStart,  xStep,  xStop, 
                             yStart,  yStep,  yStop,
                             &pyrDistArr[elementPyrDist][0]); //hio0
  }
  
  Convolve::internalReduce(&image_dist[0], xDim,  yDim, 
                           &lo0FiltOneRow[0], &temp[0],  xFDimLo0,  y_fdim_lo0,
                           xStart,  xStep,  xStop, 
                           yStart,  yStep,  yStop,
                           &lo0_dist[0]);
  
  elementPyrDist--;
  buildSpyrLevs(&lo0_dist[0], pYrSize, inp1->m_width[Y_COMP],inp1->m_height[Y_COMP], pyrDist,  pYrSize,  elementPyrDist, pyrDistArr, subbandUsed);
  
  //================================
  // calculate the parameters of the distortion channel
  std::vector<std::vector<std::vector<double> > >   gAll     (pYrSize * bFiltsZ2 + 2);
  std::vector<std::vector<std::vector<double> > >   VvAll    (pYrSize * bFiltsZ2 + 2);
  std::vector<std::vector<int> >                    lenWhGAll(pYrSize * bFiltsZ2 + 2);
  
  for (int i=0;i<pYrSize*bFiltsZ2+2;i++)
    lenWhGAll[i].resize(2);
  
  for (int ii=0; ii<sizeSubBand;ii++)	{
    int sub = subbands [ii];
    int  newSizeYWidth = (int) (floor(lenWh[sub][0]/((double)M))*(double)M);
    int newSizeYHeight = (int) (floor(lenWh[sub][1]/((double)M))*(double)M);
    
    lenWhGAll [sub][0] = (int) (newSizeYWidth  / (double) M);  
    lenWhGAll [sub][1] = (int) (newSizeYHeight / (double) M); 
    gAll[sub].resize(lenWhGAll [sub][0]);
    VvAll[sub].resize( lenWhGAll [sub][0]);
    for (int jj=0;jj< lenWhGAll [sub][0];jj++)		{
      gAll[sub][jj].resize( lenWhGAll [sub][1]);
      VvAll[sub][jj].resize( lenWhGAll [sub][1]);
    }
  }
  
  //calculate the parameters of the distortion channel
  vifSubEstM (pyrOrgArr, pyrDistArr,  subbands,  M,  lenWh,sizeSubBand, gAll,  VvAll, lenWhGAll);
  
  std::vector<std::vector<std::vector<double> > >  ssArr(pYrSize * bFiltsZ2 + 2);
  std::vector<std::vector<std::vector<double> > >  cuArr(pYrSize * bFiltsZ2 + 2);
  std::vector<std::vector<double> >                lArr (pYrSize * bFiltsZ2 + 2);
  
  for (int ii=0;ii<sizeSubBand;ii++)  {
    int i = subbands[ii];
    int newSizeYWidth  = (int) ((floor(lenWh[i][0] / ((double) M))) * (double) M);
    int newSizeYHeight = (int) ((floor(lenWh[i][1] / ((double) M))) * (double) M);
    
    cuArr[i].resize(M * M);
    ssArr[i].resize((int(newSizeYWidth / (double) M)));
    lArr[i].resize(M * M);
    
    for (int j=0;j<M * M;j++) {
      cuArr[i][j].resize( M * M );
    }
    
    for (int j=0;j<int(newSizeYWidth / (double) M);j++)  {
      ssArr[i][j].resize( int(newSizeYHeight / (double) M));
    }
  }
  
  //calculate the parameters of the reference image
  refParamsVecGSM (pyrOrgArr ,   subbands,  M,  lenWh,  sizeSubBand,  ssArr,  lArr,  cuArr);
  
  // compute reference and distorted image information from each subband
  std::vector<double> num(sizeSubBand);
  std::vector<double> den(sizeSubBand);
  
  for (int i=0;i<sizeSubBand;i++)  {
    int sub=subbands[i];
    //how many eigenvalues to sum over. default is 1.
    //compute the size of the window used in the distortion channel estimation, and use it to calculate the offset from subband borders
    //we do this to avoid all coefficients that may suffer from boundary
    //effects
    int lev     = (int) ceil((sub)/6.0);
    int winSize = (1 << lev) + 1; 
    int offset  = (winSize - 1)>> 1;
    offset = (int) (ceil(offset / (double) M));
    int sizeGWidth  = lenWhGAll[sub][0] - 2 * offset;
    int sizeGHeight = lenWhGAll[sub][1] - 2 * offset;
    
    std::vector<std::vector<double> >  g (sizeGWidth);
    std::vector<std::vector<double> >  vv(sizeGWidth);
    std::vector<std::vector<double> >  ss(sizeGWidth);
    
    for (int jj = 0; jj < sizeGWidth; jj++)    {
      g [jj].resize(sizeGHeight);
      vv[jj].resize(sizeGHeight);
      ss[jj].resize(sizeGHeight);
    }
    
    int w1=0;
    for (int w = offset; w < lenWhGAll[sub][0]-offset; w++)  {
      int h1=0;	
      for (int h=offset;h<lenWhGAll[sub][1]-offset; h++)  {
        // select only valid portion of the output.
        g [w1][h1] = gAll [sub][w][h]; //(offset+1:end-offset,offset+1:end-offset);
        vv[w1][h1] = VvAll[sub][w][h]; //(offset+1:end-offset,offset+1:end-offset);
        ss[w1][h1] = ssArr[sub][w][h]; //(offset+1:end-offset,offset+1:end-offset);
        h1++;
      }
      w1++;
    }
    
    // VIF
    double  temp1=0; double temp2=0;
    for (int j=0;j<M * M; j++)  {
      temp1 += sumSumLog2GGSSLambdaDivVV( g, sizeGWidth, sizeGHeight,  ss, lArr[sub][j],  sigma_nsq,  vv);
      temp2 += sumSumLog2SSLambdaDivSigma (ss,sizeGWidth, sizeGHeight,lArr[sub][j],sigma_nsq); 
    }
    num[i]=temp1;
    den[i]=temp2;
    
    //compuate IFC and normalize to size of the image
    double ifc1=0;
    double sumNum=0;
    double sumDen=0;
    for (int j=0;j<sizeSubBand; j++)
    {
      sumNum += num[j];
      sumDen += den[j];
      ifc1   += num[j] / (inp0->m_width[Y_COMP] * inp0->m_height[Y_COMP]);
    }
    
    m_vifFrame = sumNum / sumDen;
    m_vifFrameStats.updateStats(m_vifFrame);
    
  }
}

void DistortionMetricVIF::computeMetric (Frame* inp0, Frame* inp1, int component)
{
  printf("computeMetric DeltaE in One component is not possible\n");
}
//needs to be updated
void DistortionMetricVIF::reportMetric ()
{
  printf(" %0.6f  ",m_vifFrame);
}

void DistortionMetricVIF::reportSummary  ()
{
  printf(" %0.6f  ", m_vifFrameStats.getAverage());
}

void DistortionMetricVIF::reportMinimum  ()
{
  printf(" %0.6f  ", m_vifFrameStats.minimum);
}

void DistortionMetricVIF::reportMaximum  ()
{
  printf(" %0.6f  ", m_vifFrameStats.maximum);
}

void DistortionMetricVIF::printHeader()
{
  printf ("Frame_VIF  ");
}

void DistortionMetricVIF::printSeparator(){
  printf("-------------");
}
