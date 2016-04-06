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


/* 
 ;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
 ;;;  File: convolve.c
 ;;;  Author: Eero Simoncelli
 ;;;  Description: General convolution code for 2D images
 ;;;  Creation Date: Spring, 1987.
 ;;;  MODIFICATIONS:
 ;;;     10/89: approximately optimized the choice of register vars on SPARCS.
 ;;;      6/96: Switched array types to double double.
 ;;;      2/97: made more robust and readable.  Added STOP arguments.
 ;;;      8/97: Bug: when calling internalReduce with edges in {reflect1,repeat,
 ;;;            extend} and an even filter dimension.  Solution: embed the filter
 ;;;            in the upper-left corner of a filter with odd Y and X dimensions.
 ;;;  ----------------------------------------------------------------
 ;;;    Object-Based Vision and Image Understanding System (OBVIUS),
 ;;;      Copyright 1988, Vision Science Group,  Media Laboratory,  
 ;;;              Massachusetts Institute of Technology.
 ;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
 */

#include <stdio.h>
#include <math.h>
#include "Convolve.H"
#include <string.h>


#define sgn(a)  ( ((a)>0)?1:(((a)<0)?-1:0) )
#define clip(a,mn,mx)  ( ((a)<(mn))?(mn):(((a)>=(mx))?(mx-1):(a)) )

/*
 --------------------------------------------------------------------
 Correlate FILT with IMAGE, subsampling according to START, STEP, and
 STOP parameters, with values placed into RESULT array.  RESULT
 dimensions should be ceil((stop-start)/step).  TEMP should be a
 pointer to a temporary double array the size of the filter.
 EDGES is a string specifying how to handle boundaries -- see edges.c.
 The convolution is done in 9 sections, where the border sections use
 specially computed edge-handling filters (see edges.c). The origin 
 of the filter is assumed to be (floor(xFDim/2), floor(yFDim/2)).
 ------------------------------------------------------------------------ */

/* abstract out the inner product computation */
void Convolve::INPROD(int XCNR,int YCNR,int xDim, int xFDim, int filtSize, int resPos, double* result,  double* temp, double* image)
{
  double sum       = 0.0; 
  int    imPos     = YCNR * xDim + XCNR;
  int    filtPos   = 0;
  int    xFiltStop = xFDim;
  for (;  xFiltStop <= filtSize; imPos += (xDim - xFDim), xFiltStop += xFDim) {
    for (; filtPos < xFiltStop; filtPos++, imPos++) {
      sum+= image[imPos] * temp[filtPos]; 
    }
  }
  result[resPos] = sum;
		
}
int Convolve::internalReduce(double* image, int xDim, int yDim, 
                             const double *filt, double *temp, int xFDim, int yFDim,
                             int xStart, int xStep, int xStop, 
                             int yStart, int yStep, int yStop,
                             double* result)
{ 
  //double sum;
  int xPos     = xFDim * yFDim;
  int filtSize = xFDim * yFDim;
  int yPos, resPos;
  int yCtrStop = yDim - ((yFDim == 1) ? 0 : yFDim);
  int xCtrStop = xDim - ((xFDim == 1) ? 0 : xFDim);
  int xResDim = (xStop - xStart + xStep - 1) / xStep;
  int xCtrStart = ((xFDim == 1) ? 0 : 1);
  int yCtrStart = ((yFDim == 1) ? 0 : 1);
  int xFMid = xFDim / 2;
  int yFMid = yFDim / 2;
  int baseResPos;
  // fptr reflect = edge_function(edges);  /* look up edge-handling function */
  //  if (!reflect) return(-1);
  
  /* shift start/stop coords to filter upper left hand corner */
  xStart -= xFMid;   
  yStart -= yFMid;
  xStop  -= xFMid;   
  yStop  -= yFMid;
  
  if (xStop < xCtrStop) 
    xCtrStop = xStop;
  if (yStop < yCtrStop) 
    yCtrStop = yStop;
  
  /* TOP ROWS */
  for (resPos = 0, yPos = yStart; yPos < yCtrStart; yPos += yStep)  {
  /* TOP-LEFT CORNER */
    for (xPos = xStart; xPos < xCtrStart; xPos += xStep, resPos++)    {
      reflect1(filt, xFDim, yFDim, xPos - 1, yPos - 1, temp, 0);
      INPROD(0, 0, xDim, xFDim, filtSize, resPos, result, temp, image);
    }
    
    reflect1(filt,xFDim,yFDim,0,yPos-1,temp, 0);

    /* TOP EDGE */
    for (; xPos < xCtrStop; xPos += xStep, resPos++) 
      INPROD(xPos, 0, xDim, xFDim, filtSize, resPos, result, temp, image);

    /* TOP-RIGHT CORNER */
    for (;xPos < xStop; xPos += xStep, resPos++) {
      reflect1(filt, xFDim, yFDim, xPos - xCtrStop + 1, yPos - 1, temp, 0);
      INPROD(xCtrStop, 0, xDim, xFDim, filtSize, resPos, result, temp, image);
    }
  } /* end TOP ROWS */   
  
  yCtrStart = yPos;			      /* hold location of top */

   /* LEFT EDGE */
  for (baseResPos = resPos, xPos = xStart; xPos < xCtrStart; xPos += xStep, baseResPos++) {
    reflect1(filt, xFDim, yFDim, xPos - 1, 0, temp, 0);
    for (yPos = yCtrStart, resPos = baseResPos; yPos < yCtrStop; yPos += yStep, resPos += xResDim)
      INPROD(0, yPos, xDim, xFDim, filtSize, resPos, result, temp, image);
  }
  
  reflect1(filt,xFDim,yFDim,0,0,temp, 0);
  /* CENTER */
  for (; xPos < xCtrStop; xPos += xStep, baseResPos++) {
    for (yPos = yCtrStart, resPos = baseResPos; yPos < yCtrStop; yPos += yStep, resPos += xResDim)
      INPROD(xPos, yPos, xDim, xFDim, filtSize, resPos, result, temp, image);
  }
  /* RIGHT EDGE */
  for (; xPos < xStop; xPos += xStep, baseResPos++)  {
    reflect1(filt, xFDim, yFDim, xPos - xCtrStop + 1, 0, temp, 0);    
    for (yPos = yCtrStart, resPos = baseResPos; yPos<yCtrStop; yPos += yStep, resPos += xResDim)
      INPROD(xCtrStop, yPos, xDim, xFDim, filtSize, resPos, result, temp, image);
  }
  
  /* BOTTOM ROWS */
  for (resPos -= (xResDim - 1); yPos < yStop; yPos += yStep) {
  /* BOTTOM-LEFT CORNER */
    for (xPos = xStart; xPos < xCtrStart; xPos+=xStep, resPos++) {
      reflect1(filt, xFDim, yFDim, xPos - 1, yPos - yCtrStop + 1, temp, 0);
      INPROD(0, yCtrStop, xDim, xFDim, filtSize, resPos, result,  temp,  image);
    }
    
    reflect1(filt, xFDim, yFDim, 0, yPos - yCtrStop + 1, temp, 0);
    /* BOTTOM EDGE */
    for (;xPos < xCtrStop; xPos += xStep, resPos++) 
      INPROD(xPos, yCtrStop, xDim, xFDim, filtSize, resPos, result, temp, image);

     /* BOTTOM-RIGHT CORNER */
    for (; xPos < xStop; xPos += xStep, resPos++) {
      reflect1(filt, xFDim, yFDim, xPos - xCtrStop + 1, yPos - yCtrStop + 1,temp, 0);
      INPROD(xCtrStop, yCtrStop, xDim, xFDim, filtSize, resPos, result, temp, image);
    }
  } /* end BOTTOM */
  return(0);
} /* end of internalReduce */


/*
 --------------------------------------------------------------------
 Upsample IMAGE according to START,STEP, and STOP parameters and then
 convolve with FILT, adding values into RESULT array.  IMAGE
 dimensions should be ceil((stop-start)/step).  See
 description of internalReduce (above).
 
 WARNING: this subroutine destructively modifies the RESULT array!
 ------------------------------------------------------------------------ */

/* abstract out the inner product computation */

int Convolve::reflect1( const double * filt,int xDim,int yDim,int xPos,int yPos,double* result,int rOrE)
{
  int filtSz = xDim * yDim;
  int yFilt,xFilt, yRes, xRes;
  int xBase  = (xPos > 0) ? (xDim - 1):0;
  int yBase  = xDim * ((yPos > 0) ? (yDim - 1) : 0); 
  int xOverhang = (xPos > 0) ? (xPos - 1) : ((xPos < 0) ? (xPos + 1) : 0);
  int yOverhang = xDim * ((yPos > 0) ? (yPos - 1) : ((yPos < 0) ? (yPos + 1) : 0));
  int i;
  int mXPos = (xPos < 0) ? (xDim / 2) : ((xDim - 1) / 2);
  int mYPos = xDim * ((yPos < 0) ? (yDim / 2) : ((yDim - 1) / 2));
  
  for (i=0; i<filtSz; i++) 
    result[i] = 0.0;
  
  if (rOrE == 0) {
    for (yFilt = 0, yRes = yOverhang - yBase; yFilt < filtSz; yFilt += xDim, yRes += xDim) {
      for (xFilt = yFilt, xRes = xOverhang - xBase; xFilt < yFilt + xDim; xFilt++, xRes++) {
        result[iAbs(yBase - iAbs(yRes)) + iAbs(xBase - iAbs(xRes))] += filt[xFilt];
      }
    }
  }
  else {
    yOverhang = iAbs(yOverhang); 
    xOverhang = iAbs(xOverhang);
    for (yRes=yBase, yFilt = yBase - yOverhang; yFilt > yBase - filtSz; yFilt -= xDim, yRes -= xDim)    {
      for (xRes=xBase, xFilt = xBase - xOverhang; xFilt > xBase - xDim; xRes--, xFilt--)
        result[iAbs(yRes) + iAbs(xRes)] += filt[iAbs(yFilt) + iAbs(xFilt)];
      
      if ((xOverhang != mXPos) && (xPos != 0)) {
        for (xRes = xBase, xFilt = xBase - 2 * mXPos + xOverhang; xFilt > xBase - xDim; xRes--, xFilt--)
          result[iAbs(yRes) + iAbs(xRes)] += filt[iAbs(yFilt) + iAbs(xFilt)];
      }
    }
    if ((yOverhang != mYPos) && (yPos != 0)) {
      for (yRes = yBase, yFilt = yBase - 2 * mYPos + yOverhang; yFilt > yBase - filtSz; yFilt -= xDim, yRes -= xDim)  {
        for (xRes = xBase, xFilt = xBase - xOverhang; xFilt > xBase - xDim; xRes--, xFilt--)
          result[iAbs(yRes) + iAbs(xRes)] += filt[iAbs(yFilt) + iAbs(xFilt)];
          
        if  ((xOverhang != mXPos) && (xPos != 0)) {
          for (xRes=xBase, xFilt = xBase - 2 * mXPos + xOverhang; xFilt > xBase - xDim; xRes--, xFilt--)
            result[iAbs(yRes) + iAbs(xRes)] += filt[iAbs(yFilt) + iAbs(xFilt)];
        }
      }
    }
  }
  return(0);
  
}
