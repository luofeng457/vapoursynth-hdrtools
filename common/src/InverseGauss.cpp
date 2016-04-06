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


#include "Global.H"
#include "InverseGauss.H"
//http://www.sanfoundry.com/cpp-program-finds-inverse-graph-matrix/
//The code is modified for the VIF metric

void InverseGauss::inverseMatrix(std::vector<std::vector<double> > &input, int n, std::vector<std::vector<double> >  &output)
{
  std::vector<std::vector<double> >  a;
  a.resize(2 * n);
  
  for (int i = 0; i < 2 * n;i++) {
    a[i].resize(2 * n);
    for(int j = 0; j < 2 * n;j++) {
      if(j > n - 1 || i > n - 1)
        a[i][j]=0;
      else if(i < n && j < n)
        a[i][j]=input[i][j];
    }
  }
  for (int i = 0; i < n; i++) {
    for (int j = 0; j <  2*n; j++) {
      if (j == (i + n)) {
        a[i][j]=1;
      }
    }
  }
  
  for (int i = n-1; i > 0; i--)
  {
    if (a[i-1][1] < a[i][1]) {
      for(int j = 0; j < n * 2; j++) {
        swap(&a[i][j], &a[i - 1][j]);
      }
    }
  }
  
  for (int i = 0; i < n; i++) {
    for (int j = 0; j < n * 2; j++) {
      if (j != i) {
        double d = a[j][i] / a[i][i];
        for (int k =0 ; k < n * 2; k++) {
          a[j][k] = a[j][k] - (a[i][k] * d);
        }
      }
    }
  }
  
  for (int i = 0; i < n; i++)  {
    double d=a[i][i];
    for (int j = 0; j < n * 2; j++) {
      a[i][j] = a[i][j] / d;
      if (j > n - 1) {
        output[i][j - n]=a[i][j];
      }
    }
  }
}

