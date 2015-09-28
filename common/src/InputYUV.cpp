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
 * \file InputYUV.cpp
 *
 * \brief
 *    InputYUV class C++ file for allowing input of concatenated YUV files
 *
 * \author
 *     - Alexis Michael Tourapis         <atourapis@apple.com>
 *
 *************************************************************************************
 */

//-----------------------------------------------------------------------------
// Include headers
//-----------------------------------------------------------------------------

#include <string.h>
#include "InputYUV.H"
#include "Global.H"
#include "IOFunctions.H"

//-----------------------------------------------------------------------------
// Macros/Defines
//-----------------------------------------------------------------------------

//-----------------------------------------------------------------------------
// Constructor/destructor
//-----------------------------------------------------------------------------

InputYUV::InputYUV(IOVideo *inputFile, FrameFormat *format) {
  m_isFloat                   = FALSE;
  format->m_isFloat           = m_isFloat;
  m_colorSpace                = format->m_colorSpace;
  m_colorPrimaries            = format->m_colorPrimaries;
  m_transferFunction          = format->m_transferFunction;
  m_systemGamma               = format->m_systemGamma;
  m_sampleRange               = format->m_sampleRange;
  m_chromaFormat              = format->m_chromaFormat;
  m_isInterleaved             = inputFile->m_isInterleaved;
  m_isInterlaced              = format->m_isInterlaced;
  m_chromaLocation[FP_TOP]    = format->m_chromaLocation[FP_TOP];
  m_chromaLocation[FP_BOTTOM] = format->m_chromaLocation[FP_BOTTOM];
  if (m_isInterlaced == FALSE && m_chromaLocation[FP_TOP] != m_chromaLocation[FP_BOTTOM]) {
    printf("Progressive Content. Chroma Type Location needs to be the same for both fields.\n");
    printf("Resetting Bottom field chroma location from type %d to type %d\n", m_chromaLocation[FP_BOTTOM], m_chromaLocation[FP_TOP]);
    m_chromaLocation[FP_BOTTOM] = format->m_chromaLocation[FP_BOTTOM] = m_chromaLocation[FP_TOP];    
  }

  m_pixelFormat               = format->m_pixelFormat;
  m_frameRate                 = format->m_frameRate;
  m_bitDepthComp[Y_COMP]      = format->m_bitDepthComp[Y_COMP];
  m_bitDepthComp[U_COMP]      = format->m_bitDepthComp[U_COMP];
  m_bitDepthComp[V_COMP]      = format->m_bitDepthComp[V_COMP];
  m_height[Y_COMP]            = format->m_height[Y_COMP];
  m_width [Y_COMP]            = format->m_width [Y_COMP];

  if (format->m_chromaFormat == CF_420) {
    m_height [V_COMP] = m_height [U_COMP] = m_height [Y_COMP] >> 1;
    m_width  [V_COMP] = m_width  [U_COMP] = m_width  [Y_COMP] >> 1;
  }
  else if (format->m_chromaFormat == CF_422) {
    m_height [V_COMP] = m_height [U_COMP] = m_height [Y_COMP];
    m_width  [V_COMP] = m_width  [U_COMP] = m_width  [Y_COMP] >> 1;
  }
  else {
    m_height [V_COMP] = m_height [U_COMP] = m_height [Y_COMP];
    m_width  [V_COMP] = m_width  [U_COMP] = m_width  [Y_COMP];
  }

  m_compSize[Y_COMP] = m_height[Y_COMP] * m_width[Y_COMP];
  m_compSize[U_COMP] = m_height[U_COMP] * m_width[U_COMP];
  m_compSize[V_COMP] = m_height[V_COMP] * m_width[V_COMP];

  m_size = m_compSize[Y_COMP] + m_compSize[U_COMP] + m_compSize[V_COMP];

  if (m_isInterleaved) {
    m_iBuf  = new uint8 [(int) m_size * format->m_picUnitSizeShift3];
  }

  m_buf  = new uint8 [(int) m_size * format->m_picUnitSizeShift3];

  if (format->m_picUnitSizeShift3 > 1) {
    m_ui16Data = new uint16[(int) m_size];
    m_ui16Comp[Y_COMP] = m_ui16Data;
    m_ui16Comp[U_COMP] = m_ui16Comp[Y_COMP] + m_compSize[Y_COMP];
    m_ui16Comp[V_COMP] = m_ui16Comp[U_COMP] + m_compSize[U_COMP];
    m_data = NULL;
    m_comp[Y_COMP] = NULL;
    m_comp[U_COMP] = NULL;
    m_comp[V_COMP] = NULL;
    m_floatData = NULL;
    m_floatComp[Y_COMP] = NULL;
    m_floatComp[U_COMP] = NULL;
    m_floatComp[V_COMP] = NULL;
  }
  else {
    m_ui16Data = NULL;
    m_ui16Comp[Y_COMP] = NULL;
    m_ui16Comp[U_COMP] = NULL;
    m_ui16Comp[V_COMP] = NULL;
    m_data = new imgpel[(int) m_size];
    m_comp[Y_COMP] = m_data;
    m_comp[U_COMP] = m_comp[Y_COMP] + m_compSize[Y_COMP];
    m_comp[V_COMP] = m_comp[U_COMP] + m_compSize[U_COMP];
    m_floatData = NULL;
    m_floatComp[Y_COMP] = NULL;
    m_floatComp[U_COMP] = NULL;
    m_floatComp[V_COMP] = NULL;
  }
}

InputYUV::~InputYUV() {
  if (m_isInterleaved) {
    if (m_iBuf != NULL) {
      delete[] m_iBuf;
      m_iBuf = NULL;
    }
  }

  if (m_buf != NULL) {
    delete[] m_buf;
    m_buf = NULL;
  }
  if (m_data != NULL) {
    m_comp[Y_COMP] = NULL;
    m_comp[U_COMP] = NULL;
    m_comp[V_COMP] = NULL;
    
    delete [] m_data;
    m_data = NULL;
  }
  
  if (m_ui16Data != NULL) {
    m_ui16Comp[Y_COMP] = NULL;
    m_ui16Comp[U_COMP] = NULL;
    m_ui16Comp[V_COMP] = NULL;

    delete [] m_ui16Data;
    m_ui16Data = NULL;
  }
  
  if (m_floatData != NULL) {
    m_floatComp[Y_COMP] = NULL;
    m_floatComp[U_COMP] = NULL;
    m_floatComp[V_COMP] = NULL;
    
    delete [] m_floatData;
    m_floatData = NULL;
  }
  clear();
}

//-----------------------------------------------------------------------------
// Private methods
//-----------------------------------------------------------------------------

int InputYUV::readData (int vfile,  FrameFormat *source, uint8 *buf) {
  uint8 *curBuf = buf;
  int readSize = source->m_picUnitSizeShift3 * source->m_width[Y_COMP];
  int i, j;
  for (i = 0; i < source->m_height[Y_COMP]; i++) {
    if (mm_read(vfile, curBuf, readSize) != readSize) {
      //printf ("readData: cannot read %d bytes from input file, unexpected EOF!\n", source->m_width[Y_COMP]);
      return 0;
    }
    curBuf += readSize;
  }
  
  if (source->m_chromaFormat != CF_400) {
    for (j = U_COMP; j <= V_COMP; j++) {
      readSize = source->m_picUnitSizeShift3 * source->m_width[ (int) j];
      for (i = 0; i < source->m_height[(int) j]; i++) {
        if (mm_read(vfile, curBuf, readSize) != readSize) {
          printf ("readData: cannot read %d bytes from input file, unexpected EOF!\n", source->m_width[1]);
          return 0;
        }
        curBuf += readSize;
      }
    }
  }
  return 1;
}

int InputYUV::readData (int vfile, int framesizeInBytes, uint8 *buf)
{
  if (mm_read(vfile, buf, (int) framesizeInBytes) != (int) framesizeInBytes)  {
    printf ("read_one_frame: cannot read %d bytes from input file, unexpected EOF!\n", (int) framesizeInBytes);
    return 0;
  }
  else  {
    return 1;
  }
}

int64 InputYUV::getFrameSizeInBytes(FrameFormat *source, bool isInterleaved)
{
  uint32 symbolSizeInBytes = source->m_picUnitSizeShift3;
  int64 framesizeInBytes;
  
  const int bytesY  = source->m_compSize[Y_COMP];
  const int bytesUV = source->m_compSize[U_COMP];
  
  if (isInterleaved == FALSE) {
    framesizeInBytes = (bytesY + 2 * bytesUV) * symbolSizeInBytes;
  }
  else {
    switch (source->m_chromaFormat) {
      case CF_420:
        framesizeInBytes = (bytesY + 2 * bytesUV) * symbolSizeInBytes;
        break;
      case CF_422:
        switch (source->m_pixelFormat) {
          case PF_YUYV:
          case PF_YUY2:
          case PF_YVYU:
          case PF_UYVY:
            framesizeInBytes = (bytesY + 2 * bytesUV) * symbolSizeInBytes;
            break;
          case PF_V210:
          case PF_UYVY10:
            // Pack 12 10-bit samples into each 16 bytes
            //framesizeInBytes = ((bytesY + 2 * bytesUV) / 3) << 2;
            framesizeInBytes = (bytesY / 3) << 3;
            break;
#ifdef __SIM2_SUPPORT_ENABLED__
          case PF_SIM2:
            framesizeInBytes = (bytesY * 3);
            break;
#endif
          default:
            fprintf(stderr, "Unsupported pixel format.\n");
            exit(EXIT_FAILURE);
            break;
        }
        break;
      case CF_444:
        framesizeInBytes = (bytesY + 2 * bytesUV) * symbolSizeInBytes;
        break;
      default:
        fprintf(stderr, "Unknown Chroma Format type %d\n", source->m_chromaFormat);
        exit(EXIT_FAILURE);
        break;
    }
  }
  
  return framesizeInBytes;
}

//-----------------------------------------------------------------------------
// Public methods
//-----------------------------------------------------------------------------


/*!
 ************************************************************************
 * \brief
 *    Reads one new frame from concatenated raw file
 *
 * \param inputFile
 *    Input file to read from
 * \param frameNumber
 *    Frame number in the source file
 * \param f_header
 *    Number of bytes in the source file to be skipped
 * \param frameSkip
 *    Start position in file
 ************************************************************************
 */
int InputYUV::readOneFrame (IOVideo *inputFile, int frameNumber, int fileHeader, int frameSkip) {
  int fileRead = 0;
  int vfile = inputFile->m_fileNum;
  FrameFormat *format = &inputFile->m_format;
  uint32 symbolSizeInBytes = format->m_picUnitSizeShift3;

  const int64 framesizeInBytes = getFrameSizeInBytes(format, inputFile->m_isInterleaved);
  bool isBytePacked = (bool) (inputFile->m_isInterleaved && (format->m_pixelFormat == PF_V210 || format->m_pixelFormat == PF_UYVY10)) ? TRUE : FALSE;

  // Let us seek directly to the current frame
  if (lseek (vfile, fileHeader + framesizeInBytes * (frameNumber + frameSkip), SEEK_SET) == -1)  {
    fprintf(stderr, "readOneFrame: cannot lseek to (Header size) in input file\n");
    exit(EXIT_FAILURE);
  }
  // Here we are at the correct position for the source frame in the file.  
  // Now read it.

  if ((format->m_picUnitSizeOnDisk & 0x07) == 0)  {
    if (isBytePacked == TRUE)
      fileRead = readData (vfile, (int) framesizeInBytes, m_buf);
    else
      fileRead = readData (vfile, format, m_buf);
  
    // If format is interleaved, then perform deinterleaving
    if (m_isInterleaved)
      deInterleave ( &m_buf, &m_iBuf, format, symbolSizeInBytes);

    if (m_bitDepthComp[Y_COMP] == 8)
      imageReformat ( m_buf, m_data, format, symbolSizeInBytes );
    else 
      imageReformatUInt16 ( m_buf, format, symbolSizeInBytes );
  }
  else {
    fprintf (stderr, "readOneFrame (NOT IMPLEMENTED): pic unit size on disk must be divided by 8");
    exit(EXIT_FAILURE);
  }

  return fileRead;
}
//-----------------------------------------------------------------------------
// End of file
//-----------------------------------------------------------------------------

