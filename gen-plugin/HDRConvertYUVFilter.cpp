/* The copyright in this software is being made available under the BSD
 * License, included below. This software may be subject to other third party
 * and contributor rights, including patent rights, and no such rights are
 * granted under this license.
 *
 * <OWNER> = Yanan Zhao <arthurchiao@hotmail.com>
 * <YEAR> = 2016
 *
 * Copyright (c) 2016, Yanan Zhao
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
 *  * Neither the name of the <ORGANIZATION> nor the names of its contributors
 *may
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

#include <time.h>
#include <string.h>
#include <math.h>
#include "ProjectParameters.H"
#include "HDRConvertYUV.H"
#include "VapourSynth.h"
#include "VSHelper.h"
#include "VSScript.h"

class YUVPlugin
{
  public:
    static void VS_CC create(const VSMap *in, VSMap *out, void *user_data_ptr,
                      VSCore *core, const VSAPI *vsapi);

  private:
    static void VS_CC init_filter(VSMap *in, VSMap *out, void **instanceData,
                                  VSNode *node, VSCore *core,
                                  const VSAPI *vsapi);
    static const VSFrameRef *VS_CC
        get_frame(int n, int activationReason, void **instanceData,
                  void **frameData, VSFrameContext *frameCtx, VSCore *core,
                  const VSAPI *vsapi);

    static void VS_CC
        free_filter(void *instanceData, VSCore *core, const VSAPI *vsapi);

    YUVPlugin();
    virtual ~YUVPlugin();

  private:
    int m_width;
    int m_height;
};

void VS_CC YUVPlugin::init_filter(VSMap *in, VSMap *out, void **instanceData,
                                  VSNode *node, VSCore *core,
                                  const VSAPI *vsapi)
{
    std::cout << "this is " << __FILE__ << ": " << __func__ << std::endl;
    return;
}

const VSFrameRef *VS_CC
    YUVPlugin::get_frame(int n, int activationReason, void **instanceData,
                         void **frameData, VSFrameContext *frameCtx,
                         VSCore *core, const VSAPI *vsapi)
{
    std::cout << "this is " << __FILE__ << ": " << __func__ << std::endl;
    return NULL;
}

void VS_CC
    YUVPlugin::free_filter(void *instanceData, VSCore *core, const VSAPI *vsapi)
{
    return;
}

void VS_CC YUVPlugin::create(const VSMap *in, VSMap *out, void *userData,
                             VSCore *core, const VSAPI *vsapi)
{
    std::cout << "this is " << __FILE__ << ": " << __func__ << std::endl;
    std::cout << "creating HDRConvertYUV Filter ..." << std::endl;

    assert(in != NULL);
    assert(out != NULL);
    assert(core != NULL);
    assert(vsapi != NULL);

    int err;
    int width = vsapi->propGetInt(in, "w", 0, &err);
    int height = vsapi->propGetInt(in, "h", 0, &err);
    std::cout << "m_width: " << width << ", m_height: " << height << std::endl;

    /***********************************************************************/
    /* init plugin */
    /***********************************************************************/
    ProjectParameters params;

    std::cout << "m_width: " << width << ", m_height: " << height << std::endl;
    /* params->m_inputFile; */
    /* params->m_outputFile; */
    /* params->m_source; */
    /* params->m_output; */
    /* params->m_frameSkip;     //! Frame skipping for input */

    /* params->m_srcNormalScale; */
    /* params->m_outNormalScale; */
    /* params->m_srcMinValue;               //!< Brightness range value (min) */
    /* params->m_srcMaxValue;               //!< Brightness range value (max) */
    /* params->m_outMinValue;               //!< Brightness range value (min) */
    /* params->m_outMaxValue;               //!< Brightness range value (max) */

    /* params->m_transformPrecision; */
    /* params->m_useSingleTransferStep;     //!< Use single or multiple steps
     * (for normalization) when applying PQ */
    /* params->m_filterInFloat;             //!< Use floating precision for
     * filtering */
    /* params->m_enableTFunctionLUT;        //!< Use Transfer function LUT for
     * some operations */
    /* params->m_enableTFLUTs;              //!< Use Transfer function LUTs */
    /* params->m_chromaDownsampleFilter;    //!< Chroma downsampling filter */
    /* params->m_chromaUpsampleFilter;      //!< Chroma upsampling filter */
    /* params->m_outputSinglePrecision;     //!< Set output, for OpenEXR files,
     * to single instead of half precision */
    /* params->m_useMinMax; */
    /* params->m_useHighPrecisionTransform; //!< High precision transform for
     * BT.2020 (proper inverse) */
    /* params->m_addNoise;                  //! Add noise to the input signal */
    /* params->m_noiseVariance;             //! Noise variance */
    /* params->m_noiseMean;                 //! Noise Mean */
    /* params->m_closedLoopConversion; */
    /* params->m_closedLoopIterations; */
    /* params->m_linearDownConversion; */
    /* params->m_useAdaptiveUpsampling; */
    /* params->m_useAdaptiveDownsampling; */
    /* params->m_rgbDownConversion; */
    /* params->m_bUseChromaDeblocking; */
    /* params->m_bUseWienerFiltering; */
    /* params->m_bUseNLMeansFiltering; */
    /* params->m_bUse2DSepFiltering; */
    /* params->m_b2DSepMode; */

    /* params->m_cropOffsetLeft; */
    /* params->m_cropOffsetTop; */
    /* params->m_cropOffsetRight; */
    /* params->m_cropOffsetBottom; */

    /* params->m_toneMapping; */
    /* params->m_tmParams; */
    /* params->m_tfParams; */
    /* params->m_ctParams; */

    HDRConvertYUV *plugin = new HDRConvertYUV(&params);

    std::cout << "m_width: " << width << ", m_height: " << height << std::endl;
    /***********************************************************************/
    /* init plugin finish */
    /***********************************************************************/

    if (plugin) {
        vsapi->createFilter(
            in, out,
            "HDRConvertYUVFilter", // plugin->filter_name.c_str(),
            &init_filter, &get_frame, &free_filter,
            NULL, // filter mode
            NULL, // filter flags
            plugin, core);
    }

    std::cout << "Filter created successful" << std::endl;
}

/* static void VS_CC _create(const VSMap *in, VSMap *out, void *userData, */
/*                              VSCore *core, const VSAPI *vsapi) */
/* { */
/*     YUVPlugin::create(in, out, userData, core, vsapi); */
/* } */

VS_EXTERNAL_API(void) VapourSynthPluginInit(VSConfigPlugin config_fnc,
                                            VSRegisterFunction register_fnc,
                                            VSPlugin *plugin)
{
    config_fnc("hdrconv", "hdrc",
               "HDR Converter, " "0.0.1", VAPOURSYNTH_API_VERSION, 1,
               plugin);

    register_fnc("YUVConverter",
                 "clip:clip;"
                 "w:int:opt;"
                 "h:int:opt;",
                 &YUVPlugin::create,
                 0,
                 plugin);
}
