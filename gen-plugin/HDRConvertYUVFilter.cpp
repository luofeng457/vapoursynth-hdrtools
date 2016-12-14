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

#ifdef TRACE_LINE
#undef TRACE_LINE
#endif
#define TRACE_LINE                                                             \
/* std::cout << "@" << __FILE__ << ": " << __LINE__ << " " << __func__ \ */
/*           << std::endl; */

#ifdef PARAM_DEBUG
static void print_params(ProjectParameters *params);
#endif

struct Plugin
{
    VSNodeRef *node;
    const VSVideoInfo *vi;

    const char *cfgfile; /* cfg file */

    ProjectParameters *params;
    HDRConvert *converter;
};

void VS_CC init_filter(VSMap *in, VSMap *out, void **instanceData, VSNode *node,
                       VSCore *core, const VSAPI *vsapi)
{
    TRACE_LINE

    Plugin *plugin = (Plugin *)*instanceData;

    int numOfOutputs = 1;
    vsapi->setVideoInfo(plugin->vi, numOfOutputs, node);

    /* init hdr converter */
    plugin->params = &ccParams;
    plugin->params->refresh();
    plugin->params->m_silentMode = FALSE;
    plugin->params->readConfigFile((char *)plugin->cfgfile);
    plugin->params->update();
#ifdef PARAM_DEBUG
    print_params(plugin->params);
#endif

    HDRConvert *converter = HDRConvert::create(plugin->params);
    if (!converter) {
        std::cerr << "HDRConvert::create() failed\n";
        return;
    }
    plugin->converter = converter;
    plugin->converter->init(plugin->params);

    TRACE_LINE
}

/**
 * init the member variable m_inputFrame in HDRConvertYUV
 */
void init_input_frame(Plugin *plugin, int n, VSFrameContext *frameCtx,
                      VSCore *core, const VSAPI *vsapi)
{
    assert(plugin != NULL);
    HDRConvertYUV *converter = (HDRConvertYUV *)(plugin->converter);

    // destination YUV frame in this func
    Input *dst = converter->m_inputFrame;

    // sizeof(one_Y_pixel) == sizeof(U) == sizeof(V)
    int bytesPerSample = plugin->vi->format->bytesPerSample;
    assert(bytesPerSample == 1 || bytesPerSample == 2);

    int bitdepth = plugin->vi->format->bitsPerSample;

    // loop over Y/U/V or R/G/B or R/G/B/Alpha planes
    const VSFrameRef *src = vsapi->getFrameFilter(n, plugin->node, frameCtx);
    const VSFormat *format = plugin->vi->format;
    for (int plane = 0; plane < format->numPlanes; plane++) {
        const uint8_t *_src = vsapi->getReadPtr(src, plane);
        int src_stride = vsapi->getStride(src, plane);

        // note that if a frame has the same dimensions and
        // format, the stride is guaranteed to be the same.
        // int dst_stride = src_stride would be fine too in this filter.
        // Since planes may be subsampled you have to query the height of
        // them individually
        int h = vsapi->getFrameHeight(src, plane);
        int w = vsapi->getFrameWidth(src, plane);

        // re-init these fields, override the values parsed from cfg
        dst->m_height[plane] = h;
        dst->m_width[plane] = w;
        dst->m_bitDepthComp[plane] = bitdepth;
        dst->m_picUnitSizeOnDisk = bitdepth;
        dst->m_picUnitSizeShift3 = dst->m_picUnitSizeOnDisk >> 3;

        // should has been initialzed in constructor
        uint8_t *_dst = (bitdepth == 8 ? (uint8_t *)dst->m_comp[plane]
                                       : (uint8_t *)dst->m_ui16Comp[plane]);
        assert(_src != NULL && _dst != NULL);

        // copy from VS (src) to HDR (dst = converter->m_inputFrame)
        int dst_stride = src_stride;
        vs_bitblt(_dst, dst_stride,   // dst
                  _src, src_stride,   // src
                  w * bytesPerSample, // total bytes of each line
                  h                   // plane height
                  );
    }

    dst->m_isFloat = false;
    dst->m_isInterleaved = false;
    dst->m_isInterlaced = false;

    // seems only DPX needs this, Yanan Zhao 2016-12-11 23:43:31
    // dst->m_components;

    dst->m_compSize[Y_COMP] = dst->m_height[Y_COMP] * dst->m_width[Y_COMP];
    dst->m_compSize[U_COMP] = dst->m_height[U_COMP] * dst->m_width[U_COMP];
    dst->m_compSize[V_COMP] = dst->m_height[V_COMP] * dst->m_width[V_COMP];
    dst->m_size = dst->m_compSize[Y_COMP] + dst->m_compSize[U_COMP] +
                  dst->m_compSize[V_COMP];

// we do not need this currently - Yanan Zhao, 2016-12-11 23:43:10
// m_compSize[A_COMP] = ;
// m_height[A_COMP] = ;
// m_width [A_COMP] = ;

/* following fields should have been initialized by parsing cfg file
 * before this stage */
#if 0
    m_colorSpace;                   //!< Color space
    m_colorPrimaries;               //!< Color primaries
    m_chromaFormat;                 //!< 0: 420, 1:422, 2:444
    m_chromaLocation[FP_TOTAL];
    //!< Note that we keep two elements for top and bottom fields.
    m_sampleRange;                  //!< 0: standard, 1: full, 2: SDI
    m_transferFunction;             //!< Supported transfer functions
    m_pixelFormat;
    m_systemGamma;
    m_pixelType[4];                 //!< pixel type (for OpenEXR)
    m_frameRate;
#endif
}

void copy_output_frame(Plugin *plugin, VSFrameRef *dst,
                       VSFrameContext *frameCtx, VSCore *core,
                       const VSAPI *vsapi)
{
    assert(plugin != NULL);
    HDRConvertYUV *converter = (HDRConvertYUV *)(plugin->converter);

    // source YUV frame in this func
    Output *src = converter->m_outputFrame;
#if 0 // debug
    // Input *src = converter->m_inputFrame;
    // Frame *src = converter->getiFrameStore();
#endif

    // sizeof(one_Y_pixel) == sizeof(U) == sizeof(V)
    int bytesPerSample = plugin->vi->format->bytesPerSample;
    assert(bytesPerSample == 1 || bytesPerSample == 2);

    // loop over Y/U/V or R/G/B or R/G/B/Alpha planes
    const VSFormat *format = plugin->vi->format;
    for (int plane = 0; plane < format->numPlanes; plane++) {
        uint8_t *_dst = vsapi->getWritePtr(dst, plane);
        int dst_stride = vsapi->getStride(dst, plane);

        int h = vsapi->getFrameHeight(dst, plane);
        int w = vsapi->getFrameWidth(dst, plane);

        int src_stride = src->m_width[plane] * bytesPerSample;
        uint8_t *_src = (uint8_t *)src->m_ui16Comp[plane];
        assert(_src != NULL && _dst != NULL);

        // copy to vs output frame
        vs_bitblt(_dst, dst_stride,   // dst
                  _src, src_stride,   // src
                  w * bytesPerSample, // total bytes of each line
                  h                   // plane height
                  );
    }

#if 0
    src->m_isFloat;
    src->m_isInterleaved = false;
    src->m_isInterlaced = false;
    src->m_size;                         //!< number of samples
    src->m_compSize[4];                  //!< number of samples in specific component
    src->m_height[4];                    //!< height of a specific component in pixels
    src->m_width [4];                    //!< width of a specific component in pixels
    src->m_colorSpace;                   //!< Color Space
    src->m_colorPrimaries;               //!< Color primaries
    src->m_sampleRange;                  //!< Sample range
    src->m_chromaFormat;                 //!< 0: 420, 1:422, 2:444
    src->m_chromaLocation[FP_TOTAL];     //!< Chroma location type for subsampled ChromaFormats. 

    src->m_transferFunction;             //!< output transfer function
    src->m_pixelFormat;
    src->m_systemGamma;
    src->m_bitDepthComp[4];
    src->m_pixelType[4];                 //!< pixel type (for OpenEXR)
    src->m_frameRate;
    src->m_picUnitSizeOnDisk;            //!< picture sample unit size on storage medium
    src->m_picUnitSizeShift3;            //!< m_picUnitSizeOnDisk >> 3
#endif
}

const VSFrameRef *VS_CC get_frame(int n, int activationReason,
                                  void **instanceData, void **frameData,
                                  VSFrameContext *frameCtx, VSCore *core,
                                  const VSAPI *vsapi)
{
    TRACE_LINE

    Plugin *plugin = (Plugin *)*instanceData;

    // Request the source frame on the first call
    if (activationReason == arInitial) {
        // std::cerr << "activationReason: arInitial\n";

        // n: The frame number. Negative values will cause an error.
        vsapi->requestFrameFilter(n, plugin->node, frameCtx);
        return NULL;
    }

    if (activationReason != arAllFramesReady) {
        std::cerr << "activationReason error: " << activationReason << "\n";
        return NULL;
    }

    const VSFrameRef *src = vsapi->getFrameFilter(n, plugin->node, frameCtx);
    // The reason we query this on a per frame basis is because we want our
    // filter to accept clips with varying dimensions.
    // If we reject such content using plugin->vi would be better.
    const VSFormat *format = plugin->vi->format;
    int height = vsapi->getFrameHeight(src, 0);
    int width = vsapi->getFrameWidth(src, 0);

#if 0
    printf("fmt->name: %s\n", format->name);
    printf("fmt->id: %d\n", format->id);
    printf("fmt->colorFamily: %d\n", format->colorFamily);
    printf("fmt->sampleType: %d\n", format->sampleType);
    printf("fmt->bitsPerSample: %d\n", format->bitsPerSample);
    printf("fmt->bytesPerSample: %d\n", format->bytesPerSample);
    printf("fmt->subSamplingW: %d\n", format->subSamplingW);
    printf("fmt->subSamplingH: %d\n", format->subSamplingH);
    printf("fmt->numPlanes: %d\n", format->numPlanes);
#endif
#if 0 // TODO
    if (0) {
        ProjectParameters *params = plugin->params;
        dst->colorFamily = params->m_output->m_colorSpace; /* TODO: HDR enum to VS enum */
        dst->sampleType = ; /* one of int and float */
        dst->bitsPerSample = params->m_output->m_bitDepthComp[Y_COMP];
        dst->bytesPerSample = ;
        dst->subSamplingW = ;
        dst->subSamplingH = ;
        dst->numPlanes = 3; /* TODO: always 3 ? */
    }
#endif
    init_input_frame(plugin, n, frameCtx, core, vsapi);

    HDRConvertYUV *converter = (HDRConvertYUV *)plugin->converter;
    converter->processOneFrame(plugin->params);

    // When creating a new frame for output it is VERY EXTREMELY SUPER
    // IMPORTANT to supply the "dominant" source frame to copy properties from.
    // Frame props are an essential part of the filter chain and you should
    // NEVER break it.
    VSFrameRef *dst = vsapi->newVideoFrame(format, width, height, src, core);
    copy_output_frame(plugin, dst, frameCtx, core, vsapi);

    // Release the source frame
    vsapi->freeFrame(src);

    // A reference is consumed when it is returned, so saving the dst
    // reference somewhere and reusing it is not allowed.
    TRACE_LINE
    return dst;
}

void VS_CC free_filter(void *instanceData, VSCore *core, const VSAPI *vsapi)
{
    Plugin *plugin = (Plugin *)instanceData;

    TRACE_LINE

    if (plugin) {
        vsapi->freeNode(plugin->node);

        if (plugin->converter) {
            plugin->converter->destroy();
            delete plugin->converter;
        }

        delete plugin;
    }

    return;
}

void VS_CC create(const VSMap *in, VSMap *out, void *userData, VSCore *core,
                  const VSAPI *vsapi)
{
    TRACE_LINE

    int err;

    assert(in != NULL);
    assert(out != NULL);
    assert(core != NULL);
    assert(vsapi != NULL);

    Plugin *plugin = new Plugin();
    if (!plugin) {
        std::cerr << "create HDRConvert plugin failed\n";
        return;
    }

    const int nbr_elt = vsapi->propNumElements(in, "cfgfile");
    assert(nbr_elt > 0);
    const char *cfgfile = vsapi->propGetData(in, "cfgfile", 0, &err);

    plugin->node = vsapi->propGetNode(in, "clip", 0, 0);
    plugin->vi = vsapi->getVideoInfo(plugin->node);
    plugin->cfgfile = cfgfile;

    vsapi->createFilter(in, out,
                        "YUVConverter", // plugin->filter_name.c_str(),
                        &init_filter, &get_frame, &free_filter,
                        0, // filter mode
                        0, // filter flags
                        plugin, core);

    TRACE_LINE
}

VS_EXTERNAL_API(void)
VapourSynthPluginInit(VSConfigPlugin config_fnc,
                      VSRegisterFunction register_fnc, VSPlugin *plugin)
{
    config_fnc("hdrconv", "hdrc", "HDR Converter 0.0.1",
               VAPOURSYNTH_API_VERSION, 1, plugin);

    register_fnc("YUVConverter", "clip:clip;"
                                 "w:int:opt;"
                                 "h:int:opt;"
                                 "cfgfile:data[]",
                 &create, 0, plugin);
}

/*************************************************************************/
/*                  internal funtions                                    */
/*************************************************************************/
#ifdef PARAM_DEBUG
extern IntegerParameter intParameterList[];
extern DoubleParameter doubleParameterList[];
extern FloatParameter floatParameterList[];
extern StringParameter stringParameterList[];
extern BoolParameter boolParameterList[];

static void print_params(ProjectParameters *params)
{
    ProjectParameters *p = params;
    if (!p) {
        printf("print_params() failed: ProjectParameters is NULL\n");
        return;
    }

    printf("=========================================================\n");
    printf("==============     ProjectParameters        =============\n");
    printf("=========================================================\n");

    // input/output
    printf("m_inputFile: %s\n", p->m_inputFile.m_fName);
    printf("m_inputFile.m_videoType: %d\n", p->m_inputFile.m_videoType);
    printf("m_outputFile: %s\n", p->m_outputFile.m_fName);
    printf("m_outputFile.m_videoType: %d\n", p->m_outputFile.m_videoType);

    // input/output format
    int i;
    for (i = 0; intParameterList[i].ptr != NULL; i++)
        printf("%s = %d\n", intParameterList[i].name,
               *(intParameterList[i].ptr));
    for (i = 0; boolParameterList[i].ptr != NULL; i++)
        printf("%s = %d\n", boolParameterList[i].name,
               *(boolParameterList[i].ptr));
    for (i = 0; floatParameterList[i].ptr != NULL; i++)
        printf("%s = %f\n", floatParameterList[i].name,
               *(floatParameterList[i].ptr));
    for (i = 0; doubleParameterList[i].ptr != NULL; i++)
        printf("%s = %f\n", doubleParameterList[i].name,
               *(doubleParameterList[i].ptr));
    for (i = 0; stringParameterList[i].ptr != NULL; i++)
        printf("%s = %d\n", stringParameterList[i].name,
               *(stringParameterList[i].ptr));

    printf("---------------------------------------------------------\n");
    printf("m_srcNormalScale       : %f\n", p->m_srcNormalScale);
    printf("m_outNormalScale       : %f\n", p->m_outNormalScale);
    printf("m_srcMinValue          : %f\n", p->m_srcMinValue);
    printf("m_srcMaxValue          : %f\n", p->m_srcMaxValue);
    printf("m_outMinValue          : %f\n", p->m_outMinValue);
    printf("m_outMaxValue          : %f\n", p->m_outMaxValue);
    printf("m_transformPrecision   : %s\n",
           p->m_transformPrecision ? "true" : "false");
    printf("m_useSingleTransferStep: %s\n",
           p->m_useSingleTransferStep ? "true" : "false");
    printf("m_filterInFloat        : %s\n",
           p->m_filterInFloat ? "true" : "false");
    printf("m_enableTFunctionLUT   : %s\n",
           p->m_enableTFunctionLUT ? "true" : "false");
    printf("m_enableTFLUTs         : %s\n",
           p->m_enableTFLUTs ? "true" : "false");

    printf("m_chromaDownsampleFilter: %d\n", p->m_chromaDownsampleFilter);
    printf("m_chromaUpsampleFilter : %d\n", p->m_chromaUpsampleFilter);
    printf("m_outputSinglePrecision: %s\n",
           p->m_outputSinglePrecision ? "true" : "false");
    printf("m_useMinMax            : %d\n", p->m_useMinMax);
    printf("m_useHighPrecisionTransform: %d\n", p->m_useHighPrecisionTransform);
    printf("m_addNoise             : %d\n", p->m_addNoise);

    printf("m_noiseVariance        : %f\n", p->m_noiseVariance);
    printf("m_noiseMean: %f\n", p->m_noiseMean);
    // ClosedLoopTrans

    printf("m_closedLoopIterations : %d\n", p->m_closedLoopIterations);
    printf("m_linearDownConversion : %s\n",
           p->m_linearDownConversion ? "true" : "false");

    printf("m_useAdaptiveUpsampling: %d\n", p->m_useAdaptiveUpsampling);
    printf("m_useAdaptiveDownsampling: %d\n", p->m_useAdaptiveDownsampling);

    printf("m_rgbDownConversion    : %s\n",
           p->m_rgbDownConversion ? "true" : "false");
    printf("m_bUseChromaDeblocking : %s\n",
           p->m_bUseChromaDeblocking ? "true" : "false");
    printf("m_bUseWienerFiltering  : %s\n",
           p->m_bUseWienerFiltering ? "true" : "false");
    printf("m_bUseNLMeansFiltering : %s\n",
           p->m_bUseNLMeansFiltering ? "true" : "false");
    printf("m_bUse2DSepFiltering   : %s\n",
           p->m_bUse2DSepFiltering ? "true" : "false");
    printf("m_b2DSepMode: %s\n", p->m_b2DSepMode ? "true" : "false");

    printf("m_cropOffsetLeft       : %d\n", p->m_cropOffsetLeft);
    printf("m_cropOffsetRight      : %d\n", p->m_cropOffsetRight);
    printf("m_cropOffsetTop        : %d\n", p->m_cropOffsetTop);
    printf("m_cropOffsetBottom     : %d\n", p->m_cropOffsetBottom);

    printf("m_toneMapping          : %d\n", p->m_toneMapping);
}
#endif
