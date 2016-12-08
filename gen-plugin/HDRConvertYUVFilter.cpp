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
    std::cout << "@" << __FILE__ << ": " << __LINE__ << " " << __func__        \
              << std::endl;

static void print_params(ProjectParameters *params);

struct Plugin
{
    VSNodeRef *node;
    const VSVideoInfo *vi;
    int enabled;

    int width;
    int height;
    const char *cfgfile; /* cfg file */

    ProjectParameters *params;
};

void VS_CC init_filter(VSMap *in, VSMap *out, void **instanceData, VSNode *node,
                       VSCore *core, const VSAPI *vsapi)
{
    Plugin *plugin = (Plugin *)*instanceData;

    TRACE_LINE

    plugin->params = &ccParams;
    plugin->params->refresh();
    plugin->params->m_silentMode = FALSE;
    plugin->params->readConfigFile((char *)plugin->cfgfile);
    plugin->params->update();
    print_params(plugin->params);

    HDRConvert *hdrProcess = HDRConvert::create(plugin->params);

    int test = 0;
    std::cout << "---------test" << test++ << std::endl;
    hdrProcess->init(plugin->params);
    std::cout << "---------test" << test++ << std::endl;
    hdrProcess->outputHeader(plugin->params);
    std::cout << "---------test" << test++ << std::endl;
    hdrProcess->process(plugin->params);
    std::cout << "---------test" << test++ << std::endl;
    hdrProcess->outputFooter(plugin->params);
    std::cout << "---------test" << test++ << std::endl;
    hdrProcess->destroy();

    delete hdrProcess;

    return;
}

const VSFrameRef *VS_CC get_frame(int n, int activationReason,
                                  void **instanceData, void **frameData,
                                  VSFrameContext *frameCtx, VSCore *core,
                                  const VSAPI *vsapi)
{
    Plugin *plugin = (Plugin *)*instanceData;

    TRACE_LINE

    std::cout << "width: " << plugin->width << ", height: " << plugin->height
              << std::endl;

    return NULL;
}

void VS_CC free_filter(void *instanceData, VSCore *core, const VSAPI *vsapi)
{
    Plugin *plugin = (Plugin *)instanceData;

    TRACE_LINE

    if (plugin) {
        // TODO: delete node & others first

        delete plugin;
    }

    return;
}

void VS_CC create(const VSMap *in, VSMap *out, void *userData, VSCore *core,
                  const VSAPI *vsapi)
{
    TRACE_LINE

    std::cout << "creating HDRConvertYUV Filter ..." << std::endl;

    assert(in != NULL);
    assert(out != NULL);
    assert(core != NULL);
    assert(vsapi != NULL);

    Plugin *plugin = new Plugin();

    int err;
    plugin->width = vsapi->propGetInt(in, "w", 0, &err);
    plugin->height = vsapi->propGetInt(in, "h", 0, &err);
    std::cout << "width: " << plugin->width << ", height: " << plugin->height
              << std::endl;

    const int nbr_elt = vsapi->propNumElements(in, "cfgfile");
    assert(nbr_elt > 0);
    const char *cfgfile = vsapi->propGetData(in, "cfgfile", 0, &err);
    std::cout << "cfgfile: " << cfgfile << std::endl;

    plugin->node = vsapi->propGetNode(in, "clip", 0, 0);
    plugin->vi = vsapi->getVideoInfo(plugin->node);
    plugin->cfgfile = cfgfile;

    if (plugin) {
        vsapi->createFilter(
            in, out,
            "HDRConvertYUVFilter", // plugin->filter_name.c_str(),
            &init_filter, &get_frame, &free_filter,
            0, // filter mode
            0, // filter flags
            plugin, core);
    }

    std::cout << "Filter created successful" << std::endl;
}

VS_EXTERNAL_API(void)
VapourSynthPluginInit(VSConfigPlugin config_fnc,
                      VSRegisterFunction register_fnc, VSPlugin *plugin)
{
    config_fnc("hdrconv", "hdrc", "HDR Converter, ""0.0.1",
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
