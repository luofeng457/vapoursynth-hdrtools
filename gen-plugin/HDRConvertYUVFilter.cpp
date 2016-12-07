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

struct Plugin
{
    VSNodeRef *node;
    const VSVideoInfo *vi;
    int enabled;

    int width;
    int height;
    const char *cfgfile; /* cfg file */

    ProjectParameters params;
};

void VS_CC init_filter(VSMap *in, VSMap *out, void **instanceData, VSNode *node,
                       VSCore *core, const VSAPI *vsapi)
{
    Plugin *plugin = (Plugin *)*instanceData;

    std::cout << "this is " << __FILE__ << ": " << __func__ << std::endl;
    std::cout << "width: " << plugin->width << ", height: " << plugin->height
              << std::endl;

    return;
}

const VSFrameRef *VS_CC get_frame(int n, int activationReason,
                                  void **instanceData, void **frameData,
                                  VSFrameContext *frameCtx, VSCore *core,
                                  const VSAPI *vsapi)
{
    Plugin *plugin = (Plugin *)*instanceData;

    std::cout << "this is " << __FILE__ << ": " << __func__ << std::endl;
    std::cout << "width: " << plugin->width << ", height: " << plugin->height
              << std::endl;

    return NULL;
}

void VS_CC free_filter(void *instanceData, VSCore *core, const VSAPI *vsapi)
{
    Plugin *plugin = (Plugin *)instanceData;

    std::cout << "this is " << __FILE__ << ": " << __func__ << std::endl;

    if (plugin) {
        // TODO: delete node & others first

        delete plugin;
    }

    return;
}

void VS_CC create(const VSMap *in, VSMap *out, void *userData, VSCore *core,
                  const VSAPI *vsapi)
{
    std::cout << "this is " << __FILE__ << ": " << __func__ << std::endl;
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

    plugin->params.refresh();
    plugin->params.m_silentMode = FALSE;
    plugin->params.readConfigFile((char *)cfgfile);
    plugin->params.update();

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

VS_EXTERNAL_API(void)
VapourSynthPluginInit(VSConfigPlugin config_fnc,
                      VSRegisterFunction register_fnc, VSPlugin *plugin)
{
    config_fnc("hdrconv", "hdrc", "HDR Converter, "
                                  "0.0.1",
               VAPOURSYNTH_API_VERSION, 1, plugin);

    register_fnc("YUVConverter", "clip:clip;"
                                 "w:int:opt;"
                                 "h:int:opt;"
                                 "cfgfile:data[]",
                 &create, 0, plugin);
}
