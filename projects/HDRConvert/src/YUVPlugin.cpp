#include <stdlib.h>
#include "VapourSynth.h"
#include "VSHelper.h"

typedef struct
{
    VSNodeRef *node;
    const VSVideoInfo *vi;
    int enabled;
} HDRConvertYUVData;

// This function is called immediately after vsapi->createFilter().
// This is the only place where the video properties may be set.
// In this case we simply use the same as the input clip.
// You may pass an array
// of VSVideoInfo if the filter has more than one output, like rgb+alpha as two
// separate clips.
static void VS_CC hdrConvertYUVInit(VSMap *in, VSMap *out, void **instanceData,
                                    VSNode *node, VSCore *core,
                                    const VSAPI *vsapi)
{
    HDRConvertYUVData *d = (HDRConvertYUVData *)*instanceData;
    int numOfOutputs = 1;
    vsapi->setVideoInfo(d->vi, numOfOutputs, node);
}

// Free all allocated data on filter destruction
static void VS_CC
    invertFree(void *instanceData, VSCore *core, const VSAPI *vsapi)
{
    HDRConvertYUVData *d = (HDRConvertYUVData *)instanceData;
    vsapi->freeNode(d->node);
    free(d);
}

static const VSFrameRef *VS_CC
    hdrConvertYUVGetFrame(int n, int activationReason, void **instanceData,
                          void **frameData, VSFrameContext *frameCtx,
                          VSCore *core, const VSAPI *vsapi)
{
    HDRConvertYUVData *d = (HDRConvertYUVData *)*instanceData;

    if (activationReason == arInitial) {
        // Request the source frame on the first call
        // n: The frame number. Negative values will cause an error.
        vsapi->requestFrameFilter(n, d->node, frameCtx);

        return 0;
    }

    if (activationReason == arAllFramesReady) {
        const VSFrameRef *src = vsapi->getFrameFilter(n, d->node, frameCtx);
        // The reason we query this on a per frame basis is because we want our
        // filter
        // to accept clips with varying dimensions. If we reject such content
        // using d->vi would be better.
        const VSFormat *format = d->vi->format;
        int height = vsapi->getFrameHeight(src, 0);
        int width = vsapi->getFrameWidth(src, 0);

        // When creating a new frame for output it is VERY EXTREMELY SUPER
        // IMPORTANT to
        // supply the "dominant" source frame to copy properties from. Frame
        // props
        // are an essential part of the filter chain and you should NEVER break
        // it.
        VSFrameRef *dst =
            vsapi->newVideoFrame(format, width, height, src, core);

        // It's processing loop time!
        // Loop over all the planes
        int plane;
        for (plane = 0; plane < format->numPlanes; plane++) {
            const uint8_t *srcp = vsapi->getReadPtr(src, plane);
            int src_stride = vsapi->getStride(src, plane);
            uint8_t *dstp = vsapi->getWritePtr(dst, plane);
            int dst_stride = vsapi->getStride(
                dst, plane); // note that if a frame has the same dimensions and
                             // format, the stride is guaranteed to be the same.
                             // int dst_stride = src_stride would be fine too in
                             // this filter.
            // Since planes may be subsampled you have to query the height of
            // them individually
            int h = vsapi->getFrameHeight(src, plane);
            int y;
            int w = vsapi->getFrameWidth(src, plane);
            int x;

            for (y = 0; y < h; y++) {
                for (x = 0; x < w; x++)
                    dstp[x] = ~srcp[x];

                dstp += dst_stride;
                srcp += src_stride;
            }
        }

        // Release the source frame
        vsapi->freeFrame(src);

        // A reference is consumed when it is returned, so saving the dst
        // reference somewhere
        // and reusing it is not allowed.
        return dst;
    }

    return 0;
}

// This function is responsible for validating arguments and creating a new
// filter
static void VS_CC hdrConvertYUVCreate(const VSMap *in, VSMap *out,
                                      void *userData, VSCore *core,
                                      const VSAPI *vsapi)
{
    HDRConvertYUVData *data = new HDRConvertYUVData();
    int err;

    // Get a clip reference from the input arguments. This must be freed later.
    data->node = vsapi->propGetNode(in, "clip", 0, 0);
    data->vi = vsapi->getVideoInfo(data->node);

    // Creates a new filter and returns a reference to it. Always pass on the in
    // and out
    // arguments or unexpected things may happen. The name should be something
    // that's
    // easy to connect to the filter, like its function name.
    // The three function pointers handle initialization, frame processing and
    // filter destruction.
    // The filtermode is very important to get right as it controls how
    // threading of the filter
    // is handled. In general you should only use fmParallel whenever possible.
    // This is if you
    // need to modify no shared data at all when the filter is running.
    // For more complicated filters, fmParallelRequests is usually easier to
    // achieve as it can
    // be prefetched in parallel but the actual processing is serialized.
    // The others can be considered special cases where fmSerial is useful to
    // source filters and
    // fmUnordered is useful when a filter's state may change even when deciding
    // which frames to
    // prefetch (such as a cache filter).
    // If your filter is really fast (such as a filter that only resorts frames)
    // you should set the
    // nfNoCache flag to make the caching work smoother.
    vsapi->createFilter(in, out, "Invert", hdrConvertYUVInit, hdrConvertYUVGetFrame,
                        invertFree, fmParallel, 0, data, core);
}

//////////////////////////////////////////
// Init

// This is the entry point that is called when a plugin is loaded.
// registerFunc is called once for each function you want to register. Function
// names
// should be PascalCase. The argument string has this format:
// name:type; or name:type:flag1:flag2....;
// All argument name should be lowercase and only use [a-z_].
// The valid types are int,float,data,clip,frame,func. [] can be appended to
// allow arrays of type to be passed (numbers:int[])
// The available flags are opt, to make an argument optional, empty, which
// controls whether
// or not empty arrays are accepted
VS_EXTERNAL_API(void)
VapourSynthPluginInit(VSConfigPlugin configFunc,
                      VSRegisterFunction registerFunc, VSPlugin *plugin)
{
    configFunc("com.example.invert", // id, reverse url
               "invert",             // namespace, should only use [a-z_]
               "VapourSynth Invert Example", // full name description
               VAPOURSYNTH_API_VERSION, 1, plugin);

    registerFunc("HDRConvertYUV", "clip:clip;"
                                  "enabled:int:opt;",
                 hdrConvertYUVCreate, 0, plugin);

#if 0
    registerFunc("HDRConvertTIFF", "clip:clip;"
                                   "enabled:int:opt;",
                 hdrConvertTIFFCreate, 0, plugin);

    registerFunc("HDRConvertEXR", "clip:clip;"
                                  "enabled:int:opt;",
                 hdrConvertEXRCreate, 0, plugin);
#endif
}
