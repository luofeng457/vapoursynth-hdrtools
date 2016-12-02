# vim: set filetype=python
#
# simple vspipe script to test hdrconv plugin
#
# Author: Yanan Zhao <arthurchiao@hotmail.com>
# Date  : 2016-12-02 10:10:45

import vapoursynth as vs

core = vs.get_core()
core.std.LoadPlugin("/home/cloud/opensource/arthurchiao/vapoursynth-hdrtools/gen-plugin/libhdrconv.so")

clip = core.ffms2.Source(source='/home/cloud/in.mp4')
clip = core.hdrc.YUVConverter(clip=clip, w=1920, h=1088)

# from hdrc import hdrconv
# print(core.hdrc)
# print(core.list_functions())
