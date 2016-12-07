# vim: set filetype=python
#
# simple vspipe script to test hdrconv plugin
#
# Author: Yanan Zhao <arthurchiao@hotmail.com>
# Date  : 2016-12-02 10:10:45

import vapoursynth as vs

core = vs.get_core()
core.std.LoadPlugin("/usr/lib/x86_64-linux-gnu/libffms2.so")
core.std.LoadPlugin("./libhdrconv.so")

clip = core.ffms2.Source(source='/home/arthurchiao/cherry/video_files/in.mp4')
clip = core.hdrc.YUVConverter(clip=clip, w=1920, h=1088)

# from hdrc import hdrconv
# print(core.hdrc)
# print(core.list_functions())
