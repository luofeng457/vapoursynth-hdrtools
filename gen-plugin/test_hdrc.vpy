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

# clip = core.ffms2.Source(source='/home/arthurchiao/testmedia/in.mp4')
clip = core.ffms2.Source(source='/home/arthurchiao/testmedia/jellyfish-3-mbps-hd-hevc-10bit.mkv')
# clip = core.ffms2.Source(source='/jellyfish-3-mbps-hd-hevc-10bit.mkv')
clip = core.hdrc.YUVConverter(clip=clip, cfgfile="yuv420_10bit_to_yuv.cfg")
clip.set_output()
