#!/bin/sh

# vspipe test_hdrc.vpy -

# vspipe --y4m test_hdrc.vpy - | ffmpeg -i pipe: out.mkv
# vspipe --y4m test_hdrc.vpy - | x264 --demuxer y4m - --output out.mkv
# vspipe --y4m --start 1 --end 10 test_hdrc.vpy - | ffmpeg -i pipe: out.mkv
vspipe --y4m --start 1 --end 10 test_hdrc.vpy - | ffmpeg -i pipe: -filter:v "setpts=20.0*PTS" out.mkv
