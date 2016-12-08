#!/bin/sh

# vspipe test_hdrc.vpy -

vspipe --y4m test_hdrc.vpy - | ffmpeg -i pipe: out.mkv
