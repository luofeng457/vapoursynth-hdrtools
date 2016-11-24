#!/bin/sh

ctags -R --languages=C,C++ \
    --exclude=projects/ChromaConvert \
    --exclude=projects/HDRConvScaler \
    --exclude=projects/HDRMetrics \
    --exclude=projects/HDRMontage \
    --exclude=projects/HDRVQM \
    *
