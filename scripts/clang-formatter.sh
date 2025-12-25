#!/bin/bash

find kitti*/ mola*/ -iname *.h -o -iname *.hpp -o -iname *.cpp -o -iname *.c \
    | grep -v 3rdparty \
    | xargs -I FIL bash -c \
    "echo FIL && clang-format-14 -i FIL"
