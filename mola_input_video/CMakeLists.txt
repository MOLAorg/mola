# ------------------------------------------------------------------------------
#        A Modular Optimization framework for Localization and mApping
#                               (MOLA)
#
# Copyright (C) 2018-2026, Jose Luis Blanco-Claraco, contributors (AUTHORS.md)
# All rights reserved.
# Released under GNU GPL v3. See LICENSE file
# ------------------------------------------------------------------------------

cmake_minimum_required(VERSION 3.5)

# Tell CMake we'll use C++ for use in its tests/flags
project(mola_input_video LANGUAGES CXX)

find_package(mola_common REQUIRED)
find_package(mrpt-hwdrivers REQUIRED)
find_package(mrpt-obs REQUIRED)

find_mola_package(mola_kernel)

set(LIB_SRCS
  src/VideoDataset.cpp
)

set(LIB_PUBLIC_HDRS
  include/mola_input_video/VideoDataset.h
)

mola_add_library(
  TARGET ${PROJECT_NAME}
  SOURCES ${LIB_SRCS} ${LIB_PUBLIC_HDRS}
  PRIVATE_LINK_LIBRARIES
    mola::mola_kernel
    mrpt::hwdrivers
    mrpt::obs
)
