# ------------------------------------------------------------------------------
#        A Modular Optimization framework for Localization and mApping
#                               (MOLA)
#
# Copyright (C) 2018-2024, Jose Luis Blanco-Claraco, contributors (AUTHORS.md)
# All rights reserved.
# Released under GNU GPL v3. See LICENSE file
# ------------------------------------------------------------------------------

# Minimum CMake vesion: limited by CMAKE_WINDOWS_EXPORT_ALL_SYMBOLS
cmake_minimum_required(VERSION 3.4)

# Tell CMake we'll use C++ for use in its tests/flags
project(mola_kernel LANGUAGES CXX)

# MOLA CMake scripts: "mola_xxx()"
find_package(mola_common REQUIRED)
find_package(mola_yaml REQUIRED)

# find dependencies:
find_package(MRPT 2.1.0 REQUIRED COMPONENTS gui obs maps)

# define lib:
file(GLOB_RECURSE LIB_SRCS src/*.cpp src/*.h)
file(GLOB_RECURSE LIB_PUBLIC_HDRS include/*.h)

mola_add_library(
	TARGET ${PROJECT_NAME}
	SOURCES ${LIB_SRCS} ${LIB_PUBLIC_HDRS}
	PUBLIC_LINK_LIBRARIES
		mrpt::obs
		mrpt::gui
		mola::mola_yaml
	PRIVATE_LINK_LIBRARIES
		mrpt::gui   #TODO: remove after mapviz!
		mrpt::maps  #TODO: remove after mapviz!
	CMAKE_DEPENDENCIES
		mrpt-obs
		mrpt-gui
		mola_yaml
)

if ("$ENV{VERBOSE}")
	message(STATUS "mola_kernel: Using MRPT version: ${MRPT_VERSION}")
endif()
