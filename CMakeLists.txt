# ------------------------------------------------------------------------------
#        A Modular Optimization framework for Localization and mApping
#                               (MOLA)
#
# Copyright (C) 2018-2019, Jose Luis Blanco-Claraco, contributors (AUTHORS.md)
# All rights reserved.
# Released under GNU GPL v3. See LICENSE file
# ------------------------------------------------------------------------------

# Minimum CMake vesion: limited by CMAKE_WINDOWS_EXPORT_ALL_SYMBOLS
cmake_minimum_required(VERSION 3.4)

# Tell CMake we'll use C++ for use in its tests/flags
project(mola-launcher LANGUAGES CXX)

# MOLA CMake scripts: "mola_xxx()"
find_package(mola-common REQUIRED)

# find dependencies:
find_package(mrpt-core)

find_mola_package(mola-kernel)

# -----------------------
# define lib:
file(GLOB_RECURSE LIB_SRCS src/*.cpp src/*.h)
file(GLOB_RECURSE LIB_PUBLIC_HDRS include/*.h)


mola_add_library(
	TARGET ${PROJECT_NAME}
	SOURCES ${LIB_SRCS} ${LIB_PUBLIC_HDRS}
	PUBLIC_LINK_LIBRARIES
		mrpt::core
		mola-kernel
	PRIVATE_LINK_LIBRARIES
		${CMAKE_DL_LIBS} # To be able to load DLLs/.so's
)

target_compile_definitions(${PROJECT_NAME}
    PRIVATE
    BUILDTIME_MOLA_MODULES_LIB_PATH="${CMAKE_LIBRARY_OUTPUT_DIRECTORY}"
    # Use top-level source directory:
    BUILDTIME_MOLA_MODULES_SHARED_PATH="${CMAKE_SOURCE_DIR}/modules"
    )

# apps:
add_subdirectory(apps)
