# ------------------------------------------------------------------------------
#        A Modular Optimization framework for Localization and mApping
#                               (MOLA)
#
# Copyright (C) 2018-2019, Jose Luis Blanco-Claraco, contributors (AUTHORS.md)
# All rights reserved.
# Released under GNU GPL v3. See LICENSE file
# ------------------------------------------------------------------------------

# Tell CMake we'll use C++ for use in its tests/flags
project(mola-kernel LANGUAGES CXX)

# define lib:
file(GLOB_RECURSE LIB_SRCS src/*.cpp src/*.h)
file(GLOB_RECURSE LIB_PUBLIC_HDRS include/*.h)

add_library(${PROJECT_NAME} SHARED
    ${LIB_SRCS}
    ${LIB_PUBLIC_HDRS}
    )

# Define common flags:
mola_set_target_build_options(${PROJECT_NAME})
mola_configure_library(${PROJECT_NAME})

# find dependencies:
find_package(MRPT 1.9.9 REQUIRED COMPONENTS gui obs maps)
find_package(TBB COMPONENTS tbb tbbmalloc)

if (TARGET tbb)
    message("mola-kernel: Using TBB")
    target_link_libraries(${PROJECT_NAME} PRIVATE tbbmalloc tbb)
    target_compile_definitions(${PROJECT_NAME} PUBLIC MOLA_KERNEL_HAS_TBB)
endif()

message("mola-kernel: Using MRPT version: ${MRPT_VERSION}")


# Public lib Dependencies:
target_link_libraries(${PROJECT_NAME}
  PUBLIC
    mrpt::obs
)

# Private lib Dependencies:
target_link_libraries(${PROJECT_NAME}
  PRIVATE
    gtsam
    mrpt::gui   #TODO: remove after mapviz!
    mrpt::maps  #TODO: remove after mapviz!
)
