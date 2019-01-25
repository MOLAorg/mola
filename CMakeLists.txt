# ------------------------------------------------------------------------------
#        A Modular Optimization framework for Localization and mApping
#                               (MOLA)
#
# Copyright (C) 2018-2019, Jose Luis Blanco-Claraco, contributors (AUTHORS.md)
# All rights reserved.
# Released under GNU GPL v3. See LICENSE file
# ------------------------------------------------------------------------------

# Tell CMake we'll use C++ for use in its tests/flags
project(mola-sensor-kitti-dataset LANGUAGES CXX)

# -----------------------
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
find_package(mrpt-math) # for MRPT Eigen utilities
find_package(mrpt-maps)
find_package(mrpt-poses)
find_package(mrpt-tclap)

# Public lib Dependencies:
target_link_libraries(${PROJECT_NAME}
  PUBLIC
    mola-kernel
)

# Private lib Dependencies:
target_link_libraries(${PROJECT_NAME}
  PRIVATE
    mrpt::maps
    mrpt::math
)


# ----------------------
# define app target:
add_executable(mola-kitti-eval-error
  apps/mola-kitti-eval-error.cpp)

# Define common flags:
mola_set_target_build_options(mola-kitti-eval-error)
mola_configure_app(mola-kitti-eval-error)
if (CMAKE_COMPILER_IS_GNUCXX)
    # Silent tons of warnings from kitti-eval original code:
    target_compile_options(mola-kitti-eval-error PRIVATE "-Wno-unused-result" "-Wno-shadow" "-Wno-sign-compare")
endif()

# Dependencies:
target_link_libraries(mola-kitti-eval-error
    mrpt::tclap
    mrpt::poses
)
