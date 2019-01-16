# ------------------------------------------------------------------------------
#        A Modular Optimization framework for Localization and mApping
#                               (MOLA)
#
# Copyright (C) 2018-2019, José Luis Blanco-Claraco, contributors (AUTHORS.md)
# All rights reserved.
# Released under GNU GPL v3. See LICENSE file
# ------------------------------------------------------------------------------

# Tell CMake we'll use C++ for use in its tests/flags
project(mola-launcher LANGUAGES CXX)

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
find_package(mrpt-core)

# Public lib Dependencies:
target_link_libraries(${PROJECT_NAME}
  PUBLIC
    mrpt::core
    mola-kernel
)

# Private lib Dependencies:
if(UNIX)
    # To be able to load DLLs/.so's:
    target_link_libraries(${PROJECT_NAME} PRIVATE ${CMAKE_DL_LIBS})
endif()

target_compile_definitions(${PROJECT_NAME}
    PRIVATE
    -DMOLA_MODULES_DIR="${CMAKE_LIBRARY_OUTPUT_DIRECTORY}"
    )

# ----------------------
# define app target:
file(GLOB_RECURSE SRCS cli/*.cpp cli/*.h)
add_executable(mola-cli ${SRCS})

# Define common flags:
mola_set_target_build_options(mola-cli)
mola_configure_app(mola-cli)

# find dependencies:
find_package(mrpt-core)
find_package(mrpt-tclap) # tclap wrapper, useful for Windows, etc.

# Dependencies:
target_link_libraries(mola-cli
    mrpt::core
    mrpt::tclap
    yaml-cpp
    ${PROJECT_NAME}
)
