# ------------------------------------------------------------------------------
#        A Modular Optimization framework for Localization and mApping
#                               (MOLA)
#
# Copyright (C) 2018-2019, José Luis Blanco-Claraco, contributors (AUTHORS.md)
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
mola_define_library(${PROJECT_NAME})

# Dependencies:
find_package(mrpt-graphs)
find_package(mrpt-maps)
find_package(mrpt-obs)

target_link_libraries(${PROJECT_NAME}
  PUBLIC
    mrpt-graphs
    mrpt-maps
    mrpt-obs
)

# make install:
install(TARGETS ${PROJECT_NAME} EXPORT ${PROJECT_NAME}-config
  ARCHIVE  DESTINATION ${CMAKE_INSTALL_LIBDIR}
  LIBRARY  DESTINATION ${CMAKE_INSTALL_LIBDIR}
  RUNTIME  DESTINATION ${CMAKE_INSTALL_BINDIR}
)
install(
  DIRECTORY include/
  DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}
)
# make project importable:
install(
  EXPORT ${PROJECT_NAME}-config
  DESTINATION share/${PROJECT_NAME}/cmake
)
# make project importable from build_dir:
export(
  TARGETS ${PROJECT_NAME}
  FILE ${PROJECT_NAME}-config.cmake
)
