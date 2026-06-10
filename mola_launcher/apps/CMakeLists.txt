# ------------------------------------------------------------------------------
#        A Modular Optimization framework for Localization and mApping
#                               (MOLA)
#
# Copyright (C) 2018-2026, Jose Luis Blanco-Claraco, contributors (AUTHORS.md)
# All rights reserved.
# Released under GNU GPL v3. See LICENSE file
# ------------------------------------------------------------------------------

find_package(mrpt-core)
find_package(CLI11 REQUIRED)
find_package(mrpt-containers)

mola_add_executable(
  TARGET  mola-cli
  SOURCES mola-cli.cpp
  LINK_LIBRARIES
    mola::mola_launcher
    mola::mola_kernel
    mrpt::core
    CLI11::CLI11
)

mola_add_executable(
  TARGET  mola-dir
  SOURCES mola-dir.cpp
  LINK_LIBRARIES
    mola::mola_launcher
    mola::mola_kernel
)

mola_add_executable(
  TARGET  mola-yaml-parser
  SOURCES mola-yaml-parser.cpp
  LINK_LIBRARIES
    mola::mola_kernel
    CLI11::CLI11
    mrpt::containers
)
