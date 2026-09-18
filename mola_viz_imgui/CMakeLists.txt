# ------------------------------------------------------------------------------
#        A Modular Optimization framework for Localization and mApping
#                               (MOLA)
#
# Copyright (C) 2018-2026, Jose Luis Blanco-Claraco, contributors (AUTHORS.md)
# All rights reserved.
# Released under GNU GPL v3. See LICENSE file
# ------------------------------------------------------------------------------

cmake_minimum_required(VERSION 3.5)
project(mola_viz_imgui LANGUAGES CXX)

# MOLA CMake scripts: "mola_xxx()"
find_package(mola_common REQUIRED)

# -- External dependencies ----------------------------------------------------
find_package(mrpt_maps    REQUIRED)
find_package(mrpt_opengl  REQUIRED)
find_package(mrpt_obs     REQUIRED)
find_package(mrpt_gui     REQUIRED)
find_package(mrpt_viz     REQUIRED)
find_package(mrpt_imgui   REQUIRED)
find_mola_package(mola_kernel)

# GLFW is needed explicitly for our own GLFW window management.
find_package(glfw3 REQUIRED)

set(OpenGL_GL_PREFERENCE GLVND)
find_package(OpenGL REQUIRED)

# -- Dear ImGui static library (from 3rdparty submodule) ---------------------
#
# The submodule lives at:  <repo_root>/3rdparty/imgui   (docking branch)
# The helper CMakeLists that builds it as a static lib lives at:
#   3rdparty/imgui_static/CMakeLists.txt
#
add_subdirectory(3rdparty/imgui_static imgui_static_build)
# After this, the target  imgui::imgui  is available.

# -- Module sources -----------------------------------------------------------
set(LIB_SRCS
  src/ConsoleLogSink.cpp
  src/MetricsRegistry.cpp
  src/MolaVizImGuiCore.cpp
  src/MolaVizImGui_handlers.cpp
  src/MolaVizImGui.cpp
  src/MolaVizImGui_widgets.cpp
)
set(LIB_PUBLIC_HDRS
  include/mola_viz_imgui/MolaVizImGuiCore.h
  include/mola_viz_imgui/MolaVizImGui.h
)

mola_add_library(
  TARGET ${PROJECT_NAME}
  SOURCES ${LIB_SRCS} ${LIB_PUBLIC_HDRS}
  PUBLIC_LINK_LIBRARIES
    mola::mola_kernel
    mrpt::mrpt_opengl
    mrpt::mrpt_maps
    mrpt::mrpt_obs
    mrpt::mrpt_viz
    OpenGL::GL
    mrpt::mrpt_gui
    mrpt::mrpt_imgui   # public header includes mrpt/imgui/CImGuiSceneView.h
  PRIVATE_LINK_LIBRARIES
    imgui::imgui
  CMAKE_DEPENDENCIES
    mola_kernel
    mrpt_opengl
    mrpt_maps
    mrpt_obs
    mrpt_gui
    mrpt_imgui
    mrpt_viz
)

# -- Compiler options ---------------------------------------------------------
# C++17 required for std::visit, if_constexpr, structured bindings. Already req by MRPT anyway.
target_compile_features(${PROJECT_NAME} PUBLIC cxx_std_17)
