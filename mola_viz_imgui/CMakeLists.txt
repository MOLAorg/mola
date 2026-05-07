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
find_package(mrpt-maps    REQUIRED)
find_package(mrpt-opengl  REQUIRED)
find_package(mrpt-obs     REQUIRED)
find_package(mrpt-gui     REQUIRED)
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
  src/MolaVizImGui_handlers.cpp
  src/MolaVizImGui.cpp
  src/MolaVizImGui_widgets.cpp
)
set(LIB_PUBLIC_HDRS
  include/mola_viz_imgui/MolaVizImGui.h
)

mola_add_library(
  TARGET ${PROJECT_NAME}
  SOURCES ${LIB_SRCS} ${LIB_PUBLIC_HDRS}
  PUBLIC_LINK_LIBRARIES
    mola::mola_kernel
    mrpt::opengl
    mrpt::maps
    mrpt::obs
    OpenGL::GL
    mrpt::gui          # public header includes mrpt/imgui/CImGuiSceneView.h
  PRIVATE_LINK_LIBRARIES
    imgui::imgui
  CMAKE_DEPENDENCIES
    mola_kernel
    mrpt-opengl
    mrpt-maps
    mrpt-obs
    mrpt-gui
)

# -- Compiler options ---------------------------------------------------------
# C++17 required for std::visit, if_constexpr, structured bindings. Already req by MRPT anyway.
target_compile_features(${PROJECT_NAME} PUBLIC cxx_std_17)
