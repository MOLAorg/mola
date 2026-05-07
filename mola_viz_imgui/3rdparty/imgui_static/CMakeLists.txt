# ------------------------------------------------------------------------------
# Static library target wrapping Dear ImGui (docking branch).
#
# Expected layout:
#   <this file's parent>/../../3rdparty/imgui/   ← git submodule
#
# The submodule must be the "docking" branch of
# https://github.com/ocornut/imgui
# ------------------------------------------------------------------------------

cmake_minimum_required(VERSION 3.5)
project(imgui_static LANGUAGES CXX)

# Locate the submodule root relative to this file
get_filename_component(_IMGUI_ROOT
    "${CMAKE_CURRENT_SOURCE_DIR}/../../3rdparty/imgui" ABSOLUTE)

if(NOT EXISTS "${_IMGUI_ROOT}/imgui.h")
  message(FATAL_ERROR
    "Dear ImGui submodule not found at ${_IMGUI_ROOT}.\n"
    "Run:  git submodule update --init --recursive")
endif()

# Find GLFW (already required by mrpt-gui, so it will be present)
find_package(glfw3 REQUIRED)
find_package(OpenGL REQUIRED)

set(_IMGUI_SRCS
  ${_IMGUI_ROOT}/imgui.cpp
  ${_IMGUI_ROOT}/imgui_draw.cpp
  ${_IMGUI_ROOT}/imgui_tables.cpp
  ${_IMGUI_ROOT}/imgui_widgets.cpp
#  ${_IMGUI_ROOT}/imgui_demo.cpp          # can be excluded in release builds
  ${_IMGUI_ROOT}/misc/cpp/imgui_stdlib.cpp   # std::string InputText support
  ${_IMGUI_ROOT}/backends/imgui_impl_glfw.cpp
  ${_IMGUI_ROOT}/backends/imgui_impl_opengl3.cpp
)

add_library(imgui_static STATIC ${_IMGUI_SRCS})
set_target_properties(imgui_static PROPERTIES POSITION_INDEPENDENT_CODE TRUE)

target_include_directories(imgui_static
  PUBLIC
    ${_IMGUI_ROOT}
    ${_IMGUI_ROOT}/backends
    ${_IMGUI_ROOT}/misc/cpp
)

# Enable the docking branch feature set.
# No explicit loader macro: imgui_impl_opengl3 will use its bundled
# imgui_impl_opengl3_loader.h (available since ImGui ~1.87), so we do not need
# to find or link GLEW/GLAD/gl3w separately.
target_compile_definitions(imgui_static
  PUBLIC
    IMGUI_ENABLE_DOCKING       # activates docking API
)

target_link_libraries(imgui_static
  PUBLIC
    glfw
    OpenGL::GL
)

# Suppress warnings from third-party code
if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  target_compile_options(imgui_static PRIVATE -w)
endif()

# Export as an alias so parent CMake can use it as imgui::imgui
add_library(imgui::imgui ALIAS imgui_static)
