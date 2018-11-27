
# yaml-cpp:
# See: https://github.com/jbeder/yaml-cpp
# MIT License

option(MOLA_BUILD_YAML_CPP OFF "Build yaml-cpp from sources?")

if (NOT MOLA_BUILD_YAML_CPP)
  find_package(yaml-cpp QUIET)
  if (NOT yaml-cpp_FOUND)
    message(ERROR
" yaml-cpp was not found: \n\
* either install it (or set yaml-cpp_DIR correctly if it's already installed),\n\
  In Ubuntu: install with `sudo apt install libyaml-cpp-dev`, or \n\
* set MOLA_BUILD_YAML_CPP=ON to build it from sources\n\
")
  endif()
else()
  mark_as_advanced(yaml-cpp_DIR)
  project(EP_yamlcpp)
  set(YAML_CPP_SOURCE_DIR ${EP_yamlcpp_SOURCE_DIR}/yaml-cpp)

  # Define a custom build of yaml-cpp. It's the easiest way for users.
  # I'm not satisfied with neither ExternalProject_Add() (it's so hard to find_package then
  # w/o having to run cmake twice...), nor Hunter, etc.
  # What follows is all based on yaml-cpp/CMakeLists.txt:

  file(GLOB sources "yaml-cpp/src/[a-zA-Z]*.cpp")
  file(GLOB_RECURSE public_headers "yaml-cpp/include/yaml-cpp/[a-zA-Z]*.h")
  file(GLOB private_headers "yaml-cpp/src/[a-zA-Z]*.h")

  set(library_sources
    ${sources}
    ${public_headers}
    ${private_headers}
  )
  add_library(yaml-cpp STATIC ${library_sources})
  set_target_properties(yaml-cpp PROPERTIES FOLDER "ExternalProjectTargets")

  target_include_directories(yaml-cpp
      PUBLIC $<BUILD_INTERFACE:${YAML_CPP_SOURCE_DIR}/include>
#TODO?:         $<INSTALL_INTERFACE:${INCLUDE_INSTALL_ROOT_DIR}>
      PRIVATE $<BUILD_INTERFACE:${YAML_CPP_SOURCE_DIR}/src>)

  set(yaml_c_flags ${CMAKE_C_FLAGS})
  set(yaml_cxx_flags ${CMAKE_CXX_FLAGS})

  # GCC or Clang or Intel Compiler specialities
  if(CMAKE_CXX_COMPILER_ID MATCHES "GNU" OR
    CMAKE_CXX_COMPILER_ID MATCHES "Clang" OR
    CMAKE_CXX_COMPILER_ID MATCHES "Intel")
      set(yaml_cxx_flags "-Wall ${GCC_EXTRA_OPTIONS} -pedantic -Wno-long-long ${yaml_cxx_flags}")
  endif()
  if(MSVC)
   set(yaml_cxx_flags "/W3 /wd4127 /wd4355 ${yaml_cxx_flags}")
  endif()

  set(LIB_INSTALL_DIR "lib${LIB_SUFFIX}")
  if(WIN32 AND NOT CYGWIN)
  	set(INSTALL_CMAKE_DIR CMake)
  else()
  	set(INSTALL_CMAKE_DIR ${LIB_INSTALL_DIR}/cmake/yaml-cpp)
  endif()

  set_target_properties(yaml-cpp PROPERTIES
   COMPILE_FLAGS "${yaml_c_flags} ${yaml_cxx_flags}"
  )
  export(
     TARGETS yaml-cpp
     FILE "${PROJECT_BINARY_DIR}/yaml-cpp-targets.cmake")
  export(PACKAGE yaml-cpp)
  set(EXPORT_TARGETS yaml-cpp CACHE INTERNAL "export targets")
  install(TARGETS yaml-cpp EXPORT yaml-cpp-targets
    RUNTIME DESTINATION bin
    LIBRARY DESTINATION ${LIB_INSTALL_DIR}
    ARCHIVE DESTINATION ${LIB_INSTALL_DIR}
  )
  install(EXPORT yaml-cpp-targets DESTINATION ${INSTALL_CMAKE_DIR})

endif()
