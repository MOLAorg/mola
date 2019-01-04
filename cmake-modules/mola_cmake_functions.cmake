

# Avoid the need for DLL export/import macros in Windows:
if (WIN32)
  set(CMAKE_WINDOWS_EXPORT_ALL_SYMBOLS  ON)
endif()

# Detect wordsize:
if(CMAKE_SIZEOF_VOID_P EQUAL 8)  # Size in bytes!
  set(MOLA_WORD_SIZE 64)
else()
  set(MOLA_WORD_SIZE 32)
endif()

# Default output dirs for libs:
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/lib/")
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY  "${CMAKE_BINARY_DIR}/lib/")
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/bin/")

# Compiler ID:
if (MSVC)
  # 1700 = VC 11.0 (2012)
  # 1800 = VC 12.0 (2013)
  #           ... (13 was skipped!)
  # 1900 = VC 14.0 (2015)
  # 1910 = VC 14.1 (2017)
  math(EXPR MSVC_VERSION_3D "(${MSVC_VERSION}/10)-60")
  if (MSVC_VERSION_3D GREATER 120)
    math(EXPR MSVC_VERSION_3D "${MSVC_VERSION_3D}+10")
  endif()
  set(MOLA_COMPILER_NAME "msvc${MSVC_VERSION_3D}")
else()
  set(MOLA_COMPILER_NAME "${CMAKE_CXX_COMPILER_ID}")
endif()

# Build DLL full name:
if (WIN32)
  set(MOLA_DLL_VERSION_POSTFIX
    "${MOLA_VERSION_NUMBER_MAJOR}${MOLA_VERSION_NUMBER_MINOR}${MOLA_VERSION_NUMBER_PATCH}_${MOLA_COMPILER_NAME}_x${MOLA_WORD_SIZE}")
  message(STATUS "Using DLL version postfix: ${MOLA_DLL_VERSION_POSTFIX}")
else()
  set(MOLA_DLL_VERSION_POSTFIX "")
endif()

# Group projects in "folders"
set_property(GLOBAL PROPERTY USE_FOLDERS ON)
set_property(GLOBAL PROPERTY PREDEFINED_TARGETS_FOLDER "CMakeTargets")

# We want libraries to be named "libXXX" in all compilers, "libXXX-dbg" in MSVC
set(CMAKE_SHARED_LIBRARY_PREFIX "lib")
set(CMAKE_IMPORT_LIBRARY_PREFIX "lib")
set(CMAKE_STATIC_LIBRARY_PREFIX "lib")
set(CMAKE_DEBUG_POSTFIX "-dbg")

# -----------------------------------------------------------------------------
# mola_set_target_build_options(target)
#
# Set defaults for each MOLA cmake target
# -----------------------------------------------------------------------------
function(mola_set_target_build_options TARGETNAME)
  # Build for C++17
  # -------------------------
  target_compile_features(${TARGETNAME} INTERFACE cxx_std_17)
  if (MSVC)
    # this seems to be required in addition to the cxx_std_17 above (?)
    target_compile_options(${TARGETNAME} INTERFACE /std:c++latest)
  endif()

  # Warning level:
  # -------------------------
  if (MSVC)
    # msvc:
    target_compile_options(${TARGETNAME} PRIVATE /W3)
    target_compile_definitions(${TARGETNAME} PRIVATE
      _CRT_SECURE_NO_DEPRECATE
      _CRT_NONSTDC_NO_DEPRECATE
      _SILENCE_ALL_CXX17_DEPRECATION_WARNINGS
    )
  else()
    # gcc & clang:
    target_compile_options(${TARGETNAME} PRIVATE
      -Wall -Wextra -Wshadow -Wreturn-type -Wabi=11
      -Wtype-limits -Wcast-align -Wparentheses
      -fPIC
    )
  endif()

  # Optimization:
  # -------------------------
  if((NOT MSVC) AND ((NOT CMAKE_CROSSCOMPILING) AND (NOT CMAKE_BUILD_TYPE STREQUAL "Debug")))
    target_compile_options(${TARGETNAME} PRIVATE -O3 -mtune=native)
  endif()

endfunction()

# -----------------------------------------------------------------------------
# mola_configure_library(target)
#
# Define a consistent install behavior for cmake-based library project:
# -----------------------------------------------------------------------------
function(mola_configure_library TARGETNAME)
  # Public hdrs interface:
  target_include_directories(${TARGETNAME} PUBLIC
      $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
      $<INSTALL_INTERFACE:include>
      PRIVATE src
    )

  # Dynamic libraries output options:
  # -----------------------------------
  set_target_properties(${TARGETNAME} PROPERTIES
    OUTPUT_NAME "${TARGETNAME}${MOLA_DLL_VERSION_POSTFIX}"
    COMPILE_PDB_NAME "${TARGETNAME}${MOLA_DLL_VERSION_POSTFIX}"
    COMPILE_PDB_NAME_DEBUG "${TARGETNAME}${MOLA_DLL_VERSION_POSTFIX}${CMAKE_DEBUG_POSTFIX}"
    VERSION "${MOLA_VERSION_NUMBER_MAJOR}.${MOLA_VERSION_NUMBER_MINOR}.${MOLA_VERSION_NUMBER_PATCH}"
    SOVERSION ${MOLA_VERSION_NUMBER_MAJOR}.${MOLA_VERSION_NUMBER_MINOR}
    )

  # Project "folder":
  # -------------------
  set_target_properties(${TARGETNAME} PROPERTIES FOLDER "MOLA-modules")

  # Install lib:
  install(TARGETS ${TARGETNAME} EXPORT ${TARGETNAME}-config
      ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
      LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
      RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
    )
  # Install hdrs:
  install(
    DIRECTORY include/
    DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}
  )

  # Install cmake config module
  install(EXPORT ${TARGETNAME}-config DESTINATION share/${TARGETNAME}/cmake)

  # make project importable from build_dir:
  export(
    TARGETS ${TARGETNAME}
    FILE ${TARGETNAME}-config.cmake
  )

endfunction()

# -----------------------------------------------------------------------------
# mola_configure_app(target)
#
# Define common properties of cmake-based executable projects:
# -----------------------------------------------------------------------------
function(mola_configure_app TARGETNAME)
  # Project "folder":
  set_target_properties(${TARGETNAME} PROPERTIES FOLDER "MOLA-apps")

  #TODO: install?

endfunction()
