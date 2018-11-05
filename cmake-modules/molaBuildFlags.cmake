

# We want libraries to be named "libXXX" in all compilers, "libXXX-dbg" in MSVC
set(CMAKE_SHARED_LIBRARY_PREFIX "lib")
set(CMAKE_STATIC_LIBRARY_PREFIX "lib")
if(MSVC)
  set(CMAKE_DEBUG_POSTFIX "-dbg")
else()
  set(CMAKE_DEBUG_POSTFIX "")
endif()

# Set defaults for each project CMake target:
function(mola_set_target_build_options TARGETNAME)
  # Build for C++17
  # -------------------------
  target_compile_features(${TARGETNAME} INTERFACE cxx_std_17)
  if (MSVC)
    # this seems to be required in addition to the cxx_std_17 above (?)
    add_compile_options(${TARGETNAME} INTERFACE /std:c++latest)
  endif()

  # Warning level:
  # -------------------------
  if (MSVC)
    # msvc:
    add_compile_options(${TARGETNAME} PRIVATE /W3)
    target_compile_definitions(${TARGETNAME} PRIVATE
      _CRT_SECURE_NO_DEPRECATE
      _CRT_NONSTDC_NO_DEPRECATE
      _SILENCE_ALL_CXX17_DEPRECATION_WARNINGS
    )
  else()
    # gcc & clang:
    target_compile_options(${TARGETNAME} PRIVATE
      -Wall -Wextra -Wshadow -Wreturn-type -Wabi
      -Wtype-limits -Wcast-align -Wparentheses
      -fPIC
    )
  endif()

  # Optimization:
  # -------------------------
  if((NOT MSVC) AND ((NOT CMAKE_CROSSCOMPILING) AND (NOT CMAKE_BUILD_TYPE STREQUAL "Debug")))
    add_compile_options(${TARGETNAME} PRIVATE -O3 -mtune=native)
  endif()
endfunction()

# Define a consistent install behavior for cmake-based library project:
function(mola_define_library TARGETNAME)
  # Public hdrs interface:
  target_include_directories(${TARGETNAME} PUBLIC
      $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
      $<INSTALL_INTERFACE:include>
      PRIVATE src
    )

  # Install lib:
  install(TARGETS ${TARGETNAME} EXPORT ${TARGETNAME}Config
      ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
      LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
      RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
    )
  # Install hdrs:
  install(DIRECTORY include/ DESTINATION ${CMAKE_INSTALL_INCLUDEDIR})

  # Install cmake config module
  install(EXPORT ${TARGETNAME}Config DESTINATION share/${TARGETNAME}/cmake)  
endfunction()
