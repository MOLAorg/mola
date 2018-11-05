

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
