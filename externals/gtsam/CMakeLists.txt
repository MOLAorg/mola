
# GTSAM: Georgia Tech Smoothing and Mapping library
# See: https://bitbucket.org/gtborg/gtsam
# BSD License

# In the beginning, gtsam was added as a CMake ExternalProject.
# But this has some strong drawbacks: it's not easy to set cache variables
# for missing dependencies (especially for Windows users), the projects
# can be exported only in tricky ways, etc.
# So, the choice is to replicate part of gtsam's own CMakeLists.txt here
# and adding the lib as a git submodule.
# Yes, I know... it might require some maintainance over time, but I think
# this is worth the benefits.

# Start of stripped version of gtsam/CMakeLists.txt
# =================================================================

project(GTSAM CXX C)

# new feature to Cmake Version > 2.8.12
# Mac ONLY. Define Relative Path on Mac OS
if(NOT DEFINED CMAKE_MACOSX_RPATH)
  set(CMAKE_MACOSX_RPATH 0)
endif()

# Set the version number for the library
set (GTSAM_VERSION_MAJOR 4)
set (GTSAM_VERSION_MINOR 0)
set (GTSAM_VERSION_PATCH 0)
math (EXPR GTSAM_VERSION_NUMERIC "10000 * ${GTSAM_VERSION_MAJOR} + 100 * ${GTSAM_VERSION_MINOR} + ${GTSAM_VERSION_PATCH}")
set (GTSAM_VERSION_STRING "${GTSAM_VERSION_MAJOR}.${GTSAM_VERSION_MINOR}.${GTSAM_VERSION_PATCH}")

###############################################################################
# Gather information, perform checks, set defaults

# Set the default install path to home
#set (CMAKE_INSTALL_PREFIX ${HOME} CACHE PATH "Install prefix for library")

set(CMAKE_MODULE_PATH "${CMAKE_MODULE_PATH}" "${CMAKE_CURRENT_SOURCE_DIR}/gtsam/cmake")
include(GtsamMakeConfigFile)

# Record the root dir for gtsam - needed during external builds, e.g., ROS
set(GTSAM_SOURCE_ROOT_DIR ${CMAKE_CURRENT_SOURCE_DIR}/gtsam/)
message(STATUS "GTSAM_SOURCE_ROOT_DIR: [${GTSAM_SOURCE_ROOT_DIR}]")

# Load build type flags and default to Debug mode
include(GtsamBuildTypes)

# Use macros for creating tests/timing scripts
include(GtsamTesting)
include(GtsamPrinting)

# guard against in-source builds
if(${CMAKE_SOURCE_DIR} STREQUAL ${CMAKE_BINARY_DIR})
  message(FATAL_ERROR "In-source builds not allowed. Please make a new directory (called a build directory) and run CMake from there. You may need to remove CMakeCache.txt. ")
endif()

# See whether gtsam_unstable is available (it will be present only if we're using a git checkout)
if(EXISTS "${PROJECT_SOURCE_DIR}/gtsam/gtsam_unstable" AND IS_DIRECTORY "${PROJECT_SOURCE_DIR}/gtsam/gtsam_unstable")
    set(GTSAM_UNSTABLE_AVAILABLE 1)
else()
    set(GTSAM_UNSTABLE_AVAILABLE 0)
endif()


###############################################################################
# Set up options

# Configurable Options
if(GTSAM_UNSTABLE_AVAILABLE)
    option(GTSAM_BUILD_UNSTABLE              "Enable/Disable libgtsam_unstable"          ON)
		mark_as_advanced(GTSAM_BUILD_UNSTABLE)
endif()
option(GTSAM_BUILD_STATIC_LIBRARY        "Build a static gtsam library, instead of shared" OFF)
mark_as_advanced(GTSAM_BUILD_STATIC_LIBRARY)
option(GTSAM_USE_QUATERNIONS             "Enable/Disable using an internal Quaternion representation for rotations instead of rotation matrices. If enable, Rot3::EXPMAP is enforced by default." OFF)
mark_as_advanced(GTSAM_USE_QUATERNIONS)
option(GTSAM_POSE3_EXPMAP 			 	 "Enable/Disable using Pose3::EXPMAP as the default mode. If disabled, Pose3::FIRST_ORDER will be used." OFF)
mark_as_advanced(GTSAM_POSE3_EXPMAP)
option(GTSAM_ROT3_EXPMAP 			 	 "Ignore if GTSAM_USE_QUATERNIONS is OFF (Rot3::EXPMAP by default). Otherwise, enable Rot3::EXPMAP, or if disabled, use Rot3::CAYLEY." OFF)
mark_as_advanced(GTSAM_ROT3_EXPMAP)
option(GTSAM_ENABLE_CONSISTENCY_CHECKS   "Enable/Disable expensive consistency checks"       OFF)
mark_as_advanced(GTSAM_ENABLE_CONSISTENCY_CHECKS)
option(GTSAM_WITH_TBB                    "Use Intel Threaded Building Blocks (TBB) if available" ON)
mark_as_advanced(GTSAM_WITH_TBB)
option(GTSAM_WITH_EIGEN_MKL              "Eigen will use Intel MKL if available" ON)
mark_as_advanced(GTSAM_WITH_EIGEN_MKL)
option(GTSAM_WITH_EIGEN_MKL_OPENMP       "Eigen, when using Intel MKL, will also use OpenMP for multithreading if available" ON)
mark_as_advanced(GTSAM_WITH_EIGEN_MKL_OPENMP)
option(GTSAM_THROW_CHEIRALITY_EXCEPTION "Throw exception when a triangulated point is behind a camera" ON)
mark_as_advanced(GTSAM_THROW_CHEIRALITY_EXCEPTION)
#option(GTSAM_BUILD_PYTHON                "Enable/Disable building & installation of Python module" OFF)
option(GTSAM_ALLOW_DEPRECATED_SINCE_V4   "Allow use of methods/functions deprecated in GTSAM 4" ON)
mark_as_advanced(GTSAM_ALLOW_DEPRECATED_SINCE_V4)
option(GTSAM_TYPEDEF_POINTS_TO_VECTORS   "Typdef Point2 and Point3 to Eigen::Vector equivalents" OFF)
mark_as_advanced(GTSAM_TYPEDEF_POINTS_TO_VECTORS)
option(GTSAM_SUPPORT_NESTED_DISSECTION   "Support Metis-based nested dissection" ON)
mark_as_advanced(GTSAM_SUPPORT_NESTED_DISSECTION)
option(GTSAM_TANGENT_PREINTEGRATION      "Use new ImuFactor with integration on tangent space" ON)
mark_as_advanced(GTSAM_TANGENT_PREINTEGRATION)

# Options relating to MATLAB wrapper
# TODO: Check for matlab mex binary before handling building of binaries
#option(GTSAM_INSTALL_MATLAB_TOOLBOX      "Enable/Disable installation of matlab toolbox"  OFF)
#option(GTSAM_INSTALL_CYTHON_TOOLBOX      "Enable/Disable installation of Cython toolbox"  OFF)
#option(GTSAM_BUILD_WRAP                  "Enable/Disable building of matlab/cython wrap utility (necessary for matlab/cython interface)" ON)

# Check / set dependent variables for MATLAB wrapper

###############################################################################
# Find boost

# To change the path for boost, you will need to set:
# BOOST_ROOT: path to install prefix for boost
# Boost_NO_SYSTEM_PATHS: set to true to keep the find script from ignoring BOOST_ROOT

if(MSVC)
	# By default, boost only builds static libraries on windows
	set(Boost_USE_STATIC_LIBS ON)  # only find static libs
	# If we ever reset above on windows and, ...
	# If we use Boost shared libs, disable auto linking.
	# Some libraries, at least Boost Program Options, rely on this to export DLL symbols.
	if(NOT Boost_USE_STATIC_LIBS)
		add_definitions(-DBOOST_ALL_NO_LIB)
	  add_definitions(-DBOOST_ALL_DYN_LINK)
	endif()
endif()

find_package(Boost 1.43 COMPONENTS serialization system filesystem thread program_options date_time timer chrono regex)

# Required components
if(NOT Boost_SERIALIZATION_LIBRARY OR NOT Boost_SYSTEM_LIBRARY OR NOT Boost_FILESYSTEM_LIBRARY OR
    NOT Boost_THREAD_LIBRARY OR NOT Boost_DATE_TIME_LIBRARY)
  message(FATAL_ERROR "Missing required Boost components >= v1.43, please install/upgrade Boost or configure your search paths. \
	You may need to set Boost_INCLUDE_DIR or Boost_DIR in CMake cache.")
endif()

option(GTSAM_DISABLE_NEW_TIMERS "Disables using Boost.chrono for timing" OFF)
mark_as_advanced(GTSAM_DISABLE_NEW_TIMERS)
# Allow for not using the timer libraries on boost < 1.48 (GTSAM timing code falls back to old timer library)
set(GTSAM_BOOST_LIBRARIES
    ${Boost_SERIALIZATION_LIBRARY} ${Boost_SYSTEM_LIBRARY} ${Boost_FILESYSTEM_LIBRARY}
	${Boost_THREAD_LIBRARY} ${Boost_DATE_TIME_LIBRARY} ${Boost_REGEX_LIBRARY})
if (GTSAM_DISABLE_NEW_TIMERS)
    message("WARNING:  GTSAM timing instrumentation manually disabled")
    add_definitions(-DGTSAM_DISABLE_NEW_TIMERS)
else()
    if(Boost_TIMER_LIBRARY)
      list(APPEND GTSAM_BOOST_LIBRARIES ${Boost_TIMER_LIBRARY} ${Boost_CHRONO_LIBRARY})
    else()
      list(APPEND GTSAM_BOOST_LIBRARIES rt) # When using the header-only boost timer library, need -lrt
      message("WARNING:  GTSAM timing instrumentation will use the older, less accurate, Boost timer library because boost older than 1.48 was found.")
    endif()
endif()


if(NOT (${Boost_VERSION} LESS 105600))
#	message("Ignoring Boost restriction on optional lvalue assignment from rvalues")
	add_definitions(-DBOOST_OPTIONAL_ALLOW_BINDING_TO_RVALUES -DBOOST_OPTIONAL_CONFIG_ALLOW_BINDING_TO_RVALUES)
endif()

###############################################################################
# Find TBB
find_package(TBB)

# Set up variables if we're using TBB
if(TBB_FOUND AND GTSAM_WITH_TBB)
	set(GTSAM_USE_TBB 1)  # This will go into config.h
	include_directories(BEFORE ${TBB_INCLUDE_DIRS})
	set(GTSAM_TBB_LIBRARIES "")
	if(TBB_DEBUG_LIBRARIES)
		foreach(lib ${TBB_LIBRARIES})
			list(APPEND GTSAM_TBB_LIBRARIES optimized "${lib}")
		endforeach()
		foreach(lib ${TBB_DEBUG_LIBRARIES})
			list(APPEND GTSAM_TBB_LIBRARIES debug "${lib}")
		endforeach()
	else()
		set(GTSAM_TBB_LIBRARIES ${TBB_LIBRARIES})
	endif()
	list(APPEND GTSAM_ADDITIONAL_LIBRARIES ${GTSAM_TBB_LIBRARIES})
else()
	set(GTSAM_USE_TBB 0)  # This will go into config.h
endif()

###############################################################################
# Prohibit Timing build mode in combination with TBB
if(GTSAM_USE_TBB AND (CMAKE_BUILD_TYPE  STREQUAL "Timing"))
      message(FATAL_ERROR "Timing build mode cannot be used together with TBB. Use a sampling profiler such as Instruments or Intel VTune Amplifier instead.")
endif()


###############################################################################
# Find Google perftools
find_package(GooglePerfTools)


###############################################################################
# Find MKL
find_package(MKL)

if(MKL_FOUND AND GTSAM_WITH_EIGEN_MKL)
    set(GTSAM_USE_EIGEN_MKL 1) # This will go into config.h
    set(EIGEN_USE_MKL_ALL 1) # This will go into config.h - it makes Eigen use MKL
    include_directories(${MKL_INCLUDE_DIR})
    list(APPEND GTSAM_ADDITIONAL_LIBRARIES ${MKL_LIBRARIES})

    # --no-as-needed is required with gcc according to the MKL link advisor
    if(CMAKE_CXX_COMPILER_ID STREQUAL "GNU")
        set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} -Wl,--no-as-needed")
    endif()
else()
    set(GTSAM_USE_EIGEN_MKL 0)
    set(EIGEN_USE_MKL_ALL 0)
endif()

###############################################################################
# Find OpenMP (if we're also using MKL)
find_package(OpenMP)  # do this here to generate correct message if disabled

if(GTSAM_WITH_EIGEN_MKL AND GTSAM_WITH_EIGEN_MKL_OPENMP AND GTSAM_USE_EIGEN_MKL)
    if(OPENMP_FOUND AND GTSAM_USE_EIGEN_MKL AND GTSAM_WITH_EIGEN_MKL_OPENMP)
        set(GTSAM_USE_EIGEN_MKL_OPENMP 1) # This will go into config.h
        set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
    endif()
endif()


###############################################################################
# Option for using system Eigen or GTSAM-bundled Eigen
### These patches only affect usage of MKL. If you want to enable MKL, you *must*
### use our patched version of Eigen
### See:  http://eigen.tuxfamily.org/bz/show_bug.cgi?id=704 (Householder QR MKL selection)
###       http://eigen.tuxfamily.org/bz/show_bug.cgi?id=705 (Fix MKL LLT return code)
option(GTSAM_USE_SYSTEM_EIGEN "Find and use system-installed Eigen. If 'off', use the one bundled with GTSAM" OFF)
mark_as_advanced(GTSAM_USE_SYSTEM_EIGEN)

# Switch for using system Eigen or GTSAM-bundled Eigen
if(GTSAM_USE_SYSTEM_EIGEN)
	find_package(Eigen3 REQUIRED)
	include_directories(AFTER "${EIGEN3_INCLUDE_DIR}")

	# Use generic Eigen include paths e.g. <Eigen/Core>
    set(GTSAM_EIGEN_INCLUDE_PREFIX "${EIGEN3_INCLUDE_DIR}")

	# check if MKL is also enabled - can have one or the other, but not both!
	# Note: Eigen >= v3.2.5 includes our patches
	if(EIGEN_USE_MKL_ALL AND (EIGEN3_VERSION VERSION_LESS 3.2.5))
	  message(FATAL_ERROR "MKL requires at least Eigen 3.2.5, and your system appears to have an older version. Disable GTSAM_USE_SYSTEM_EIGEN to use GTSAM's copy of Eigen, or disable GTSAM_WITH_EIGEN_MKL")
	endif()
else()
	# Use bundled Eigen include path.
	# Clear any variables set by FindEigen3
	if(EIGEN3_INCLUDE_DIR)
		set(EIGEN3_INCLUDE_DIR NOTFOUND CACHE STRING "" FORCE)
	endif()
	# Add the bundled version of eigen to the include path so that it can still be included
	# with  #include <Eigen/Core>
	include_directories(BEFORE "gtsam/gtsam/3rdparty/Eigen/")

	# set full path to be used by external projects
	# this will be added to GTSAM_INCLUDE_DIR by gtsam_extra.cmake.in
	set(GTSAM_EIGEN_INCLUDE_PREFIX "${CMAKE_INSTALL_PREFIX}/include/gtsam/3rdparty/Eigen/")

endif()

if (MSVC)
	if (NOT GTSAM_BUILD_STATIC_LIBRARY)
		# mute eigen static assert to avoid errors in shared lib
		add_definitions(-DEIGEN_NO_STATIC_ASSERT)
	endif()
	add_definitions(/wd4244) # Disable loss of precision which is thrown all over our Eigen
endif()

###############################################################################
# Global compile options

# Build list of possible allocators
set(possible_allocators "")
if(GTSAM_USE_TBB)
	list(APPEND possible_allocators TBB)
	set(preferred_allocator TBB)
else()
	list(APPEND possible_allocators BoostPool STL)
	set(preferred_allocator STL)
endif()
if(GOOGLE_PERFTOOLS_FOUND)
	list(APPEND possible_allocators tcmalloc)
endif()

# Check if current allocator choice is valid and set cache option
list(FIND possible_allocators "${GTSAM_DEFAULT_ALLOCATOR}" allocator_valid)
if(allocator_valid EQUAL -1)
	set(GTSAM_DEFAULT_ALLOCATOR ${preferred_allocator} CACHE STRING "Default allocator" FORCE)
else()
	set(GTSAM_DEFAULT_ALLOCATOR ${preferred_allocator} CACHE STRING "Default allocator")
endif()
set_property(CACHE GTSAM_DEFAULT_ALLOCATOR PROPERTY STRINGS ${possible_allocators})
mark_as_advanced(GTSAM_DEFAULT_ALLOCATOR)

# Define compile flags depending on allocator
if("${GTSAM_DEFAULT_ALLOCATOR}" STREQUAL "BoostPool")
	set(GTSAM_ALLOCATOR_BOOSTPOOL 1)
elseif("${GTSAM_DEFAULT_ALLOCATOR}" STREQUAL "STL")
	set(GTSAM_ALLOCATOR_STL 1)
elseif("${GTSAM_DEFAULT_ALLOCATOR}" STREQUAL "TBB")
	set(GTSAM_ALLOCATOR_TBB 1)
elseif("${GTSAM_DEFAULT_ALLOCATOR}" STREQUAL "tcmalloc")
	set(GTSAM_ALLOCATOR_STL 1) # tcmalloc replaces malloc, so to use it we use the STL allocator
	list(APPEND GTSAM_ADDITIONAL_LIBRARIES "tcmalloc")
endif()

# Include boost - use 'BEFORE' so that a specific boost specified to CMake
# takes precedence over a system-installed one.
include_directories(BEFORE SYSTEM ${Boost_INCLUDE_DIR})

if(GTSAM_SUPPORT_NESTED_DISSECTION)
  set(METIS_INCLUDE_DIRECTORIES
    gtsam/gtsam/3rdparty/metis/include
    gtsam/gtsam/3rdparty/metis/libmetis
    gtsam/gtsam/3rdparty/metis/GKlib)
else()
  set(METIS_INCLUDE_DIRECTORIES)
endif()

# Add includes for source directories 'BEFORE' boost and any system include
# paths so that the compiler uses GTSAM headers in our source directory instead
# of any previously installed GTSAM headers.
include_directories(BEFORE
  gtsam/gtsam/3rdparty/SuiteSparse_config
  gtsam/gtsam/3rdparty/CCOLAMD/Include
  ${METIS_INCLUDE_DIRECTORIES}
  ${PROJECT_SOURCE_DIR}/gtsam/
  ${PROJECT_BINARY_DIR} # So we can include generated config header files
  gtsam/CppUnitLite)

if(MSVC)
	add_definitions(-D_CRT_SECURE_NO_WARNINGS -D_SCL_SECURE_NO_WARNINGS)
	add_definitions(/wd4251 /wd4275 /wd4251 /wd4661 /wd4344 /wd4503) # Disable non-DLL-exported base class and other warnings
	add_definitions(/bigobj) # Allow large object files for template-based code
endif()

# GCC 4.8+ complains about local typedefs which we use for shared_ptr etc.
if(CMAKE_CXX_COMPILER_ID STREQUAL "GNU")
  if (NOT CMAKE_CXX_COMPILER_VERSION VERSION_LESS 4.8)
    add_definitions(-Wno-unused-local-typedefs)
  endif()
endif()

# As of XCode 7, clang also complains about this
if(CMAKE_CXX_COMPILER_ID STREQUAL "Clang")
  if (NOT CMAKE_CXX_COMPILER_VERSION VERSION_LESS 7.0)
    add_definitions(-Wno-unused-local-typedefs)
  endif()
endif()

if(GTSAM_ENABLE_CONSISTENCY_CHECKS)
  add_definitions(-DGTSAM_EXTRA_CONSISTENCY_CHECKS)
endif()

###############################################################################
# Add components

# Build CppUnitLite
#add_subdirectory(gtsam/CppUnitLite) # Not needed for MOLA

# Build wrap  # Not needed for MOLA
#if (GTSAM_BUILD_WRAP)
#    add_subdirectory(wrap)
#endif(GTSAM_BUILD_WRAP)

# Build GTSAM library
#add_subdirectory(gtsam/gtsam)
# ===== START OF EMBEDDED gtsam/gtsam ======
# We split the library in to separate subfolders, each containing
# tests, timing, and an optional convenience library.
# The following variable is the master list of subdirs to add
set (gtsam_subdirs
    gtsam/gtsam/base
    gtsam/gtsam/geometry
    gtsam/gtsam/inference
    gtsam/gtsam/symbolic
    gtsam/gtsam/discrete
    gtsam/gtsam/linear
    gtsam/gtsam/nonlinear
    gtsam/gtsam/sam
    gtsam/gtsam/sfm
    gtsam/gtsam/slam
    gtsam/gtsam/smart
    gtsam/gtsam/navigation
)

set(gtsam_srcs)

# Build 3rdparty separately
#message(STATUS "Building 3rdparty")
add_subdirectory(gtsam/gtsam/3rdparty)

set (3rdparty_srcs
 ${eigen_headers} # Set by 3rdparty/CMakeLists.txt
 ${CMAKE_CURRENT_SOURCE_DIR}/gtsam/gtsam/3rdparty/CCOLAMD/Source/ccolamd.c
 ${CMAKE_CURRENT_SOURCE_DIR}/gtsam/gtsam/3rdparty/SuiteSparse_config/SuiteSparse_config.c)
gtsam_assign_source_folders("${3rdparty_srcs}") # Create MSVC structure

# To exclude a source from the library build (in any subfolder)
# Add the full name to this list, as in the following example
# Sources to remove from builds
set (excluded_sources #"")
    "${CMAKE_CURRENT_SOURCE_DIR}/gtsam/gtsam/slam/serialization.cpp"
)

set (excluded_headers #"")
    "${CMAKE_CURRENT_SOURCE_DIR}/gtsam/gtsam/slam/serialization.h"
)

if(GTSAM_USE_QUATERNIONS)
    set(excluded_sources ${excluded_sources} "${CMAKE_CURRENT_SOURCE_DIR}/gtsam/gtsam/geometry/Rot3M.cpp")
else()
    set(excluded_sources ${excluded_sources} "${CMAKE_CURRENT_SOURCE_DIR}/gtsam/gtsam/geometry/Rot3Q.cpp")
endif()

# Common headers
file(GLOB gtsam_core_headers "*.h")
install(FILES ${gtsam_core_headers} DESTINATION include/gtsam)

# assemble core libaries
foreach(subdir ${gtsam_subdirs})
    # Build convenience libraries
    file(GLOB_RECURSE subdir_srcs "${subdir}/*.cpp" "${subdir}/*.h") # Include header files so they show up in Visual Studio
    list(REMOVE_ITEM subdir_srcs ${excluded_sources})
	file(GLOB subdir_test_files "${subdir}/tests/*")
	list(REMOVE_ITEM subdir_srcs ${subdir_test_files}) # Remove test files from sources compiled into library
    gtsam_assign_source_folders("${subdir_srcs}") # Create MSVC structure
    set(${subdir}_srcs ${subdir_srcs})

    # Build local library and tests
    #message(STATUS "Building ${subdir}")
    add_subdirectory(${subdir})
endforeach(subdir)

# To add additional sources to gtsam when building the full library (static or shared)
# append the subfolder with _srcs appended to the end to this list
set(gtsam_srcs ${3rdparty_srcs})
foreach(subdir ${gtsam_subdirs})
	list(APPEND gtsam_srcs ${${subdir}_srcs})
endforeach(subdir)
list(APPEND gtsam_srcs ${gtsam_core_headers})

IF(MSVC)
	# Add precompiled header to sources
	include(gtsamAddPch)
	gtsamAddPch("precompiled_header.h" "precompiled_header.cpp" ${gtsam_srcs})
	list(INSERT gtsam_srcs 0 "precompiled_header.cpp")
ENDIF(MSVC)

# Generate and install config and dllexport files
configure_file(gtsam/gtsam/config.h.in ${PROJECT_BINARY_DIR}/gtsam/config.h)
set(library_name GTSAM) # For substitution in dllexport.h.in
configure_file("${PROJECT_SOURCE_DIR}/gtsam/cmake/dllexport.h.in" "dllexport.h")
list(APPEND gtsam_srcs "${PROJECT_BINARY_DIR}/gtsam/config.h" "${PROJECT_BINARY_DIR}/gtsam/dllexport.h")
install(FILES "${PROJECT_BINARY_DIR}/gtsam/config.h" "${PROJECT_BINARY_DIR}/gtsam/dllexport.h" DESTINATION include/gtsam)

if(GTSAM_SUPPORT_NESTED_DISSECTION)
    list(APPEND GTSAM_ADDITIONAL_LIBRARIES metis)
endif()

# Versions
set(gtsam_version   ${GTSAM_VERSION_MAJOR}.${GTSAM_VERSION_MINOR}.${GTSAM_VERSION_PATCH})
set(gtsam_soversion ${GTSAM_VERSION_MAJOR})
message(STATUS "GTSAM Version: ${gtsam_version}")
#message(STATUS "Install prefix: ${CMAKE_INSTALL_PREFIX}")

# build shared and static versions of the library
if (GTSAM_BUILD_STATIC_LIBRARY)
    message(STATUS "Building GTSAM - static")
    add_library(gtsam STATIC ${gtsam_srcs})
	target_link_libraries(gtsam ${GTSAM_BOOST_LIBRARIES} ${GTSAM_ADDITIONAL_LIBRARIES})
    set_target_properties(gtsam PROPERTIES
        OUTPUT_NAME         gtsam
        CLEAN_DIRECT_OUTPUT 1
    	VERSION             ${gtsam_version}
     	SOVERSION           ${gtsam_soversion})
	if(WIN32) # Add 'lib' prefix to static library to avoid filename collision with shared library
		set_target_properties(gtsam PROPERTIES
			PREFIX "lib"
			COMPILE_DEFINITIONS GTSAM_IMPORT_STATIC)
	endif()
    install(TARGETS gtsam EXPORT GTSAM-exports ARCHIVE DESTINATION lib)
    list(APPEND GTSAM_EXPORTED_TARGETS gtsam)
    set(GTSAM_EXPORTED_TARGETS "${GTSAM_EXPORTED_TARGETS}" PARENT_SCOPE)
else()
    message(STATUS "Building GTSAM - shared")
    add_library(gtsam SHARED ${gtsam_srcs})
    target_link_libraries(gtsam ${GTSAM_BOOST_LIBRARIES} ${GTSAM_ADDITIONAL_LIBRARIES})
    set_target_properties(gtsam PROPERTIES
        OUTPUT_NAME         gtsam
        CLEAN_DIRECT_OUTPUT 1
    	VERSION             ${gtsam_version}
     	SOVERSION           ${gtsam_soversion})
	if(WIN32)
		set_target_properties(gtsam PROPERTIES
			PREFIX ""
			DEFINE_SYMBOL GTSAM_EXPORTS
			RUNTIME_OUTPUT_DIRECTORY "${PROJECT_BINARY_DIR}/bin")
	endif()
	if (APPLE)
		set_target_properties(gtsam PROPERTIES
			INSTALL_NAME_DIR
			"${CMAKE_INSTALL_PREFIX}/lib")
	endif()
    install(TARGETS gtsam EXPORT GTSAM-exports LIBRARY DESTINATION lib ARCHIVE DESTINATION lib RUNTIME DESTINATION bin)
    list(APPEND GTSAM_EXPORTED_TARGETS gtsam)
    set(GTSAM_EXPORTED_TARGETS "${GTSAM_EXPORTED_TARGETS}" PARENT_SCOPE)
endif()

# make sure that ccolamd compiles even in face of warnings
if(WIN32)
    set_source_files_properties(${3rdparty_srcs} PROPERTIES COMPILE_FLAGS "-w")
else()
    set_source_files_properties(${3rdparty_srcs} PROPERTIES COMPILE_FLAGS "-Wno-error")
endif()

# Special cases
if(MSVC)
	set_property(SOURCE
		"${CMAKE_CURRENT_SOURCE_DIR}/gtsam/gtsam/slam/serialization.cpp"
		"${CMAKE_CURRENT_SOURCE_DIR}/gtsam/gtsam/nonlinear/ISAM2.cpp"
		APPEND PROPERTY COMPILE_FLAGS "/bigobj")
endif()


# Create the matlab toolbox for the gtsam library
if (GTSAM_INSTALL_MATLAB_TOOLBOX)
    # Set up codegen
    include(GtsamMatlabWrap)

    # Generate, build and install toolbox
    set(mexFlags "${GTSAM_BUILD_MEX_BINARY_FLAGS}")
	if(GTSAM_BUILD_STATIC_LIBRARY)
		list(APPEND mexFlags -DGTSAM_IMPORT_STATIC)
	endif()

    # Wrap
    wrap_and_install_library(../gtsam.h "${GTSAM_ADDITIONAL_LIBRARIES}" "" "${mexFlags}")
endif ()
# ===== END OF EMBEDDED gtsam/gtsam =======

# Build Tests  # Not needed for MOLA
#add_subdirectory(gtsam/tests)

# Build examples  # Not needed for MOLA
#add_subdirectory(gtsam/examples)

# Build timing  # Not needed for MOLA
#add_subdirectory(gtsam/timing)

# Build gtsam_unstable
if (GTSAM_BUILD_UNSTABLE)
#    add_subdirectory(gtsam/gtsam_unstable)
# ===== START OF EMBEDDED gtsam/gtsam_unstable =======
# Build full gtsam_unstable library as a single library
# and also build tests
set (gtsam_unstable_subdirs
    gtsam/gtsam_unstable/base
    gtsam/gtsam_unstable/geometry
    gtsam/gtsam_unstable/linear
    gtsam/gtsam_unstable/discrete
    gtsam/gtsam_unstable/dynamics
    gtsam/gtsam_unstable/nonlinear
    gtsam/gtsam_unstable/slam
)

if(GTSAM_BUILD_METIS) # Only build partition if metis is built
    set (gtsam_unstable_subdirs ${gtsam_unstable_subdirs} partition)
endif(GTSAM_BUILD_METIS)

set(GTSAM_UNSTABLE_BOOST_LIBRARIES ${GTSAM_BOOST_LIBRARIES})

add_custom_target(check.unstable COMMAND ${CMAKE_CTEST_COMMAND} --output-on-failure)

# To exclude a source from the library build (in any subfolder)
# Add the full name to this list, as in the following example
# Sources to remove from builds
set (excluded_sources # "")
    "${CMAKE_CURRENT_SOURCE_DIR}/gtsam/gtsam_unstable/slam/serialization.cpp"
)

set (excluded_headers # "")
    "${CMAKE_CURRENT_SOURCE_DIR}/gtsam/gtsam_unstable/slam/serialization.h"
)

# assemble core libraries
set(gtsam_unstable_srcs "")
foreach(subdir ${gtsam_unstable_subdirs})
    # Build convenience libraries
    file(GLOB subdir_srcs "${CMAKE_CURRENT_SOURCE_DIR}/${subdir}/*.cpp")
	file(GLOB subdir_headers "${CMAKE_CURRENT_SOURCE_DIR}/${subdir}/*.h")
    list(REMOVE_ITEM subdir_srcs ${excluded_sources})
    list(REMOVE_ITEM subdir_headers ${excluded_headers})
    set(${subdir}_srcs ${subdir_srcs} ${subdir_headers})
    gtsam_assign_source_folders("${${subdir}_srcs}") # Create MSVC structure

    # assemble gtsam_unstable components
    list(APPEND gtsam_unstable_srcs ${subdir_srcs})

    # Build local library and tests
    message(STATUS "Building ${subdir}_unstable")
    add_subdirectory(${subdir})
endforeach(subdir)

if(GTSAM_BUILD_METIS) # Only build partition if metis is built
    set (gtsam_unstable_srcs ${gtsam_unstable_srcs} ${partition_srcs})
endif(GTSAM_BUILD_METIS)

# Versions - same as core gtsam library
set(gtsam_unstable_version   ${GTSAM_VERSION_MAJOR}.${GTSAM_VERSION_MINOR}.${GTSAM_VERSION_PATCH})
set(gtsam_unstable_soversion ${GTSAM_VERSION_MAJOR})
message(STATUS "GTSAM_UNSTABLE Version: ${gtsam_unstable_version}")
#message(STATUS "Install prefix: ${CMAKE_INSTALL_PREFIX}")

# build shared or static versions of the library
if (GTSAM_BUILD_STATIC_LIBRARY)
	message(STATUS "Building GTSAM_UNSTABLE - static")
	add_library(gtsam_unstable STATIC ${gtsam_unstable_srcs})
	set_target_properties(gtsam_unstable PROPERTIES
		OUTPUT_NAME         gtsam_unstable
		CLEAN_DIRECT_OUTPUT 1
		VERSION             ${gtsam_unstable_version}
		SOVERSION           ${gtsam_unstable_soversion})
	if(WIN32) # Add 'lib' prefix to static library to avoid filename collision with shared library
		set_target_properties(gtsam_unstable PROPERTIES
			PREFIX "lib"
			COMPILE_DEFINITIONS GTSAM_UNSTABLE_IMPORT_STATIC)
	endif()
	target_link_libraries(gtsam_unstable gtsam ${GTSAM_UNSTABLE_BOOST_LIBRARIES})
	install(TARGETS gtsam_unstable EXPORT GTSAM-exports ARCHIVE DESTINATION lib)
	list(APPEND GTSAM_EXPORTED_TARGETS gtsam_unstable)
	set(GTSAM_EXPORTED_TARGETS "${GTSAM_EXPORTED_TARGETS}" PARENT_SCOPE)
else()
    message(STATUS "Building GTSAM_UNSTABLE - shared")
    add_library(gtsam_unstable SHARED ${gtsam_unstable_srcs})
    set_target_properties(gtsam_unstable PROPERTIES
        OUTPUT_NAME         gtsam_unstable
        CLEAN_DIRECT_OUTPUT 1
    	VERSION             ${gtsam_unstable_version}
     	SOVERSION           ${gtsam_unstable_soversion})
	if(WIN32)
		set_target_properties(gtsam_unstable PROPERTIES
			PREFIX ""
			DEFINE_SYMBOL GTSAM_UNSTABLE_EXPORTS
			RUNTIME_OUTPUT_DIRECTORY "${PROJECT_BINARY_DIR}/bin")
	endif()
    target_link_libraries(gtsam_unstable gtsam ${GTSAM_UNSTABLE_BOOST_LIBRARIES})
    install(TARGETS gtsam_unstable EXPORT GTSAM-exports LIBRARY DESTINATION lib ARCHIVE DESTINATION lib RUNTIME DESTINATION bin)
    list(APPEND GTSAM_EXPORTED_TARGETS gtsam_unstable)
    set(GTSAM_EXPORTED_TARGETS "${GTSAM_EXPORTED_TARGETS}" PARENT_SCOPE)
endif()
# ===== END OF EMBEDDED gtsam/gtsam_unstable =======
endif(GTSAM_BUILD_UNSTABLE)

# Not needed for MOLA:
# Matlab toolbox
# Python wrap
# Cython wrap

# Install config and export files
GtsamMakeConfigFile(GTSAM "${CMAKE_CURRENT_SOURCE_DIR}/gtsam/gtsam_extra.cmake.in")
export(TARGETS ${GTSAM_EXPORTED_TARGETS} FILE GTSAM-exports.cmake)

# Check for doxygen availability - optional dependency
#find_package(Doxygen)

# CMake Tools
add_subdirectory(gtsam/cmake)

# CPack
# ...

###############################################################################
# Print configuration variables
message(STATUS "================  GTSAM Configuration Options  ===============")
message(STATUS "  CMAKE_CXX_COMPILER_ID type     : ${CMAKE_CXX_COMPILER_ID}")
message(STATUS "  CMAKE_CXX_COMPILER_VERSION     : ${CMAKE_CXX_COMPILER_VERSION}")
message(STATUS "Build flags                                               ")
print_config_flag(${GTSAM_BUILD_TESTS}                 "Build Tests                    ")
print_config_flag(${GTSAM_BUILD_EXAMPLES_ALWAYS}       "Build examples with 'make all' ")
print_config_flag(${GTSAM_BUILD_TIMING_ALWAYS}         "Build timing scripts with 'make all'")
if(GTSAM_UNSTABLE_AVAILABLE)
    print_config_flag(${GTSAM_BUILD_UNSTABLE}          "Build libgtsam_unstable        ")
endif()
string(TOUPPER "${CMAKE_BUILD_TYPE}" cmake_build_type_toupper)
if(NOT MSVC AND NOT XCODE_VERSION)
    message(STATUS "  Build type                     : ${CMAKE_BUILD_TYPE}")
    message(STATUS "  C compilation flags            : ${CMAKE_C_FLAGS} ${CMAKE_C_FLAGS_${cmake_build_type_toupper}}")
    message(STATUS "  C++ compilation flags          : ${CMAKE_CXX_FLAGS} ${CMAKE_CXX_FLAGS_${cmake_build_type_toupper}}")
endif()
if(GTSAM_USE_SYSTEM_EIGEN)
    message(STATUS "  Use System Eigen               : Yes")
else()
    message(STATUS "  Use System Eigen               : No")
endif()
if(GTSAM_USE_TBB)
	message(STATUS "  Use Intel TBB                  : Yes")
elseif(TBB_FOUND)
	message(STATUS "  Use Intel TBB                  : TBB found but GTSAM_WITH_TBB is disabled")
else()
	message(STATUS "  Use Intel TBB                  : TBB not found")
endif()
if(GTSAM_USE_EIGEN_MKL)
	message(STATUS "  Eigen will use MKL             : Yes")
elseif(MKL_FOUND)
	message(STATUS "  Eigen will use MKL             : MKL found but GTSAM_WITH_EIGEN_MKL is disabled")
else()
	message(STATUS "  Eigen will use MKL             : MKL not found")
endif()
if(GTSAM_USE_EIGEN_MKL_OPENMP)
	message(STATUS "  Eigen will use MKL and OpenMP  : Yes")
elseif(OPENMP_FOUND AND NOT GTSAM_WITH_EIGEN_MKL)
	message(STATUS "  Eigen will use MKL and OpenMP  : OpenMP found but GTSAM_WITH_EIGEN_MKL is disabled")
elseif(OPENMP_FOUND AND NOT MKL_FOUND)
	message(STATUS "  Eigen will use MKL and OpenMP  : OpenMP found but MKL not found")
elseif(OPENMP_FOUND)
	message(STATUS "  Eigen will use MKL and OpenMP  : OpenMP found but GTSAM_WITH_EIGEN_MKL_OPENMP is disabled")
else()
	message(STATUS "  Eigen will use MKL and OpenMP  : OpenMP not found")
endif()
message(STATUS "  Default allocator              : ${GTSAM_DEFAULT_ALLOCATOR}")
message(STATUS "GTSAM flags                                               ")
print_config_flag(${GTSAM_USE_QUATERNIONS}             "Quaternions as default Rot3     ")
print_config_flag(${GTSAM_ENABLE_CONSISTENCY_CHECKS}   "Runtime consistency checking    ")
print_config_flag(${GTSAM_ROT3_EXPMAP}                 "Rot3 retract is full ExpMap     ")
print_config_flag(${GTSAM_POSE3_EXPMAP}                "Pose3 retract is full ExpMap    ")
print_config_flag(${GTSAM_ALLOW_DEPRECATED_SINCE_V4}   "Deprecated in GTSAM 4 allowed   ")
print_config_flag(${GTSAM_TYPEDEF_POINTS_TO_VECTORS}          "Point3 is typedef to Vector3    ")
print_config_flag(${GTSAM_SUPPORT_NESTED_DISSECTION}   "Metis-based Nested Dissection   ")
print_config_flag(${GTSAM_TANGENT_PREINTEGRATION}      "Use tangent-space preintegration")
message(STATUS "===============================================================")
