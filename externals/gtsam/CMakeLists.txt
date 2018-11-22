
# GTSAM: Georgia Tech Smoothing and Mapping library
# See: https://bitbucket.org/gtborg/gtsam
# BSD License

include(ExternalProject)

# download
ExternalProject_Add(EP_gtsam
  GIT_REPOSITORY   https://bitbucket.org/gtborg/gtsam.git
  GIT_TAG          6f8bfe0f0a10a94db26b7dda6b5e1d1e6b8e4b72  # Nov-2018
  GIT_PROGRESS     ON
  GIT_SHALLOW      ON
  PREFIX           EP_gtsam
  SOURCE_DIR       "${mola_BINARY_DIR}/EP_gtsam/src"
  CMAKE_ARGS
  -DMETIS_SHARED=OFF   # it causes problems (Linux)
  -DGTSAM_BUILD_DOCS=OFF
  -DCMAKE_INSTALL_PREFIX=${mola_BINARY_DIR}/EP_gtsam-install/
  -DGTSAM_BUILD_EXAMPLES_ALWAYS=OFF
  -DGTSAM_BUILD_PYTHON=OFF
  -DGTSAM_BUILD_TESTS=OFF
  -DGTSAM_BUILD_WRAP=OFF
#  -DGTSAM_WITH_TBB=OFF
#  -DGTSAM_WITH_EIGEN_MKL=OFF
  -DCMAKE_LIBRARY_OUTPUT_PATH=${mola_BINARY_DIR}/lib
  -DLIBRARY_OUTPUT_PATH=${mola_BINARY_DIR}/lib
  -DCMAKE_RUNTIME_OUTPUT_DIRECTORY=${mola_BINARY_DIR}/bin
  -DRUNTIME_OUTPUT_DIRECTORY=${mola_BINARY_DIR}/bin
  TEST_COMMAND      ""
)

# Imported CMake target:
ExternalProject_Get_Property(EP_gtsam install_dir)

add_library(gtsam INTERFACE)
target_link_libraries(gtsam
  INTERFACE
  ${install_dir}/lib/libgtsam.so
)
target_include_directories(gtsam
  INTERFACE
  ${install_dir}/include
)
# Enforce building EP first:
add_dependencies(gtsam EP_gtsam)

# Or, using find_package ? Not sure if possible, since EP_gtsam does not build
# during cmake configure...
#find_package(GTSAM REQUIRED)
#add_executable(${PROJECT_NAME} main.cpp)
#target_link_libraries(${PROJECT_NAME}  PRIVATE gtsam)
#target_include_directories(${PROJECT_NAME} PRIVATE ${GTSAM_INCLUDE_DIR})
