
# Backwards compatibility for ROS2 Humble
#cmake_minimum_required(VERSION 3.22...3.31)  # Ubuntu 22.04+
cmake_minimum_required(VERSION 3.10)

project(mola_msgs)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find for these packages, and skip the package if we are not using ROS 2:
find_package(ament_cmake QUIET)
if(NOT ament_cmake_FOUND)
  message(STATUS "Skipping mola_msgs: ament_cmake not found, it seems we are not using ROS 2.")
  return()
endif()

find_package(rosidl_default_generators REQUIRED)
find_package(geometry_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/MapLoad.srv"
  "srv/MapSave.srv"
  "srv/MolaRuntimeParamGet.srv"
  "srv/MolaRuntimeParamSet.srv"
  "srv/RelocalizeFromStateEstimator.srv"
  "srv/RelocalizeNearPose.srv"
  DEPENDENCIES
  std_msgs
  geometry_msgs
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
