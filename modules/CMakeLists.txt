# ------------------------------------------------------------------------------
#        A Modular Optimization framework for Localization and mApping
#                               (MOLA)
# ------------------------------------------------------------------------------

project(all_modules LANGUAGES CXX)

# System kernel:
add_subdirectory(mola-kernel)
# Launcher app:
add_subdirectory(mola-launcher)

# Modules:
add_subdirectory(mola-fe-lidar-icp)
add_subdirectory(mola-sensor-kitti-dataset)
add_subdirectory(mola-slam-gtsam)
