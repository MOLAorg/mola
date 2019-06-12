# ------------------------------------------------------------------------------
#        A Modular Optimization framework for Localization and mApping
#                               (MOLA)
# ------------------------------------------------------------------------------

project(all_modules LANGUAGES CXX)

# mola-common: already included from top-level cmake script

# System kernel:
add_subdirectory(mola-kernel)
# Launcher app:
add_subdirectory(mola-launcher)


# Modules:
add_subdirectory(mola-fe-g2o-dataset)
add_subdirectory(mola-fe-imu-preintegration)
add_subdirectory(mola-lidar-segmentation)
add_subdirectory(mola-fe-lidar)
#add_subdirectory(mola-fe-visual-orb) # Under development
add_subdirectory(mola-input-euroc-dataset)
add_subdirectory(mola-input-hwdrivers)
add_subdirectory(mola-input-kaist-dataset)
add_subdirectory(mola-input-kitti-dataset)
add_subdirectory(mola-input-rawlog)
add_subdirectory(mola-slam-gtsam)
add_subdirectory(mola-viz)
