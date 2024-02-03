[![CI Check clang-format](https://github.com/MOLAorg/mola/actions/workflows/check-clang-format.yml/badge.svg)](https://github.com/MOLAorg/mola/actions/workflows/check-clang-format.yml)
[![CI ROS](https://github.com/MOLAorg/mola/actions/workflows/build-ros.yml/badge.svg)](https://github.com/MOLAorg/mola/actions/workflows/build-ros.yml)
[![CircleCI](https://img.shields.io/circleci/build/gh/MOLAorg/mola/develop.svg)](https://circleci.com/gh/MOLAorg/mola)
[![Docs](https://img.shields.io/badge/docs-latest-brightgreen.svg)](https://docs.mola-slam.org/latest/)

| Distro | Build dev | Release |
| --- | --- | --- |
| ROS 2 Humble (u22.04) | [![Build Status](https://build.ros2.org/job/Hdev__mola__ubuntu_jammy_amd64/badge/icon)](https://build.ros2.org/job/Hdev__mola__ubuntu_jammy_amd64/) | [![Version](https://img.shields.io/ros/v/humble/mola)](https://index.ros.org/search/?term=mola) |
| ROS 2 Iron (u22.04) | [![Build Status](https://build.ros2.org/job/Idev__mola__ubuntu_jammy_amd64/badge/icon)](https://build.ros2.org/job/Idev__mola__ubuntu_jammy_amd64/) | [![Version](https://img.shields.io/ros/v/iron/mola)](https://index.ros.org/search/?term=mola) |
| ROS 2 Rolling (u22.04) | [![Build Status](https://build.ros2.org/job/Rdev__mola__ubuntu_jammy_amd64/badge/icon)](https://build.ros2.org/job/Rdev__mola__ubuntu_jammy_amd64/) | [![Version](https://img.shields.io/ros/v/rolling/mola)](https://index.ros.org/search/?term=mola) |

# mola
A Modular Optimization framework for Localization and mApping (MOLA).
This repository contains the base open-sourced packages for the MOLA project. Other packages exist under the [MOLAorg](https://github.com/MOLAorg/) organization.

Refer to the [official documentation](https://docs.mola-slam.org/latest/) for build instructions, demos, API reference, etc.

<center><img src="https://molaorg.github.io/static-media/kitti_demo_06.gif" /></center>


## Individual package build status

| Package | ROS 2 Humble <br/> BinBuild |  ROS 2 Iron <br/> BinBuild |  ROS 2 Rolling <br/> BinBuild |
| --- | --- | --- | --- |
| mola_demos | [![Build Status](https://build.ros2.org/job/Hbin_uJ64__mola_demos__ubuntu_jammy_amd64__binary/badge/icon)](https://build.ros2.org/job/Hbin_uJ64__mola_demos__ubuntu_jammy_amd64__binary/) | [![Build Status](https://build.ros2.org/job/Ibin_uJ64__mola_demos__ubuntu_jammy_amd64__binary/badge/icon)](https://build.ros2.org/job/Ibin_uJ64__mola_demos__ubuntu_jammy_amd64__binary/)| [![Build Status](https://build.ros2.org/job/Rbin_uJ64__mola_demos__ubuntu_jammy_amd64__binary/badge/icon)](https://build.ros2.org/job/Rbin_uJ64__mola_demos__ubuntu_jammy_amd64__binary/) |
| mola_imu_preintegration | [![Build Status](https://build.ros2.org/job/Hbin_uJ64__mola_imu_preintegration__ubuntu_jammy_amd64__binary/badge/icon)](https://build.ros2.org/job/Hbin_uJ64__mola_imu_preintegration__ubuntu_jammy_amd64__binary/) | [![Build Status](https://build.ros2.org/job/Ibin_uJ64__mola_imu_preintegration__ubuntu_jammy_amd64__binary/badge/icon)](https://build.ros2.org/job/Ibin_uJ64__mola_imu_preintegration__ubuntu_jammy_amd64__binary/)| [![Build Status](https://build.ros2.org/job/Rbin_uJ64__mola_imu_preintegration__ubuntu_jammy_amd64__binary/badge/icon)](https://build.ros2.org/job/Rbin_uJ64__mola_imu_preintegration__ubuntu_jammy_amd64__binary/) |
| mola_input_euroc_dataset | [![Build Status](https://build.ros2.org/job/Hbin_uJ64__mola_input_euroc_dataset__ubuntu_jammy_amd64__binary/badge/icon)](https://build.ros2.org/job/Hbin_uJ64__mola_input_euroc_dataset__ubuntu_jammy_amd64__binary/) | [![Build Status](https://build.ros2.org/job/Ibin_uJ64__mola_input_euroc_dataset__ubuntu_jammy_amd64__binary/badge/icon)](https://build.ros2.org/job/Ibin_uJ64__mola_input_euroc_dataset__ubuntu_jammy_amd64__binary/)| [![Build Status](https://build.ros2.org/job/Rbin_uJ64__mola_input_euroc_dataset__ubuntu_jammy_amd64__binary/badge/icon)](https://build.ros2.org/job/Rbin_uJ64__mola_input_euroc_dataset__ubuntu_jammy_amd64__binary/) |
| mola_input_kitti_dataset | [![Build Status](https://build.ros2.org/job/Hbin_uJ64__mola_input_kitti_dataset__ubuntu_jammy_amd64__binary/badge/icon)](https://build.ros2.org/job/Hbin_uJ64__mola_input_kitti_dataset__ubuntu_jammy_amd64__binary/) | [![Build Status](https://build.ros2.org/job/Ibin_uJ64__mola_input_kitti_dataset__ubuntu_jammy_amd64__binary/badge/icon)](https://build.ros2.org/job/Ibin_uJ64__mola_input_kitti_dataset__ubuntu_jammy_amd64__binary/)| [![Build Status](https://build.ros2.org/job/Rbin_uJ64__mola_input_kitti_dataset__ubuntu_jammy_amd64__binary/badge/icon)](https://build.ros2.org/job/Rbin_uJ64__mola_input_kitti_dataset__ubuntu_jammy_amd64__binary/) |
| mola_input_rawlog | [![Build Status](https://build.ros2.org/job/Hbin_uJ64__mola_input_rawlog__ubuntu_jammy_amd64__binary/badge/icon)](https://build.ros2.org/job/Hbin_uJ64__mola_input_rawlog__ubuntu_jammy_amd64__binary/) | [![Build Status](https://build.ros2.org/job/Ibin_uJ64__mola_input_rawlog__ubuntu_jammy_amd64__binary/badge/icon)](https://build.ros2.org/job/Ibin_uJ64__mola_input_rawlog__ubuntu_jammy_amd64__binary/)| [![Build Status](https://build.ros2.org/job/Rbin_uJ64__mola_input_rawlog__ubuntu_jammy_amd64__binary/badge/icon)](https://build.ros2.org/job/Rbin_uJ64__mola_input_rawlog__ubuntu_jammy_amd64__binary/) |
| mola_input_ros2 | [![Build Status](https://build.ros2.org/job/Hbin_uJ64__mola_input_ros2__ubuntu_jammy_amd64__binary/badge/icon)](https://build.ros2.org/job/Hbin_uJ64__mola_input_ros2__ubuntu_jammy_amd64__binary/) | [![Build Status](https://build.ros2.org/job/Ibin_uJ64__mola_input_ros2__ubuntu_jammy_amd64__binary/badge/icon)](https://build.ros2.org/job/Ibin_uJ64__mola_input_ros2__ubuntu_jammy_amd64__binary/)| [![Build Status](https://build.ros2.org/job/Rbin_uJ64__mola_input_ros2__ubuntu_jammy_amd64__binary/badge/icon)](https://build.ros2.org/job/Rbin_uJ64__mola_input_ros2__ubuntu_jammy_amd64__binary/) |
| mola_kernel | [![Build Status](https://build.ros2.org/job/Hbin_uJ64__mola_kernel__ubuntu_jammy_amd64__binary/badge/icon)](https://build.ros2.org/job/Hbin_uJ64__mola_kernel__ubuntu_jammy_amd64__binary/) | [![Build Status](https://build.ros2.org/job/Ibin_uJ64__mola_kernel__ubuntu_jammy_amd64__binary/badge/icon)](https://build.ros2.org/job/Ibin_uJ64__mola_kernel__ubuntu_jammy_amd64__binary/)| [![Build Status](https://build.ros2.org/job/Rbin_uJ64__mola_kernel__ubuntu_jammy_amd64__binary/badge/icon)](https://build.ros2.org/job/Rbin_uJ64__mola_kernel__ubuntu_jammy_amd64__binary/) |
| mola_launcher | [![Build Status](https://build.ros2.org/job/Hbin_uJ64__mola_launcher__ubuntu_jammy_amd64__binary/badge/icon)](https://build.ros2.org/job/Hbin_uJ64__mola_launcher__ubuntu_jammy_amd64__binary/) | [![Build Status](https://build.ros2.org/job/Ibin_uJ64__mola_launcher__ubuntu_jammy_amd64__binary/badge/icon)](https://build.ros2.org/job/Ibin_uJ64__mola_launcher__ubuntu_jammy_amd64__binary/)| [![Build Status](https://build.ros2.org/job/Rbin_uJ64__mola_launcher__ubuntu_jammy_amd64__binary/badge/icon)](https://build.ros2.org/job/Rbin_uJ64__mola_launcher__ubuntu_jammy_amd64__binary/) |
| mola_viz | [![Build Status](https://build.ros2.org/job/Hbin_uJ64__mola_viz__ubuntu_jammy_amd64__binary/badge/icon)](https://build.ros2.org/job/Hbin_uJ64__mola_viz__ubuntu_jammy_amd64__binary/) | [![Build Status](https://build.ros2.org/job/Ibin_uJ64__mola_viz__ubuntu_jammy_amd64__binary/badge/icon)](https://build.ros2.org/job/Ibin_uJ64__mola_viz__ubuntu_jammy_amd64__binary/)| [![Build Status](https://build.ros2.org/job/Rbin_uJ64__mola_viz__ubuntu_jammy_amd64__binary/badge/icon)](https://build.ros2.org/job/Rbin_uJ64__mola_viz__ubuntu_jammy_amd64__binary/) |
| mola_yaml | [![Build Status](https://build.ros2.org/job/Hbin_uJ64__mola_yaml__ubuntu_jammy_amd64__binary/badge/icon)](https://build.ros2.org/job/Hbin_uJ64__mola_yaml__ubuntu_jammy_amd64__binary/) | [![Build Status](https://build.ros2.org/job/Ibin_uJ64__mola_yaml__ubuntu_jammy_amd64__binary/badge/icon)](https://build.ros2.org/job/Ibin_uJ64__mola_yaml__ubuntu_jammy_amd64__binary/)| [![Build Status](https://build.ros2.org/job/Rbin_uJ64__mola_yaml__ubuntu_jammy_amd64__binary/badge/icon)](https://build.ros2.org/job/Rbin_uJ64__mola_yaml__ubuntu_jammy_amd64__binary/) |

## Citation

The latest publications on MOLA are: 

    mola_lidar_odometry: TBD!

MOLA was initially presented in 2019 in ([PDF](http://www.roboticsproceedings.org/rss15/p43.pdf)):

```bibtex
@INPROCEEDINGS{Blanco-Claraco-RSS-19, 
    AUTHOR    = {Jose Luis Blanco-Claraco}, 
    TITLE     = {A Modular Optimization Framework for Localization and Mapping}, 
    BOOKTITLE = {Proceedings of Robotics: Science and Systems}, 
    YEAR      = {2019}, 
    ADDRESS   = {FreiburgimBreisgau, Germany}, 
    MONTH     = {June}, 
    DOI       = {10.15607/RSS.2019.XV.043} 
} 
```

## License
MOLA is released under the GNU GPL v3 license, except noted otherwise in each individual module. Other options available upon request.
Some modules are released under BSD-3. See the [official documentation](https://docs.mola-slam.org/latest/).
