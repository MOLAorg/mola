^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package mola_demos
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

2.4.0 (2025-12-28)
------------------

2.3.0 (2025-12-15)
------------------

2.2.1 (2025-11-08)
------------------

2.2.0 (2025-10-28)
------------------

2.1.0 (2025-10-20)
------------------

2.0.0 (2025-10-13)
------------------
* Merge pull request `#93 <https://github.com/MOLAorg/mola/issues/93>`_ from MOLAorg/feature/better-lio
  Changes for new LIO
* Add new module to replay LiDAR datasets from a directory with .bin files
* Normalize intensity in published point clouds
* Use ament linters
* Contributors: Jose Luis Blanco-Claraco

1.9.1 (2025-07-07)
------------------

1.9.0 (2025-06-06)
------------------
* Fix silent cmake warnings on unused variables
* Contributors: Jose Luis Blanco-Claraco

1.8.1 (2025-05-28)
------------------
* Fix: Do not use the deprecated ament_target_dependencies()
* Contributors: Jose Luis Blanco-Claraco

1.8.0 (2025-05-25)
------------------
* silent cmake warning when using CMAKE_EXPORT_COMPILE_COMMANDS
* Update license tag to "BSD-3-Clause"
* Update copyright year
* Contributors: Jose Luis Blanco-Claraco

1.7.0 (2025-05-06)
------------------
* Implement live camera mode too
* video input from video files also implemented now
* Merge pull request `#85 <https://github.com/MOLAorg/mola/issues/85>`_ from MOLAorg/feat/video-input-module
  Feature: new video input MOLA module
* Feature: new video input MOLA module. For now, implements "image directory" input.
* Contributors: Jose Luis Blanco-Claraco

1.6.4 (2025-04-23)
------------------

1.6.3 (2025-03-15)
------------------

1.6.2 (2025-02-22)
------------------

1.6.1 (2025-02-13)
------------------

1.6.0 (2025-01-21)
------------------

1.5.1 (2024-12-29)
------------------

1.5.0 (2024-12-26)
------------------

1.4.1 (2024-12-20)
------------------

1.4.0 (2024-12-18)
------------------
* MOLA system yaml files: added "enabled" optional property for modules and rds visualizers
* Contributors: Jose Luis Blanco-Claraco

1.3.0 (2024-12-11)
------------------
* Update ROS2 demos for Mulran replay
* mola_viz: Show IMU data in the GUI too
* Contributors: Jose Luis Blanco-Claraco

1.2.1 (2024-09-29)
------------------

1.2.0 (2024-09-16)
------------------

1.1.3 (2024-08-28)
------------------

1.1.2 (2024-08-26)
------------------

1.1.1 (2024-08-23)
------------------

1.1.0 (2024-08-18)
------------------
* Merge pull request `#62 <https://github.com/MOLAorg/mola/issues/62>`_ from MOLAorg/docs-fixes
  Docs fixes
* Fix ament_xmllint warnings in package.xml
* Contributors: Jose Luis Blanco-Claraco

1.0.8 (2024-07-29)
------------------
* ament_lint_cmake: clean warnings
* Contributors: Jose Luis Blanco-Claraco

1.0.7 (2024-07-24)
------------------
* Fix GNSS typo
* Contributors: Jose Luis Blanco-Claraco

1.0.6 (2024-06-21)
------------------

1.0.5 (2024-05-28)
------------------

1.0.4 (2024-05-14)
------------------
* bump cmake_minimum_required to 3.5
* Contributors: Jose Luis Blanco-Claraco

1.0.3 (2024-04-22)
------------------
* ROS2 demo yaml files: fix missing entry and unify notation with mola_lidar_odometry
* FIXBUG: inverse sensor poses in rosbag2 reader.
  Also: unify notation in C++ calls to lookupTransform()
* Contributors: Jose Luis Blanco-Claraco

1.0.2 (2024-04-04)
------------------

1.0.1 (2024-03-28)
------------------
* Clean up yaml files.
* PF localization demo for Mulran dataset
* renamed ros launch files for clear autocompletion with ros2 launch
* Contributors: Jose Luis Blanco-Claraco

1.0.0 (2024-03-19)
------------------
* Add use_fixed_sensor_pose flag
* Publish ground truth to ROS2 too
* ROS 2 launch demos
* mola-cli now does not need the -c cli flag
* More generic demo launch
* reorganize as monorepo
* Contributors: Jose Luis Blanco-Claraco

0.2.2 (2023-09-08)
------------------
* Progress with demo
* Import first demo files
* Contributors: Jose Luis Blanco-Claraco

0.2.1 (2023-09-08)
------------------
* Initial commit
* Contributors: Jose Luis Blanco-Claraco
