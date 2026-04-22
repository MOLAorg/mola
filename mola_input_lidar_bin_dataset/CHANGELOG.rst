^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package mola_input_lidar_bin_dataset
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

2.7.0 (2026-04-22)
------------------

2.6.1 (2026-04-02)
------------------

2.6.0 (2026-03-12)
------------------
* Merge pull request `#108 <https://github.com/MOLAorg/mola/issues/108>`_ from MOLAorg/feat/use-gps-msgs
  Support gps_msgs data types too for ROS2
* Remove useless cmake macro argument
* Merge pull request `#107 <https://github.com/MOLAorg/mola/issues/107>`_ from MOLAorg/fix/viz-decay-clouds
  Fix/viz-decay-clouds
* Update coyright notes
* Contributors: Jose Luis Blanco-Claraco

2.5.0 (2026-02-14)
------------------
* Merge pull request `#100 <https://github.com/MOLAorg/mola/issues/100>`_ from MOLAorg/fix/remove-mrpt-deprecated-maps
  Remove use of mrpt deprecated maps
* Avoid use of deprecated mrpt::maps classes
* Merge pull request `#99 <https://github.com/MOLAorg/mola/issues/99>`_ from MOLAorg/feat/ros2-bridge-pub-geographic
  ROS2 bridge: publish geographic poses too
* ros2 bridge: use geographic_msgs, store the last georeference info internally, and publish georef poses
  merge of these commits:
  - Enable many more clang-tidy checks
  - lint clean
  - implement publishing georeferenced poses
  - mola-viz: fix potential crash on edge case with all points having NaN value
  - FIX: potential crash if no MapServer is present and map services are called
* Contributors: Jose Luis Blanco-Claraco

2.4.0 (2025-12-28)
------------------
* fix clang-tidy warnings
* Prepare for deprecated mrpt::maps classes towards mrpt 3.0.0
* Contributors: Jose Luis Blanco-Claraco

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
* Contributors: Jose Luis Blanco-Claraco

1.9.1 (2025-07-07)
------------------
