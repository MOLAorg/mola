^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package mola_input_rosbag2
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^


2.7.0 (2026-04-22)
------------------
* Merge pull request `#128 <https://github.com/MOLAorg/mola/issues/128>`_ from MOLAorg/feat/docs-cookbook
  Feat/docs cookbook
* sanity checks; expand docs
* Docs: explain more odometry source cases
* rosbag2: Enable custom /tf names (e.g. for namespaces)
* Merge pull request `#121 <https://github.com/MOLAorg/mola/issues/121>`_ from MOLAorg/fix/clean-up-old-mrpt-version-checks
  Clean up: remove old mrpt version fallback code sections
* Contributors: Jose Luis Blanco-Claraco

2.6.1 (2026-04-02)
------------------

2.6.0 (2026-03-12)
------------------
* Support Zstd-compressed rawlog files too
* Merge pull request `#108 <https://github.com/MOLAorg/mola/issues/108>`_ from MOLAorg/feat/use-gps-msgs
  Support gps_msgs data types too for ROS2
* Support gps_msgs as alternative to NavSatFix
* Merge pull request `#107 <https://github.com/MOLAorg/mola/issues/107>`_ from MOLAorg/fix/viz-decay-clouds
  Fix/viz-decay-clouds
* Update coyright notes
* Merge pull request `#106 <https://github.com/MOLAorg/mola/issues/106>`_ from MOLAorg/refactor-rosbag2dataset
  Refactor code for handling sensors
* Refactor code for handling sensors
* Contributors: Jose Luis Blanco-Claraco

2.5.0 (2026-02-14)
------------------
* Merge pull request `#105 <https://github.com/MOLAorg/mola/issues/105>`_ from MOLAorg/feat/refactor-ros2-bridges
  Refactor to use external rosbag2 conversion in mrpt_ros_bridge
* Refactor to use external rosbag2 conversion in mrpt_ros_bridge
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
* Fix build against upcoming mrpt 2.15.4
* prepare for mrpt 2.15.4
* simplify destructors declarations
* Contributors: Jose Luis Blanco-Claraco

2.3.0 (2025-12-15)
------------------
* Import all pcd fields from ros2 messages using CGenericPointsMap
* Contributors: Jose Luis Blanco-Claraco

2.2.1 (2025-11-08)
------------------

2.2.0 (2025-10-28)
------------------
* format
* Upgrade to use the upcoming MRPT 2.15 API for CGenericsPointsMap
* Contributors: Jose Luis Blanco-Claraco

2.1.0 (2025-10-20)
------------------

2.0.0 (2025-10-13)
------------------
* fix clang-format
* Modernize copyright notice
* fix clang-format
* Contributors: Jose Luis Blanco-Claraco

1.9.1 (2025-07-07)
------------------
* fix build against latest tf2
* Contributors: Jose Luis Blanco-Claraco

1.9.0 (2025-06-06)
------------------
* Fix usage of mola_version_to_hexadecimal()
* cmake: replace local mrpt_version_to_hex() with new mrpt_common mola_version_to_hexadecimal()
* Contributors: Jose Luis Blanco-Claraco

1.8.1 (2025-05-28)
------------------

1.8.0 (2025-05-25)
------------------
* remove useless commented out dependency
* Update copyright year
* Contributors: Jose Luis Blanco-Claraco

1.7.0 (2025-05-06)
------------------

1.6.4 (2025-04-23)
------------------
* more clang-tidy fixes
* fix: Correctly handling Livox cloud timestamps ("double"s, but in nanoseconds) in BridgeROS2 and bag2 data sources. They are automatically detected, no need to change any parameter.
* modernize clang-format
* Contributors: Jose Luis Blanco-Claraco

1.6.3 (2025-03-15)
------------------
* rosbag2 input: update C++ default value to "base_link" (all launch files were already like that, here only the default value changes)
* Contributors: Jose Luis Blanco-Claraco

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

1.3.0 (2024-12-11)
------------------
* Fix typo in warning message
* Contributors: Jose Luis Blanco-Claraco

1.2.1 (2024-09-29)
------------------

1.2.0 (2024-09-16)
------------------
* Merge pull request `#70 <https://github.com/MOLAorg/mola/issues/70>`_ from MOLAorg/fix/rosbag2-memory-free
  BUGFIX: read ahead cache memory for rosbag2 was not freed
* BUGFIX: read ahead cache memory for rosbag2 was not freed if invoked via Dataset access API only
* sort <depend> entries
* Contributors: Jose Luis Blanco-Claraco

1.1.3 (2024-08-28)
------------------
* Depend on new mrpt_lib packages (deprecate mrpt2)
* Contributors: Jose Luis Blanco-Claraco

1.1.2 (2024-08-26)
------------------
* rosbag2 input: support for input directories for split bag datasets
* Contributors: Jose Luis Blanco-Claraco

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
* Better warning messages related to missing /tf messages
* Mechanism to replay datasets with mis-timestamped LiDARs
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
* FIXBUG: inverse sensor poses in rosbag2 reader.
  Also: unify notation in C++ calls to lookupTransform()
* Fix package.xml website URL
* Contributors: Jose Luis Blanco-Claraco

1.0.2 (2024-04-04)
------------------

1.0.1 (2024-03-28)
------------------
* BUGFIX: Inverted value of "use_fixed_sensor_pose" was used
* Contributors: Jose Luis Blanco-Claraco

1.0.0 (2024-03-19)
------------------
* tolerate missing tf data temporarily
* Merge ROS2 input and output in one module
* Add use_fixed_sensor_pose flag
* Fix ROS2 sensorPose from /tf data
* Fix jump forward in time in rosbags
* Implement UI Dataset in the rest of dataset sources
* New option to shutdown automatically mola-cli after dataset ends
* Automatic determination of storage id from file extension
* Show dataset replay progress percent
* Correct usage of mola:: namespace in cmake targets
* copyright update
* Hide ros2 dependencies from public .h
* Contributors: Jose Luis Blanco-Claraco

0.2.2 (2023-09-08)
------------------