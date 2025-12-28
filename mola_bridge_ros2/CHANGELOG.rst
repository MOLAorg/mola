^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package mola_bridge_ros2
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^


2.4.0 (2025-12-28)
------------------
* Prepare for not using deprecated mrpt_maps types starting for mrpt >=3.0.0
* FIX: Potential segfault if observations come before ROS2 /tf broadcasters are initialized
* Contributors: Jose Luis Blanco-Claraco

2.3.0 (2025-12-15)
------------------
* Import all pcd fields from ros2 messages using CGenericPointsMap
* Contributors: Jose Luis Blanco-Claraco

2.2.1 (2025-11-08)
------------------
* BridgeROS2: more debug traces in map publishing
* Contributors: Jose Luis Blanco-Claraco

2.2.0 (2025-10-28)
------------------
* format
* Fix build against upcoming mrpt v2.15.0
* Contributors: Jose Luis Blanco-Claraco

2.1.0 (2025-10-20)
------------------
* Publish to ROS all map types implementing getAsSimplePointsMap()
* format
* FIX: ROS2 bridge must use timestamps for map updates to publish maps and deskewed clouds
* Support publishing several maps from the same source per iteration to ROS
* clang-format
* Make use of ConstPtr across API
* Contributors: Jose Luis Blanco-Claraco

2.0.0 (2025-10-13)
------------------
* fix clang-format
* Modernize copyright notice
* Contributors: Jose Luis Blanco-Claraco

1.9.1 (2025-07-07)
------------------

1.9.0 (2025-06-06)
------------------
* fix clang-format
* Implement publishing of optional "metadata" map field too
* Contributors: Jose Luis Blanco-Claraco

1.8.1 (2025-05-28)
------------------
* Fix: Do not use the deprecated ament_target_dependencies()
* Contributors: Jose Luis Blanco-Claraco

1.8.0 (2025-05-25)
------------------
* Update license tag to "BSD-3-Clause"
* Update copyright year
* Contributors: Jose Luis Blanco-Claraco

1.7.0 (2025-05-06)
------------------

1.6.4 (2025-04-23)
------------------
* fix: Correctly handling Livox cloud timestamps ("double"s, but in nanoseconds) in BridgeROS2 and bag2 data sources. They are automatically detected, no need to change any parameter.
* modernize clang-format
* Merge pull request `#82 <https://github.com/MOLAorg/mola/issues/82>`_ from ahpinder/develop
  Add Support for Voxel Map ROS2 Publishing Via Point Map Conversion
* fixed Clang formatting
* Clean up voxel map publishing code
* Added voxel map point cloud publishing
  Added code to timerPubMap to publish the occupied voxels of a mrpt::maps::CVoxelMap as a point cloud to ROS2, allowing for real-time ROS2 visualization of 2D map capture
* Contributors: Jose Luis Blanco-Claraco, ahpinder

1.6.3 (2025-03-15)
------------------
* clang-tidy: const correctness
* Service renamed: RelocalizeFromGNSS -> RelocalizeFromStateEstimator
* FIX: Potential deadlock in initialization
* Contributors: Jose Luis Blanco-Claraco

1.6.2 (2025-02-22)
------------------
* Implement publish Diagnostics per mola module & ROS2 publishers refactored (code clean up)
* BridgeROS2: add source filter for forwarding localization updates to ROS2
* ROS2: base_footprint_frame /tf is broadcasted now as base_link -> base_footprint to avoid /tf warnings (better as a child than as a second parent in the tf tree)
* FIX: In parsing base_footprint_to_base_link_tf
* Contributors: Jose Luis Blanco Claraco, Jose Luis Blanco-Claraco

1.6.1 (2025-02-13)
------------------
* Add new option: publish_tf_from_slam; add better docs on the meaning of all parameters
* Publish georef /tf as /tf_static
* ROS2 bridge now publishes georeferenced map metadata as /tf's and as mrpt_nav_interfaces/GeoreferencingMetadata
* Revert "Feature: all MOLA modules got its MRPT logger to ROS console for easier debugging"
  This reverts commit 8a84611d85022f37b80d8bdcb7acaa1910669fc1.
* FIX: wrong variable in former commit
* Merge pull request `#75 <https://github.com/MOLAorg/mola/issues/75>`_ from MOLAorg/feature/mrpt-to-ros-console
  Feature: all MOLA modules got its MRPT logger to ROS console for easier debugging
* Feature: all MOLA modules got its MRPT logger to ROS console for easier debugging
* Contributors: Jose Luis Blanco-Claraco

1.6.0 (2025-01-21)
------------------
* Publish gridmaps too
* ros2 bridge: rep105 only for map->base_link tfs
* BridgeROS2: support forwarding more than one localization message per timer call
* Fix published /tf's: those from LocalizationSources now can explicitly define their parent and child frames
* Contributors: Jose Luis Blanco-Claraco

1.5.1 (2024-12-29)
------------------

1.5.0 (2024-12-26)
------------------

1.4.1 (2024-12-20)
------------------
* BridgeROS2: add option (now enabled by default) to publish /tfs following REP105 order
* BUG FIX: Published odometry msg lacked target frame_id
* Rename method for better reflecting its goal
* Contributors: Jose Luis Blanco-Claraco

1.4.0 (2024-12-18)
------------------
* Publish localization quality topic
* Forward --ros-args to BridgeROS2
* expose services for runtime parameters
* Load relocalize_from_topic from yaml file
* ros2bridge: handle /initialpose topic -> relocalize service
* Contributors: Jose Luis Blanco-Claraco

1.3.0 (2024-12-11)
------------------
* Support publishing IMU readings MOLA -> ROS2
* Contributors: Jose Luis Blanco-Claraco

1.2.1 (2024-09-29)
------------------
* BUGFIX: Prevent potential race condition
* Contributors: Jose Luis Blanco-Claraco

1.2.0 (2024-09-16)
------------------
* sort <depend> entries
* Contributors: Jose Luis Blanco-Claraco

1.1.3 (2024-08-28)
------------------
* Depend on new mrpt_lib packages (deprecate mrpt2)
* Contributors: Jose Luis Blanco-Claraco

1.1.2 (2024-08-26)
------------------

1.1.1 (2024-08-23)
------------------

1.1.0 (2024-08-18)
------------------
* Merge pull request `#65 <https://github.com/MOLAorg/mola/issues/65>`_ from MOLAorg/add-more-srvs
  Add more Services
* Offer ROS2 services for the new MOLA MapServer interface
* clang-format: switch to 100 columns
* ros2bridge: offer ROS2 services for relocalization
* Merge pull request `#62 <https://github.com/MOLAorg/mola/issues/62>`_ from MOLAorg/docs-fixes
  Docs fixes
* Fix ament_xmllint warnings in package.xml
* change ament linters to apply in test builds
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
* BridgeROS2: more robust /tf find_transform by using tf2::BufferCore
* FIXBUG: inverse sensor poses in rosbag2 reader.
  Also: unify notation in C++ calls to lookupTransform()
* Fix package.xml website URL
* Contributors: Jose Luis Blanco-Claraco

1.0.2 (2024-04-04)
------------------
* update docs
* Contributors: Jose Luis Blanco-Claraco

1.0.1 (2024-03-28)
------------------
* BridgeROS2: do not quit on temporary /tf timeout
* mola_bridge_ros2: option to publish /tf_static for base_footprint
* mola_bridge_ros2: implement missing MOLA->ROS2 conversion for GNSS observations
* BUGFIX: Inverted value of "use_fixed_sensor_pose" was used
* Contributors: Jose Luis Blanco-Claraco

1.0.0 (2024-03-19)
------------------
* Comply with ROS2 REP-2003
* Merge ROS2 input and output in one module
* Contributors: Jose Luis Blanco-Claraco

0.2.2 (2023-09-08)
------------------