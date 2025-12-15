^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package mola_viz
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

2.3.0 (2025-12-15)
------------------
* Fix: missing uint fields stats in PCD preview GUI in mola_viz
* mola-viz: fix NaNs in range of each point channel preview window
* Viz: Fix handling multi-line messages in the UI terminal
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
* FIX: Show correct sensor rate in Hz when visualizing with a decimation
* Fix clang-tidy warnings
* Contributors: Jose Luis Blanco-Claraco

2.0.0 (2025-10-13)
------------------
* Merge pull request `#93 <https://github.com/MOLAorg/mola/issues/93>`_ from MOLAorg/feature/better-lio
  Changes for new LIO
* Fix warnings
* fix build against old mrpt versions
* Implement removal of decayed clouds
* MolaViz: Add method clear_all_point_clouds_with_decay()
* MolaViz: Add support for inserting clouds with decay_time
* fix clang-format
* Allow extra parameters in mola_viz per-sensor preview windows
* MolaViz: show min/max intensity in input sensor point clouds
* Remove old code that was needed to support very old MRPT versions
* Contributors: Jose Luis Blanco-Claraco

1.9.1 (2025-07-07)
------------------

1.9.0 (2025-06-06)
------------------

1.8.1 (2025-05-28)
------------------

1.8.0 (2025-05-25)
------------------
* Implement new virtual Viz methods
* Update copyright year
* Contributors: Jose Luis Blanco-Claraco

1.7.0 (2025-05-06)
------------------
* Metric maps can now be rendered as semitransparent pointclouds
* Contributors: Jose Luis Blanco-Claraco

1.6.4 (2025-04-23)
------------------
* modernize clang-format
* Contributors: Jose Luis Blanco-Claraco

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
* Drop dependency on mrpt-gui in kernel by abstracting MolaViz subwindow layout operations
* MolaViz: show package name in GUI windows
* Contributors: Jose Luis Blanco-Claraco

1.4.1 (2024-12-20)
------------------

1.4.0 (2024-12-18)
------------------

1.3.0 (2024-12-11)
------------------
* mola_viz: Show IMU data in the GUI too
* Contributors: Jose Luis Blanco-Claraco

1.2.1 (2024-09-29)
------------------

1.2.0 (2024-09-16)
------------------
* mola_viz: do not add a XY ground grid by default to all GUIs
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
* Update clang-format style; add reformat bash script
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
* Viz interface: add API for rotate camera
* Contributors: Jose Luis Blanco-Claraco

1.0.6 (2024-06-21)
------------------

1.0.5 (2024-05-28)
------------------
* viz: fix mismatched free/delete inside nanogui layout
* Contributors: Jose Luis Blanco-Claraco

1.0.4 (2024-05-14)
------------------
* bump cmake_minimum_required to 3.5
* MolaViz: BUGFIX: shared_ptr were captured by lambdas, delaying proper dtors. Replaced by weak_ptr's
* Contributors: Jose Luis Blanco-Claraco

1.0.3 (2024-04-22)
------------------
* Fix package.xml website URL
* Contributors: Jose Luis Blanco-Claraco

1.0.2 (2024-04-04)
------------------

1.0.1 (2024-03-28)
------------------

1.0.0 (2024-03-19)
------------------
* ROS2 launch demos
* use new mrpt GPS covariance field
* visualize sensor pose
* mola_kernel: new UI interface for datasets
* mola-viz: show image channel of RGBD observations
* Fix sensorPose on lidar preview
* Viz: show GPS data
* mola_viz: add custom icon
* viz: more options to visualize RGBD camera observations
* viz API: add enqueue_custom_nanogui_code()
* viz console: add fading effect
* mola_viz: show console messages
* Correct usage of mola:: namespace in cmake targets
* copyright update
* mola_viz: support visualizing velodyne observations
* Add look_at() viz interface
* Fewer mutex locking()
* reorganize as monorepo
* Contributors: Jose Luis Blanco-Claraco

0.2.2 (2023-09-08)
------------------
* Initial public release.
* Contributors: Jose Luis Blanco-Claraco


