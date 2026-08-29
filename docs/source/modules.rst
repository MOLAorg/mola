.. _modules:

===================
MOLA packages
===================

MOLA's modular design means a SLAM or localization problem is split into the
smallest possible reusable pieces, each shipped as one **colcon package**.
The ones that implement a ``mola_kernel`` interface (data sources, front ends,
state estimators, map servers, visualizers) are *modules*: they are named in a
:ref:`YAML configuration file <yaml_slam_cfg_file>` and loaded at runtime,
without recompiling anything. The rest are the supporting cast, linked as
ordinary libraries or used as build-time dependencies: message definitions,
CLI tools and metapackages.

This page is the **inventory**: what each package is for, and where its sources
live. For the tasks these packages are combined to solve, see
:ref:`solutions <solutions>`; for the C++ classes inside them, see the
:doc:`API reference <doxygen-index>`.

.. note::
   Each package carries its own license (GPL-3.0 for core algorithms,
   BSD-3-Clause for utilities, bridges and demos). Check the ``<license>``
   tag of its ``package.xml``.

.. contents::
   :depth: 1
   :local:
   :backlinks: none


Core framework
----------------

Everything else builds on these. They live in the
`MOLAorg/mola <https://github.com/MOLAorg/mola>`_ repository.

.. list-table::
   :widths: 30 70
   :header-rows: 1

   * - Package
     - Purpose
   * - `mola_kernel <https://github.com/MOLAorg/mola/tree/develop/mola_kernel>`_
     - The fundamental C++ virtual interfaces and data types every other
       module derives from (``RawDataSourceBase``, ``FrontEndBase``,
       ``NavStateFilter``, ``MapServer``, ``VizInterface``, …).
   * - `mola_launcher <https://github.com/MOLAorg/mola/tree/develop/mola_launcher>`_
     - Loads modules from a YAML configuration and runs their life cycle.
       Provides :ref:`mola-cli <mola_cli>`, the main entry point.
   * - `mola_yaml <https://github.com/MOLAorg/mola/tree/develop/mola_yaml>`_
     - YAML parsing with MOLA's :ref:`extensions <yaml_extensions>`:
       variable expansion, includes, imports and formulas.
   * - `mola_msgs <https://github.com/MOLAorg/mola/tree/develop/mola_msgs>`_
     - ROS 2 messages, services and actions used by the other packages.


Input: datasets and sensors
-----------------------------

Except where noted, each of these implements ``RawDataSourceBase``, so any of
them can feed any front end. See :ref:`sensors and datasets <supported_sensors>`
for how to choose one.

.. list-table::
   :widths: 30 70
   :header-rows: 1

   * - Package
     - Purpose
   * - `mola_input_rosbag2 <https://github.com/MOLAorg/mola/tree/develop/mola_input_rosbag2>`_
     - ROS 2 bag files (``.mcap``, sqlite3), including ``/tf`` resolution.
   * - `mola_input_rosbag1 <https://github.com/MOLAorg/mola_input_rosbag1>`_
     - ROS 1 ``.bag`` files, with **no ROS 1 installation required**.
   * - `mola_input_rawlog <https://github.com/MOLAorg/mola/tree/develop/mola_input_rawlog>`_
     - MRPT ``.rawlog`` datasets.
   * - `mola_input_lidar_bin_dataset <https://github.com/MOLAorg/mola/tree/develop/mola_input_lidar_bin_dataset>`_
     - Directories of ``.bin`` LiDAR scans in the KITTI binary layout.
   * - `mola_input_video <https://github.com/MOLAorg/mola/tree/develop/mola_input_video>`_
     - Live or offline video sources, via OpenCV.
   * - `mola_input_ouster <https://github.com/MOLAorg/mola_input_ouster>`_
     - Ouster LiDARs through the native Ouster SDK: direct sensor connection
       and PCAP replay, with no ROS middleware in between.
   * - `mola_academic_datasets <https://github.com/MOLAorg/mola_academic_datasets>`_
     - *Metapackage*, not a module itself; it bundles the readers for public
       benchmark datasets, each of which is one:
       ``mola_input_kitti_dataset``, ``mola_input_kitti360_dataset``,
       ``mola_input_mulran_dataset``, ``mola_input_euroc_dataset``,
       ``mola_input_paris_luco_dataset``, plus ``kitti_metrics_eval``.
   * - `mola_bridge_ros2 <https://github.com/MOLAorg/mola/tree/develop/mola_bridge_ros2>`_
     - Bidirectional ROS 2 ↔ MOLA bridge: subscribes to live topics as a data
       source, and publishes MOLA's outputs back as topics and ``/tf``.


Odometry, SLAM and state estimation
-------------------------------------

.. list-table::
   :widths: 30 70
   :header-rows: 1

   * - Package
     - Purpose
   * - `mola_lidar_odometry <https://github.com/MOLAorg/mola_lidar_odometry>`_
     - :ref:`3D/2D LiDAR and LiDAR-inertial odometry <mola_lidar_odometry>`,
       the main MOLA front end.
   * - `mola_mapper <https://github.com/MOLAorg/mola_mapper>`_
     - :ref:`Central 3D SLAM map <mola_mapper>`: online fusion of
       LIO/VIO/IMU/GNSS/wheels with loop closure and geo-referencing.
   * - `mola_state_estimation <https://github.com/MOLAorg/mola_state_estimation>`_
     - :ref:`State estimators <mola_sta_est_index>` (``Simple`` and
       ``Smoother``), plus ``mola_georeferencing`` and ``mola_gtsam_factors``.
   * - `mola_imu_preintegration <https://github.com/MOLAorg/mola_imu_preintegration>`_
     - IMU preintegration and gravity estimation used by the LIO paths.
   * - `mola_sm_loop_closure <https://github.com/MOLAorg/mola_sm_loop_closure>`_
     - Offline loop-closure post-processing of a simple-map, as a library
       and a CLI tool.
   * - `mola_relocalization <https://github.com/MOLAorg/mola/tree/develop/mola_relocalization>`_
     - Global localization and pose estimation under large initial
       uncertainty.


Maps, tools and visualization
-------------------------------

.. list-table::
   :widths: 30 70
   :header-rows: 1

   * - Package
     - Purpose
   * - `mola_metric_maps <https://github.com/MOLAorg/mola/tree/develop/mola_metric_maps>`_
     - The map classes: ``KeyframePointCloudMap``, ``IncrementalPointCloud``,
       ``NDTMap``, ``SparseVoxelPointCloud``, super-resolution occupancy
       grids, and the ``.mm`` CLI tools around them.
   * - `mp2p_icp <https://github.com/MOLAorg/mp2p_icp>`_
     - :ref:`ICP pipelines, point-cloud filters and map tools <mp2p_icp_basics>`.
   * - `mola_pose_list <https://github.com/MOLAorg/mola/tree/develop/mola_pose_list>`_
     - Spatially-searchable pose lists (keyframe density bookkeeping).
   * - `mola_traj_tools <https://github.com/MOLAorg/mola/tree/develop/mola_traj_tools>`_
     - CLI tools to manipulate trajectory files, complementing ``evo``.
   * - `mola_gnss_to_markers <https://github.com/MOLAorg/mola_gnss_to_markers>`_
     - Publishes GNSS datums as RViz/FoxGlove ellipsoid markers on a
       :ref:`geo-referenced map <geo-referencing>`.
   * - `mola_viz <https://github.com/MOLAorg/mola/tree/develop/mola_viz>`_ /
       `mola_viz_imgui <https://github.com/MOLAorg/mola/tree/develop/mola_viz_imgui>`_
     - The two interchangeable GUI backends (nanogui and Dear ImGui). See
       :ref:`choosing a visualizer <mola_lo_gui_visualizer_selection>`.
   * - `mola_demos <https://github.com/MOLAorg/mola/tree/develop/mola_demos>`_
     - Example ``mola-cli`` launch files and ROS 2 launch files.


Reference pages
-----------------

.. toctree::
  :maxdepth: 2

  module-mola-launcher
  module-mola-yaml
