.. _dataset-conversions:

======================
Dataset conversions
======================

This page gives instructions on how to convert robotics datasets between
ROS 1 and ROS 2 bag files or live messages, MRPT's cross-platform RawLog format, and
the custom formats of popular robotics and computer vision datasets.

.. contents::
   :depth: 1
   :local:
   :backlinks: none


1. Dataset formats
---------------------------------
Robotics datasets consist of **sequences** of raw sensory data as captured by
a mobile robot, a vehicle, or just a hand-held device, as they move through the environment.

Each robotic framework or library has defined its own formats over time:

- **ROS 1** introduced `rosbags <https://wiki.ros.org/Bags>`_ (~2007) as a binary serialization
  storage for heterogeneous ROS messages.
- **ROS 2** (~2014) improved rosbags, which now can use different storage implementations (sqlite3, mcap),
  and are more efficient and flexible than older versions.
- `MRPT <https://github.com/MRPT/mrpt>`_, on which MOLA is internally based on, 
  defined its own serialized dataset format (~2005) named
  `RawLogs <https://docs.mrpt.org/reference/latest/class_mrpt_obs_CRawlog.html>`_, which is ensured
  to be portable across operative systems, backwards compatible across MRPT versions,
  and processor architecture independent.

.. note::
   Apart of these *standardized formats*, many robotic datasets have their own custom
   data layout (e.g. one binary file for each 3D LiDAR scan or image).
   MOLA makes easy reading them by providing *adapter modules* that can read the most
   popular datasets (see :ref:`list <supported_sensors>`) through unified C++ APIs
   (:ref:`doxid-classmola_1_1_offline_dataset_source`)
   or :ref:`re-publishing them to ROS <mola-dataset-to-ros>`.

|

.. _rosbag2rawlog:

2. rosbag ⇒ rawlog
----------------------------
To convert a ROS bag into a RawLog, you need two items:

- The program ``rosbag2rawlog``, and
- the **configuration file** specifying what ROS messages should be converted.

There are two versions of ``rosbag2rawlog``, for **ROS 1** and **ROS 2 bags**:

.. tab-set::

    .. tab-item:: ROS 2 bags
      :selected:

      ``rosbag2rawlog`` for **ROS 2 bags**. It is shipped with the ROS package `mrpt_rawlog <https://github.com/mrpt-ros-pkg/mrpt_navigation/tree/ros2/mrpt_rawlog>`_
      so it can be installed with:

      .. code-block:: bash

        sudo apt install ros-${ROS_DISTRO}-mrpt-rawlog

    .. tab-item:: ROS 1 bags

      ``rosbag2rawlog`` for **ROS 1 bags** belongs to upstream MRPT, and it is generated only *if*
      ROS 1 is detected at cmake configuration time. You can either build MRPT from sources after activating
      your ROS 1 environment, or install the Ubuntu package ``mrpt-apps``, which already ships ``rosbag2rawlog``
      for Ubuntu versions <=22.04 (In Ubuntu 24.04, ROS 1 packages were removed upstream).
      To ensure having the latest version, consider installing it from `the PPA <https://launchpad.net/~joseluisblancoc/+archive/ubuntu/mrpt-stable>`_.

.. note::
   As of 2025, there is an ongoing effort to move all ROS-related apps and bridge libraries into `its own repository <https://github.com/MRPT/mrpt_ros_bridge>`_,
   which will be the single repository for all MRPT-ROS stuff after the release of `MRPT 3.0.0 <https://github.com/MRPT/mrpt/issues/1330>`_.

|

Once you have ``rosbag2rawlog`` installed it can be invoked like:

.. dropdown:: rosbag2rawlog CLI arguments
  :open:

  .. code-block:: bash

      $ rosbag2rawlog --help

      USAGE: 

        rosbag2rawlog  [-b <base_link>] [-w] -c <config.yml> -o
                        <dataset_out.rawlog> [--] [--version] [-h] <log.bag> ...


      Where: 

        -b <base_link>,  --base-link <base_link>
          Reference /tf frame for the robot frame (Default: 'base_link')

        -w,  --overwrite
          Force overwrite target file without prompting.

        -c <config.yml>,  --config <config.yml>
          (required)  Config yaml file (*.yml)

        -o <dataset_out.rawlog>,  --output <dataset_out.rawlog>
          (required)  Output dataset (*.rawlog)

        --,  --ignore_rest
          Ignores the rest of the labeled arguments following this flag.

        --version
          Displays version information and exits.

        -h,  --help
          Displays usage information and exits.

        <log.bag>  (accepted multiple times)
          (required)  Input bag files (required) (*.bag)


For example:

.. code-block:: bash

    $ rosbag2rawlog -c config.yaml -o output.rawlog  input.mcap

With the ``config.yaml`` file created as explained below.


2.1. YAML config file format
==============================
``rosbag2rawlog`` needs an input configuration file to determine **what ROS messages** in the bag are to be **imported**.
The rest will be discarded, after emitting a warning to the terminal.

.. dropdown:: Example: basic Ouster LiDAR import
  :open:

  .. code-block:: yaml

    # Config file for rosbag2rawlog. It must contain a top-level "sensors" node
    sensors:
      # Then, one node per sensor to convert. This name will be used as
      # sensorLabel in MRPT observations.
      lidar:
        # Type: C++ class name (see mrpt::obs)
        type: 'CObservationPointCloud'
        topic: '/ouster/points'
        # If uncommented, /tf data will be ignored for this sensor pose:
        #fixed_sensor_pose: "0 0 0 0 0 0"  # 'x y z yaw_deg pitch_deg roll_deg''

.. dropdown:: Example: Newer College Dataset import

  .. code-block:: yaml

    # Config file for rosbag2rawlog. It must contain a top-level "sensors" node
    sensors:
      # Then, one node per sensor to convert. This name will be used as 
      # sensorLabel in MRPT observations.
      lidar:
        # Type: C++ class name (see mrpt::obs)
        type: 'CObservationPointCloud'
        # Parameters for this particular type of sensor.
        # Topic to subscribe for the pointcloud:
        topic: '/os_cloud_node/points'
        fixed_sensor_pose: "0 0 0 0 0 0"  # 'x y z yaw_deg pitch_deg roll_deg''

      cam0:
        type: 'CObservationImage'
        image_topic: '/alphasense_driver_ros/cam0/compressed'
        fixed_sensor_pose: "0 0 0 0 0 0"  # 'x y z yaw_deg pitch_deg roll_deg''


.. dropdown:: Example: Oxford Spires Dataset

  To import sequences from the `Oxford Spires dataset <https://dynamic.robots.ox.ac.uk/datasets/oxford-spires/>`_ to MRPT format:

  .. code-block:: yaml

    # Config file for rosbag2rawlog. It must contain a top-level "sensors" node
    sensors:
      # Then, one node per sensor to convert. This name will be used as
      # sensorLabel in MRPT observations.
      lidar:
        # Type: C++ class name (see mrpt::obs)
        type: "CObservationPointCloud"
        # Parameters for this particular type of sensor.
        # Topic to subscribe for the pointcloud:
        topic: "/hesai/pandar"
        fixed_sensor_pose: "0.0 0.0 0.124 180 0 0" # 'x y z yaw_deg pitch_deg roll_deg'

        # pCL_CI             : (x,y,z,yaw,pitch,roll)=(-0.0078,-0.0190,0.0705,90.63deg,-0.13deg,0.17deg)
        #T_base_imu_t_xyz_q_xyzw: [-0.018, 0.006, 0.058, 0.0, 0.0, 0.707, 0.707]   # qx qy qz qw
        #T_base_lidar_t_xyz_q_xyzw: [0.0, 0.0, 0.124, 0.0, 0.0, 1.0, 0.0]   # qx qy qz qw

      imu:
        type: "CObservationIMU"
        topic: "/alphasense_driver_ros/imu"
        fixed_sensor_pose: "-0.018 0.006 0.058 90 0 0" # 'x y z yaw_deg pitch_deg roll_deg'

    #  cam0:
    #    type: "CObservationImage"
    #    image_topic: "/alphasense_driver_ros/cam0/debayered/image/compressed"
    #    fixed_sensor_pose: "0 0 0 0 0 0" # 'x y z yaw_deg pitch_deg roll_deg'' # COMPUTE PROPERLY!


.. dropdown:: Example: LiDAR + wheels odometry from /odom

  .. code-block:: yaml

    # Config file for rosbag2rawlog. It must contain a top-level "sensors" node
    sensors:
      # Then, one node per sensor to convert. This name will be used as
      # sensorLabel in MRPT observations.
      lidar:
        # Type: C++ class name (see mrpt::obs)
        type: 'CObservationPointCloud'
        topic: '/ouster/points'
        # If uncommented, /tf data will be ignored for this sensor pose:
        #fixed_sensor_pose: "0 0 0 0 0 0"  # 'x y z yaw_deg pitch_deg roll_deg''
      odom:
        type: 'CObservationOdometry'
        topic: '/odom'

.. dropdown:: Example: LiDAR + wheels odometry from /tf

  .. code-block:: yaml
    
    # Config file for rosbag2rawlog. It must contain a top-level "sensors" node
    sensors:
      # Then, one node per sensor to convert. This name will be used as
      # sensorLabel in MRPT observations.
      lidar:
        # Type: C++ class name (see mrpt::obs)
        type: 'CObservationPointCloud'
        topic: '/ouster/points'
        # If uncommented, /tf data will be ignored for this sensor pose:
        #fixed_sensor_pose: "0 0 0 0 0 0"  # 'x y z yaw_deg pitch_deg roll_deg''

    # If provided, odometry observations will be generated from /tf messages
    # from `odom_frame_id` to `base_link` (frame_id can be changed via cli arguments):
    odom_from_tf:
        sensor_label: 'odom'  # mandatory entry
        #odom_frame_id: '/odom'


|


3. rawlog ⇒ ROS
----------------------------
For ROS 2. There is no ROS 1 republishing path: MOLA's bridge module targets
ROS 2 only, and ``mola_input_rosbag1`` reads ROS 1 bags rather than writing
them.

There is no dedicated ``rawlog2rosbag`` program either, because the general
mechanism covers this case: a rawlog is read by the ``mola_input_rawlog`` module, and any
MOLA dataset source can be republished as live ROS topics through the ROS 2
bridge. See :ref:`§5 below <mola-dataset-to-ros>`, using ``mola_input_rawlog``
as the dataset source module.

Record those topics with ``ros2 bag record`` if you want a bag rather than a
live replay.


|


.. _ros1_to_ros2:

4. ROS1 ⇒ ROS2
----------------------------
One way to use rosbags from ROS 1 with MOLA is to port them to ROS 2 bags.

You can use the Python package ``rosbags`` to `perform the conversion <https://ternaris.gitlab.io/rosbags/topics/convert.html>`_,
like in this example:

  .. code-block:: bash
    
    rosbags-convert --src my_ros1.bag --dst my_ros2.mcap --dst-storage mcap


An alternative is to use :ref:`rosbag2rawlog <rosbag2rawlog>` (the ROS 1 version!) to convert them to RawLogs, then use them as input to MOLA.

|

.. _mola-dataset-to-ros:

5. MOLA data set module ⇒ ROS
------------------------------

All you need is to put together a :ref:`MOLA launch YAML file <yaml_slam_cfg_file>` with:

- A dataset source module.
- The ros2bridge module.

and launch it using `mola-cli`. See existing ROS launch examples
under `ros2-launch (mola_demos package) <https://github.com/MOLAorg/mola/tree/develop/mola_demos/ros2-launchs>`_,
with corresponding MOLA cli launch files in the `mola-cli-launchs <https://github.com/MOLAorg/mola/tree/develop/mola_demos/mola-cli-launchs>`_ directory.

.. dropdown:: Example #1: play back a KITTI dataset sequence to ROS 2
  :open:

  .. code-block:: bash

    ros2 launch mola_demos ros-kitti-play.launch.py kitti_sequence:=01

  Result: the mola_viz GUI + RViz.

  .. figure:: https://mrpt.github.io/imgs/screenshot_mola_demo_kitti_replay_to_ros.jpg
    :width: 600

.. dropdown:: Example #2: play back a Mulran dataset sequence to ROS 2
  :open:

  Refer to the :ref:`complete tutorial <tutorial-mulran-replay-to-ros2>`.

|

.. _pointcloud-to-mm:

6. Point-cloud files ⇒ ``.mm`` metric maps
--------------------------------------------

In some cases, you may want to **import point-cloud files** in popular formats (e.g. PCD, LAS, etc.)
as a **layer within a metric map** in :ref:`mp2p_icp's metric map format <mp2p_icp_basics>` (``*.mm`` files).
Providing specialized converter programs for each format would require depending on many larger libraries
so, instead of that approach, ``mp2p_icp`` provides one single program (:ref:`app_txt2mm`) to **import
point-cloud files in text format** (e.g. ``.txt``, ``.csv``).

Therefore, the remaining step is to convert your point-cloud files to text format. Common cases are described below:

6.1. PCD files
===============

PCD files are serialized point-clouds as defined in the `PCL <https://pointclouds.org/>`_ library.

.. dropdown:: Example python script to convert PCD to text

  .. code-block:: python

    #!/bin/env python3
    # Save as 'pcd2txt.py'

    import open3d as o3d
    import numpy as np
    import pandas as pd

    import sys

    # Load PCD file
    pcd = o3d.io.read_point_cloud(sys.argv[1])

    # Convert to numpy array
    points = np.asarray(pcd.points)

    # If colors exist
    if pcd.has_colors():
        colors = np.asarray(pcd.colors)
        data = np.hstack((points, colors))
        header = "# x y z r g b"
    else:
        data = points
        header = "# x y z"

    # Save to TXT
    np.savetxt(sys.argv[1] + ".txt", data, fmt="%.6f", header=header, comments='')


  Then, you can convert a PCD file to text format with:

  .. code-block:: bash

      ./pcd2txt.py input.pcd


6.2. LAS files
===============

LAS files can be converted to text format using the ``las2txt`` (or ``las2txt64``) program,
which is part of the `LAStools <https://rapidlasso.com/lastools/>`_ library.

Once installed or built from sources, you can convert a LAS file to text format with:

.. code-block:: bash

    # If you only want XYZ coordinates (no color):
    las2txt64 -i input.las -o output.txt -parse xyz

    # For .las files with RGB color:
    las2txt64 -i input.las -o output.txt -parse xyzRGB -scale_RGB_to_8bit

