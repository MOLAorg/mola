.. _supported_sensors:

======================
Sensors and datasets
======================

This page lists the ways to feed sensory data into a MOLA system, offline from
a recording or online from a live sensor.

Every option below implements the same ``RawDataSourceBase`` interface, so the
choice of input is independent of the choice of odometry, SLAM or localization
algorithm: the same pipeline runs unchanged on a rosbag, on a rawlog, or on a
live LiDAR.

.. contents::
   :depth: 1
   :local:
   :backlinks: none

|

1. Offline: from a recording
------------------------------

Generic formats
================

.. list-table::
   :widths: 22 33 45
   :header-rows: 1

   * - Format
     - Reader module
     - Run LiDAR odometry on it with
   * - **ROS 2 bag** (``.mcap``, sqlite3)
     - `mola_input_rosbag2 <https://github.com/MOLAorg/mola/tree/develop/mola_input_rosbag2>`_
     - ``mola-lo-gui-rosbag2 <bag>`` (GUI) or
       ``mola-lo-cli-rosbag2 <bag>`` (offline)
   * - **ROS 1 bag** (``.bag``)
     - `mola_input_rosbag1 <https://github.com/MOLAorg/mola_input_rosbag1>`_,
       which needs **no ROS 1 installation**
     - ``mola-lo-gui-rosbag1 <bag> [more bags]`` or
       ``mola-lo-cli-rosbag1 <bag> [more bags]``
   * - **MRPT rawlog** (``.rawlog``)
     - `mola_input_rawlog <https://github.com/MOLAorg/mola/tree/develop/mola_input_rawlog>`_
     - ``mola-lo-gui-rawlog <file.rawlog>`` or
       ``mola-lo-cli-rawlog <file.rawlog>``
   * - **KITTI-style ``.bin`` directories**
     - `mola_input_lidar_bin_dataset <https://github.com/MOLAorg/mola/tree/develop/mola_input_lidar_bin_dataset>`_
     - No odometry launcher yet; to **replay** one, use
       ``mola-cli $(ros2 pkg prefix mola_demos)/share/mola_demos/mola-cli-launchs/lidar_bin_directory_just_replay.yaml``

.. note::
   The ``mola-lo-*`` launchers are installed by ``mola_lidar_odometry``. Each is
   a thin wrapper that picks the right ``mola-cli`` launch file (GUI) or supplies
   the right flags to ``mola-lidar-odometry-cli`` (offline), so you do not have to
   pass ``-c <pipeline.yaml>`` and ``--state-estimator-param-file`` yourself.
   Run any of them with no arguments to see its usage, and see
   :ref:`Launching MOLA-LO <launching_mola_lo>` for the underlying commands.

.. note::
   Several bags can be replayed **jointly as one merged stream**, which is what
   datasets publishing one bag per topic require. See
   :ref:`the LO launchers <mola_lo_apps>`.

To convert between these formats, see :ref:`dataset conversions <dataset-conversions>`.

Public benchmark datasets
==========================

MOLA ships a **ready-to-run launcher per dataset**, so you do not have to work
out its topic names, frame ids, extrinsics or per-dataset tuning yourself. Each
comes in two flavors: ``mola-lo-gui-<name>`` for interactive replay with the
3D GUI, and ``mola-lo-cli-<name>`` for offline batch processing at full speed.

The table below is the overview. For one dataset in particular, see
:ref:`its own page <supported_datasets>`: where to put the data, the exact
commands, and every tuned default with the reasoning behind it.

.. list-table::
   :widths: 20 55 25
   :header-rows: 1

   * - Dataset
     - What it is
     - Launcher suffix
   * - `KITTI <https://www.cvlibs.net/datasets/kitti/>`_
     - The odometry benchmark; a car with a Velodyne HDL-64.
     - ``kitti``
   * - `KITTI-360 <https://www.cvlibs.net/datasets/kitti-360/>`_
     - Panoramic successor to KITTI.
     - ``kitti360``
   * - `MulRan <https://sites.google.com/view/mulran-pr/dataset>`_
     - A car with an Ouster OS1-64 around Daejeon and Sejong.
     - ``mulran``
   * - `Newer College <https://ori-drs.github.io/newer-college-dataset/>`_
     - Handheld rig walked around New College, Oxford; ships a survey-grade
       prior map.
     - ``newer-college``
   * - `Oxford Spires <https://dynamic.robots.ox.ac.uk/datasets/oxford-spires>`_
     - Hand-held / backpack VILENS payload around Oxford colleges.
     - ``oxford-spires``
   * - `Hilti-Oxford 2022 <https://hilti-challenge.com/>`_
     - Hand-held survey pole through construction sites and the Sheldonian
       Theatre.
     - ``hilti2022``
   * - `GrandTour <https://grand-tour.leggedrobotics.com>`_
     - ANYbotics ANYmal-D quadruped carrying the "Boxi" payload; one bag per
       topic.
     - ``grandtour``
   * - `BotanicGarden <https://github.com/robot-pesg/BotanicGarden>`_
     - Ground robot driven through a botanical garden.
     - ``botanicgarden``
   * - `CitrusFarm <https://ucr-robotics.github.io/Citrus-Farm-Dataset/>`_
     - Clearpath Jackal through citrus orchard rows.
     - ``citrusfarm``
   * - `ConSLAM <https://github.com/mac137/ConSLAM>`_
     - Hand-held scanner in construction sites (ROS 1 or ROS 2 bags).
     - ``conslam``
   * - `GEODE <https://github.com/PengYu-Team/GEODE_dataset>`_
     - Built around **degenerate geometry**: flat surfaces, stairwells, metro
       tunnels, off-road, inland waterways.
     - ``geode``
   * - `TIERS <https://github.com/TIERS/tiers-lidars-dataset>`_
     - Five LiDARs recording simultaneously, hence one launcher **per sensor**.
     - ``tiers-ouster-os0``, ``tiers-ouster-os1``,
       ``tiers-velodyne``, ``tiers-livox-horizon``, ``tiers-livox-avia``
   * - `Paris-LuCo <https://github.com/PRBonn/kiss-icp>`_
     - The ParisLuco128 sequence used by CT-ICP and others.
     - ``paris-luco``

So, for example:

.. code-block:: bash

    # Interactive replay with the GUI:
    mola-lo-gui-mulran KAIST01

    # The same sequence, offline and as fast as the machine allows:
    mola-lo-cli-mulran KAIST01 --output-tum-path mulran-kaist01.tum

Every launcher prints its own usage, including the dataset-specific
environment variables it honors, when run with no arguments.

.. seealso::
   :ref:`Launching MOLA-LO <launching_mola_lo>` for the full reference, and
   :ref:`mola_lo_pipelines` for the parameters these launchers set.

|

2. Online: from a live sensor
-------------------------------

.. list-table::
   :widths: 30 70
   :header-rows: 1

   * - Path
     - When to use it
   * - **ROS 2 topics**, via
       `mola_bridge_ros2 <https://github.com/MOLAorg/mola/tree/develop/mola_bridge_ros2>`_
     - The usual choice: your sensor already has a ROS 2 driver publishing
       ``sensor_msgs/PointCloud2``, ``sensor_msgs/Imu``, ``/tf``, …
       See the :ref:`ROS 2 node <mola_lo_ros>` and the
       :ref:`ROS 2 configurations cookbook <mola_ros2_cookbook>`.
   * - **Ouster native SDK**, via
       `mola_input_ouster <https://github.com/MOLAorg/mola_input_ouster>`_
     - Direct connection to an Ouster sensor (or replay of a PCAP/OSF
       recording) with no ROS middleware in the loop. Launcher:
       ``mola-lo-gui-ouster``.
   * - **mrpt-hwdrivers**
     - Direct connection to sensors supported by MRPT without ROS: 2D lidars
       (Hokuyo, SICK, RP-Lidar, Ibeo), Velodyne 3D lidars, cameras (OpenCV /
       ffmpeg / Bumblebee2), depth sensors (Kinect, OpenNI2), IMUs (xSens,
       KVH DSP3000), and NMEA or Novatel GNSS receivers.

|

3. What if my sensor or dataset is not listed?
------------------------------------------------

Nothing above is a special case in the odometry code. If your data is in a
ROS 1 or ROS 2 bag, or in a rawlog, MOLA can already read it: what the
per-dataset launchers add is only the knowledge of *which topic is which*.
Point the generic launcher at your recording and name the topics:

.. code-block:: bash

    MOLA_LIDAR_TOPIC=/ouster/points \
    MOLA_IMU_TOPIC=/ouster/imu \
    mola-lo-gui-rosbag2 /path/to/your/dataset.mcap

See :ref:`the rosbag2 launcher <mola_lo_gui_rosbag2>` for the full list of
options, including what to do when your bag has no ``/tf``.

.. index::
   single: supported sensors
