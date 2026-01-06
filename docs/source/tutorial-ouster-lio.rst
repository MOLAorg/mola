.. _tutorial-ouster-lio:

===================================================
MOLA-LIO: Hand-held LiDAR mapping and localization
===================================================

This hands-on tutorial will show you how to **build a 3D map live using 
MOLA-LIO** from a hand-held or robot-mounted LiDAR with an internal IMU (Ouster, Livox,...),
how to **post-process** the saved map, 
and how to load that map in **LIO localization-only mode**.

.. contents::
   :depth: 1
   :local:
   :backlinks: none


The tutorial is fully available in this YouTube video, but you also have all 
the commands and steps described in text below for your convenience:

.. raw:: html

    <div style="margin-top:10px;">
      <iframe width="560" height="315" src="https://www.youtube.com/embed/z7dwRCbcvs8" title="YouTube video player" frameborder="0" allow="accelerometer; autoplay; clipboard-write; encrypted-media; gyroscope; picture-in-picture; web-share" referrerpolicy="strict-origin-when-cross-origin" allowfullscreen></iframe>
    </div>

|

Prerequisites: MOLA installation
----------------------------------
This tutorial assumes basic knowledge of ROS 2, networking, and Linux.

You must first install the packages ``mola_lidar_odometry`` and ``mola``.
Following the default :ref:`installation instructions <installing>` is enough.

|

1. Launching the LiDAR ROS driver
----------------------------------
In the following we will assume an Ouster sensor, so change these commands accordingly for other manufacturers.
  
First, clone the ROS 2 "driver" package for your LiDAR into your ROS 2 workspace and build it:

.. code-block:: bash

    # Clone:
    cd ~/ros2_ws/src
    git clone https://github.com/ouster-lidar/ouster-ros.git --recursive
    
    # Build it:
    cd ~/ros2_ws
    colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo

    # And activate the new packages:
    . install/setup.bash

Before going on, we need to power up the LiDAR and connect its Ethernet network cable to the computer, using a link-only Network configuration, 
and find the LiDAR network address (IP or local address).
See the video above for step by step details.

Next, launch the "driver" to start capturing LiDAR scans. The easiest way is using the launch file ``ouster_ros/sensor.launch.xml``
which shows the live data visually in RViz (it may take a few seconds to start). A starting point command would be (customize as needed):

.. code-block:: bash

    # Show all available options:
    ros2 launch ouster_ros sensor.launch.xml --show-args

    # Launch with all defaults, using computer-stamped timestamps:
    ros2 launch ouster_ros sensor.launch.xml sensor_hostname:=os-XXXXXXXXXXXX.local \
      timestamp_mode:=TIME_FROM_ROS_TIME


2. Record your dataset
----------------------------------
Now, there are two options:
* Either we build the map using the live sensor, or
* We record the dataset in a rosbag, then build it offline.

I would recommend grabing the data for two reasons. First, in this way, you can save several bags, browse them (using FoxGlove) 
and pick the one that looks best. Second, the computer load would be reduced during grabing if not running mapping live,
reducing the probability of missing some LiDAR network packets.

.. code-block:: bash

    # Show all available options:
    ros2 bag record  /ouster/imu /ouster/points /tf_static

Press CTRL+C to end the recording.

.. note::
    If you want to run live mapping instead, please check the LIO ROS 2 node documentation.


3. Build the map
----------------------------------
Just invoke this command to launch the LIO process on your Ouster dataset
(check the video for explanations on the different arguments):

.. code-block:: bash

    MOLA_GENERATE_SIMPLEMAP=true \
    MOLA_SIMPLEMAP_OUTPUT="myMap.simplemap" \
    MOLA_SIMPLEMAP_MIN_XYZ=0.2 \
    MOLA_LO_INITIAL_LOCALIZATION_METHOD="InitLocalization::PitchAndRollFromIMU" \
    MOLA_DESKEW_METHOD=MotionCompensationMethod::IMU \
    MOLA_IMU_TOPIC="/ouster/imu" \
    MOLA_LIDAR_TOPIC="/ouster/points" \
    MOLA_TF_BASE_LINK="os_sensor" \
    mola-lo-gui-rosbag2 \
      /path/to/your/dataset.mcap

As a result, we have a reconstructed keyframe-based map in ``.simplemap`` format.
We can apply any pipeline we want to build a metric map (an actual aggregated point cloud, or a voxel map, etc.).
For example, let's run:

.. code-block:: bash

    sm2mm -i myMap.simplemap -o myMap.mm -p sm2mm_no_decim_imu_mls_keyframe_map.yaml


Inspect the resulting map with:

.. code-block:: bash

    mm-viewer myMap.mm  -l libmola_metric_maps.so


4. Use in localization-only mode
----------------------------------

In one terminal, launch MOLA-LIO ROS 2 node in localization mode:

.. code-block:: bash

  ros2 launch mola_lidar_odometry ros2-lidar-odometry.launch.py \
    start_active:=False \
    publish_localization_following_rep105:=False \
    start_mapping_enabled:=False \
    lidar_topic_name:="/ouster/points" \
    imu_topic_name:="/ouster/imu" \
    mola_tf_base_link:="os_sensor" \
    mola_deskew_method:="MotionCompensationMethod::IMU" \
    mola_initial_map_mm_file:=$(pwd)/myMap.mm

Then, either launch again the live LiDAR or ``ros2 bag play`` another dataset you want to use for localization.

Finally, two key actions are required: we need to tell MOLA-LIO an initial localization guess 
(e.g. using RViz InitialPose button) and, only after that,
**enable** it so incoming LiDAR scans are actually processed and matched against the prebuilt map:


   .. code-block:: bash

      # active: true
      ros2 service call /mola_runtime_param_set mola_msgs/srv/MolaRuntimeParamSet \
         "{parameters: \"mola::LidarOdometry:lidar_odom:\n  active: true\n\"}"

Check the complete service call documentation :ref:`here <ros2api_runtime_params>`.


.. note::
    Other initialization options, such as GNSS-based, are possible via the smoother state estimation.
    Documentation and examples to come soon!


|

