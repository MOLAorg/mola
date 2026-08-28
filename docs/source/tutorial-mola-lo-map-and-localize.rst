.. _tutorial-mola-lo-map-and-localize:

===============================================
MOLA-LO: Build a map and then localize
===============================================

This tutorial will show you how to build a map using MOLA-LO, then save the map to disk, 
and how to load that map to use the LO localization mode.

.. contents::
   :depth: 1
   :local:
   :backlinks: none

|

This video shows the steps in the tutorial:

.. raw:: html

    <div style="margin-top:10px;">
      <iframe width="560" height="315" src="https://www.youtube.com/embed/CXiU_2vYMQE" title="YouTube video player" frameborder="0" allow="accelerometer; autoplay; clipboard-write; encrypted-media; gyroscope; picture-in-picture; web-share" referrerpolicy="strict-origin-when-cross-origin" allowfullscreen></iframe>
    </div>

|

Prerequisites: MOLA installation
----------------------------------
This tutorial requires the installation of these packages: ``mola_lidar_odometry``, ``mola_viz``, ``mvsim``.
The MVSim simulator is used as an example, but a live robot or LiDAR sensor can be used instead.

Following the default :ref:`installation instructions <installing>` is enough.


|

1. Create a map and save it
----------------------------------

Open **three terminals**, and run these commands in each one:

.. tab-set::

    .. tab-item:: #1: Simulator
      :selected:

      Launch the simulator or, if you are on a real robot, make sure to have the system up
      and LiDAR data coming out:

      .. code-block:: bash

          ros2 launch mvsim demo_warehouse.launch.py \
            do_fake_localization:=False \
            use_rviz:=False

      **Note:** If you choose to enable RViz at this point instead (``use_rviz:=True``), it
      will start emitting warnings about "Message filter dropping message", and LiDAR
      will not be visible in its GUI. It's OK. That is because we still did not launch MOLA-LO
      and `tf` is missing the transform `map -> odom`.


    .. tab-item:: #2: MOLA-LO

        In terminal #2, launch MOLA-LO, enabling saving the map in simple-map format:

        .. code-block:: bash

            ros2 launch mola_lidar_odometry ros2-lidar-odometry.launch.py \
              lidar_topic_name:=/lidar1_points \
              generate_simplemap:=True

        .. note::

          Remember replacing ``/lidar1_points`` with your actual PointCloud2 topic with raw LiDAR data.

        Next, **move the robot around**.
        In the simulator, you can click on the MVSim GUI and use keys ``a/d``+ ``s/w`` to drive around,
        or `use a gamepad to teleoperate it <https://mvsimulator.readthedocs.io/en/latest/teleoperation.html>`_.

    .. tab-item:: #3: Save the map

        Once the map looks OK in the mola_viz GUI, let's save it.
        In terminal #3, run:

        .. code-block:: bash

            ros2 service call /map_save mola_msgs/srv/MapSave "map_path: '/tmp/my_map'"

        Watch the response to check that ``success`` is ``true``. Your map is now
        stored as file(s) named ``/tmp/my_map*``.

|

2. Optional: Build down-sampled, globally consistent map
----------------------------------------------------------

The map saved in the former step comprises two files:

- A **key-frame (view-based) map** (``*.simplemap``): a set of SE(3) poses annotated with metadata and raw sensor observations.
- The metric **local map** used by the specific LO/LIO pipeline; For the for the ``lidar3d-gicp`` :ref:`pipeline <mola_3d_gicp_pipeline>` (default),
  it is a key-frame based point cloud based on :ref:`mola_metric_maps/KeyframePointCloudMap <doxid-classmola_1_1_keyframe_point_cloud_map>`.
  For the ``lidar3d-icp`` :ref:`pipeline <mola_3d_icp_pipeline>`,  it is a voxel-based point cloud of the current area, which may include an area of
  more than 100 meters around the latest robot pose, but if you mapped a larger area, it would be an incomplete representation of the whole map.


The key-frame map is the more versatile one since it allows for running further map filtering,
loop closure, downsampling, etc. while the metric local map is just what LO needs for **local** alignment of
incoming scans.

If your map is "small" in comparison to the sensor range (e.g. a few hundreds of meters length for 3D LiDARs outdoors),
the local map would contain the whole scenario and you are good to go with it as is, so you can skip to next section.

.. dropdown:: Visually inspect the generated ``*.mm`` file
  :icon: code-review

  You can check how the final metric map looks like using:

  .. code-block:: bash

    mm-viewer -l libmola_metric_maps.so  /path/to/your/map.mm

  Where the ``-l`` flag is used to load the additional metric map classes defined in ``mola_metric_maps``, and
  used in the ``lidar3d-default`` :ref:`pipeline <mola_lo_pipelines>`.


However, if the map is much larger, you need to **generate a new local metric map** that includes the whole area where
the robot needs to operate so it can localize correctly.

.. dropdown:: How to generate a new local metric map
  :icon: checklist

  Discard the local metric map file (``*.mm``) and let's start processing the key-frame map (``*.simplemap``).

  First, if your map is so large that it needs to close loops (e.g. several building blocks, a large part of a campus, etc.)
  the :ref:`loop closure module <solutions>` would be needed to post-process the ``*.simplemap`` file.

  Next, either if you already have closed loops or you do not need them, follow :ref:`these instructions <building-maps_step_mm>`,
  taking into account that there must exist an output metric map **layer** named as expected by the LO pipeline (e.g. :ref:`lidar3d-default <mola_lo_pipelines>`).

  TO-DO: Write complete example files and commands!



|


3. Load a prebuilt map in localization-only mode
--------------------------------------------------

..  note::

  Make sure of closing the former instance of MOLA-LO in terminal #2 used to build the map
  before going on with this part.

Again, we will use **three terminals**:

.. tab-set::

    .. tab-item:: #1: Simulator

      You can keep the former instance of the MVSim simulator running from the
      former step, or launch it again and move to a different pose, it is up to you!

    .. tab-item:: #2: MOLA-LO in localization mode
      :selected:

      Let's launch MOLA-LO in **non-mapping (localization only) mode** with:

      .. code-block:: bash

          ros2 launch mola_lidar_odometry ros2-lidar-odometry.launch.py \
            start_active:=False \
            start_mapping_enabled:=False \
            lidar_topic_name:=/lidar1_points

      .. note::

        Remember replacing ``/lidar1_points`` with your actual PointCloud2 topic with raw LiDAR data.

      Explanation:

      - ``start_mapping_enabled:=False`` disables map updates, so the loaded map will remain static.
      - ``start_active:=False`` is recommended so LO does not attempt to match incoming sensor
        data until a relocalization method or rough initial localization is set (see next section below).

    .. tab-item:: #3: Load the map

        Next, in terminal #3, let's order MOLA-LO to **load our former map** from disk:

        .. code-block:: bash

            ros2 service call /map_load mola_msgs/srv/MapLoad "map_path: '/tmp/my_map'"

        If you want to load a post-processed metric map (just the ``*.mm`` file), use the full
        path to the map file, **without the ``*.mm`` extension**.
        Read more on how to generate metric maps
        from key-frame maps (``*.simplemap``) :ref:`here <building-maps_step_mm>`.

        Note that it is also possible to directly launch MOLA-LO with a map loaded from disk
        from the beginning, but it implies passing one or both maps (``*.mm`` and ``*.simplemap``)
        by command line as environment variables.

|

.. dropdown:: Directly loading the map from MOLA-LO start up

    Instead of first invoking MOLA-LO and then requesting to load the map via a ROS 2 service,
    it is possible to instruct MOLA-LO to start loading the map straightaway as it starts up
    by specifying the path to **both** map files, instead of the **map prefix** used
    in the ROS 2 service:

    .. code-block:: bash

        ros2 launch mola_lidar_odometry ros2-lidar-odometry.launch.py \
          start_active:=False \
          start_mapping_enabled:=False \
          lidar_topic_name:=/lidar1_points \
          mola_initial_map_mm_file:=/tmp/my_map.mm \
          mola_initial_map_sm_file:=/tmp/my_map.simplemap

    Of course, the ROS 2 service offers a greater flexibility to switch
    between maps at run-time. You can drop ``MOLA_LOAD_SM`` if you do not need
    to extend the map (multi-session mapping).


|


4. Invoke relocalization
---------------------------------------------

As explained :ref:`here <localization-only_common>`, initial localization is a hard problem on its own
and can be handled in different ways.

Here we will show the common situation of wanting to re-localize the robot in a prebuilt map,
given we already know **a rough estimation of its actual pose**, including the orientation.
Check out :ref:`all the details <mola_ros2api_relocalization>` about requesting
relocalization via ROS 2 API.


.. tab-set::

    .. tab-item:: Re-localize with a topic
      :selected:

      Just use the RViz2's button ``2D pose estimate`` or FoxGlove's "Pose Estimate"
      to pick a pose and MOLA-LO will try to re-localize the vehicle in the given area.

      .. image:: https://mrpt.github.io/imgs/mola-lo-relocalization-from-fox-glove.jpg

    .. tab-item:: Re-localize with a service

      There is also a ROS 2 service for programmatically request a relocalization, and
      obtaining feedback about whether the request was received or not by a running MOLA module:

      - Service default name: ``/relocalize_near_pose``
      - Service interface: `mola_msgs::srv::RelocalizeNearPose <https://docs.ros.org/en/rolling/p/mola_msgs/srv/RelocalizeNearPose.html>`_

|

Once MOLA-LO knows how to handle initialization, we can activate it, either from the GUI (click the "active" checkbox)
or via this command (see all other similar commands):

   .. code-block:: bash

      # active: true
      ros2 service call /mola_runtime_param_set mola_msgs/srv/MolaRuntimeParamSet \
         "{parameters: \"mola::LidarOdometry:lidar_odom:\n  active: true\n\"}"

Then, the module will start to produce localization estimates via ``/tf``, ``Odometry`` messages,
together with a localization quality metric (see all :ref:`published topics <ros2api_topics>`).

|

