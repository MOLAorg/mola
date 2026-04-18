.. _mola_ros2api:

======================
ROS 2 API
======================
This page reflects the topics and services that a MOLA system will expose when running a SLAM 
or LiDAR-odometry module. At present, this applies to:

- Any MOLA system including the :ref:`BridgeROS2 <doxid-classmola_1_1_bridge_r_o_s2>` module:
  This module acts as a wrapper of ``mola-kernel`` virtual interfaces implemented in other
  MOLA modules and the ROS 2 system.
- :ref:`mola_lidar_odometry`
- :ref:`mola_sta_est_index`

.. image:: https://mrpt.github.io/imgs/mola-lo-ros2-launch-demo-kitti.png

____________________________________________

.. contents:: Table of Contents
   :depth: 2
   :local:
   :backlinks: none

____________________________________________

|

1. Nodes and launch files
--------------------------------------

.. _ros2_node_lo_docs:

1.1. ROS 2 node for LiDAR Odometry (LO) / LiDAR-inertial Odometry (LIO)
=========================================================================

.. include:: ../../../mola_lidar_odometry/docs/mola_lo_ros_node.rst

|

----

.. _map_loading_saving:

2. Map loading / saving
--------------------------------------
During a live SLAM run, ``BridgeROS2`` will look for modules implementing
:ref:`MapServer <doxid-classmola_1_1_map_server>` and will expose
these **ROS 2 services** to load or save the current map:

* ``/map_load``: See ROS docs for `mola_msgs/MapLoad <https://docs.ros.org/en/rolling/p/mola_msgs/srv/MapLoad.html>`_

* ``/map_save``: See ROS docs for `mola_msgs/MapSave <https://docs.ros.org/en/rolling/p/mola_msgs/srv/MapSave.html>`_

.. dropdown:: Example ROS 2 cli service calls

   To save the current map:

   .. code-block:: bash

      ros2 service call /map_save mola_msgs/srv/MapSave "map_path: '/tmp/my_map_file_prefix'"

   To load a map from disk:

   .. code-block:: bash

      ros2 service call /map_load mola_msgs/srv/MapLoad "map_path: '/tmp/my_map_file_prefix'"

Note that filename **extension** should not be given, since each service implementation
may add a different extension, or even save several files that should all, together, be
later on loaded as one to load the map again.

Alternatively, you can enable saving the map when mapping is ended by checking
the corresponding checkbox in the
:ref:`MOLA-LO GUI <mola_lo_gui_common_parts>` (block "6" below):

.. image:: imgs/gui_parts.png


|

----

.. _mola_ros2api_relocalization:

3. Re-localization
--------------------------------------
There are two ROS services that can be used to enforce the MOLA subsystem to relocalize, for example,
to address the problem of initial localization:


3.1. Specify the new localization and its initial uncertainty
=================================================================

The service ``/relocalize_near_pose`` (``mola_msgs/srv/RelocalizeNearPose``) can be
used to directly request a relocalization in a given area (a pose with uncertainty):

   .. code-block:: bash

      ros2 service call /relocalize_near_pose mola_msgs/srv/RelocalizeNearPose "{
      pose: {
         header: {
            stamp: {sec: 0, nanosec: 0},
            frame_id: 'map'
         },
         pose: {
            pose: {
            position: {x: 1.0, y: 2.0, z: 0.0},
            orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}
            },
            covariance: [1.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                        0.0, 1.0, 0.0, 0.0, 0.0, 0.0,
                        0.0, 0.0, 1.0, 0.0, 0.0, 0.0,
                        0.0, 0.0, 0.0, 1.0, 0.0, 0.0,
                        0.0, 0.0, 0.0, 0.0, 1.0, 0.0,
                        0.0, 0.0, 0.0, 0.0, 0.0, 1.0]
         }
      }
      }"


3.2. Request automatic relocalization from GNSS (GPS)
=================================================================

In geo-referenced maps, it is possible to request MOLA-LO to use incoming GPS readings to
bootstrap LiDAR-based localization. 

.. note::
   
   This method requires the use of the :ref:`smoother state estimator <mola_sta_est_index>`.

Request a relocalization now with:

   .. code-block:: bash

      ros2 service call /relocalize_from_state_estimator  mola_msgs/srv/RelocalizeFromStateEstimator "{}"


Depending on the parameters, it may take some time for the re-localization to take effect.


|

----

.. _mola_ros2_tf_frames:

4. Published ``/tf`` frames
--------------------------------------
The frames of reference (`frame_id`s) at work when using MOLA depend on
your system configuration:

- Using just ``mola_lidar_odometry``: Two situations here depending on the ROS :ref:`launch argument <ros2_node_lo_docs>` ``publish_localization_following_rep105``:

  - (A) Strictly following `ROS REP-105 <https://www.ros.org/reps/rep-0105.html>`_ in systems with wheels (encoders-based) high-frequency odometry, or
  - (B) Not following ``REP-105`` (e.g. if you do not have wheels odometry).

- (C) Using :ref:`state estimation data fusion <mola_sta_est_index>` (this case does **not** follow ``REP-105``), and

And orthogonal to both above, whether the map is :ref:`geo-referenced <geo-referencing>` or not.

The diagrams below show the cases of following or not following `ROS REP-105 <https://www.ros.org/reps/rep-0105.html>`_
for the different situations listed above:

.. tab-set::

   .. tab-item:: (A) LO+REP105
      :selected:

      For cases with ground robots with wheel-based odometry:

      .. figure:: https://mrpt.github.io/imgs/mola_mrpt_ros_geo_referenced_utm_frames.png
         :width: 500
         :align: center

      This is who is responsible of publishing each transformation:

      - ``odom → base_link``: Wheel odometry module. High-frequency, relatively accurate in the short term, but drifts in the long term.
      - ``map → odom``: :ref:`Localization <localization>` module, which corrects the odometry drift.
      - ``enu → {map, utm}``: Published by ``mrpt_map_server`` (`github <https://github.com/mrpt-ros-pkg/mrpt_navigation/tree/ros2/mrpt_map_server/>`_)
        or ``mola_lidar_odometry`` :ref:`map loading service <map_loading_saving>` if fed with a geo-referenced metric map (``.mm``) file.

      .. note::

         Internally, the MOLA localization module (``mola_lidar_odometry`` or a
         state estimator) always emits ``map → base_link`` updates.
         :ref:`BridgeROS2 <doxid-classmola_1_1_bridge_r_o_s2>` is what splits
         that into the REP-105 ``map → odom`` /tf broadcast by querying the
         current ``odom → base_link`` from /tf and composing it out.

   .. tab-item:: (B) LO, no REP105

      When using just a LiDAR as single sensor.

      .. figure:: https://mrpt.github.io/imgs/mola_mrpt_ros_frames_no_rep105.png
         :width: 500
         :align: center

      This is who is responsible of publishing each transformation:

      - ``map → base_link``: :ref:`Localization <localization>` module.
      - ``enu → {map, utm}``: Published by ``mrpt_map_server`` (`github <https://github.com/mrpt-ros-pkg/mrpt_navigation/tree/ros2/mrpt_map_server/>`_)
        or ``mola_lidar_odometry`` :ref:`map loading service <map_loading_saving>` if fed with a geo-referenced metric map (``.mm``) file.

   .. tab-item:: (C) Data fusion

         When using :ref:`state estimation data fusion <mola_sta_est_index>`: applicable if having just one LiDAR sensor,
         or LiDAR + wheel odometry, or several odometry sources, optionally GNNS (GPS) and IMU, etc.

      .. figure:: https://mrpt.github.io/imgs/mola_mrpt_ros_frames_fusion.png
         :width: 500
         :align: center

      This is who is responsible of publishing each transformation:

      - ``odom_{i} → base_link``: One or more odometry sources.
      - ``map → base_link``: Published by :ref:`state estimation data fusion <mola_sta_est_index>`.
      - ``enu → {map, utm}``: Published by ``mrpt_map_server`` (`github <https://github.com/mrpt-ros-pkg/mrpt_navigation/tree/ros2/mrpt_map_server/>`_)
        or ``mola_lidar_odometry`` :ref:`map loading service <map_loading_saving>` if fed with a geo-referenced metric map (``.mm``) file.

      .. note::

         External odometry sources must enter MOLA as ``nav_msgs/Odometry``
         topics (consumed as observations), **not** via /tf. If a wheel
         driver also broadcasts ``odom → base_link`` to /tf while the
         state estimator publishes ``map → base_link``, ``base_link``
         ends up with two parents and tf2 rejects the tree. See the
         :ref:`cookbook §4.4 <mola_ros2_cookbook_4_4>`.

.. note::

   For non geo-referenced maps, all frames remain the same but ``utm`` and ``enu`` will not exist.

Definition of the frames above:

- ``base_link``: The robot reference frame. For ground vehicles, normally placed at the
  center of the rear axle.
- ``base_footprint`` (optional): The projection of ``base_link`` on the ground plane. In MOLA, this frame is
  published by BridgeROS2 as a child of ``base_link``.
- ``odom``, ``odom_1``,... ``odom_n``: The arbitrary origin for odometry measurements.
  There may be different odometry sources: wheels, LiDAR odometry, visual odometry, etc.
- ``map``: The origin of the reference metric map used for localization.
- ``enu``: For geo-referenced maps, the North (``y`` axis), East (``x`` axis), Up (``z`` axis) frame for which
  we have reference geodetic coordinates (latitude and longitude). Different maps built in the same zone
  will surely have different ``enu`` frames, since it is defined by collected GNSS measurements.
- ``utm``: The origin of the `UTM zone <https://en.wikipedia.org/wiki/Universal_Transverse_Mercator_coordinate_system>`_
  in which ``enu`` falls. Unlike ``enu``, it is **independent** of the trajectory followed while building the map.


|

----

|

.. _ros2api_subscribed_topics:

5. Subscribed topics (``subscribe`` YAML block)
---------------------------------------------------

``BridgeROS2`` subscribes to ROS 2 sensor topics listed under the ``subscribe``
key in its YAML parameters.  Each entry in the sequence must contain at least
the following fields:

.. list-table::
   :header-rows: 1
   :widths: 25 15 60

   * - Field
     - Required
     - Description
   * - ``topic``
     - Yes
     - ROS 2 topic name (e.g. ``/imu``).  **If empty, the entry is silently
       skipped**, which allows defining optional subscription slots controlled
       via environment variables (e.g. ``topic: ${ODOM2_TOPIC|}``).
   * - ``msg_type``
     - Yes
     - One of: ``PointCloud2``, ``LaserScan``, ``Imu``, ``NavSatFix``,
       ``Odometry``, ``Image``.
   * - ``output_sensor_label``
     - Yes
     - Label assigned to the MOLA observation forwarded to front-ends.
   * - ``fixed_sensor_pose``
     - No
     - ``"x y z yaw pitch roll"`` - if present together with
       ``use_fixed_sensor_pose: true``, the sensor-on-vehicle pose is taken from
       this value instead of querying ``/tf``.
   * - ``use_fixed_sensor_pose``
     - No
     - Boolean (default ``false``).  When ``false``, the sensor pose is looked up
       via ``/tf`` at runtime.

Example:

.. code-block:: yaml

    subscribe:
      # Slot always active:
      - topic: /imu
        msg_type: Imu
        output_sensor_label: "imu"

      # Optional slot - disabled when ODOM2_TOPIC is unset:
      - topic: ${ODOM2_TOPIC|}
        msg_type: Odometry
        output_sensor_label: "${ODOM2_LABEL|odom2}"

|

----

|

.. _ros2api_topics:

6. Published topics
--------------------------------------
Write me!

|

----

|

7. Map publishing
--------------------------------------
There are three ways of publishing maps to ROS:

1. Using ``mrpt_map_server`` (`github <https://github.com/mrpt-ros-pkg/mrpt_navigation/tree/ros2/mrpt_map_server/>`_):
   the recommended way for static, previously-built maps, if you are **not** using MOLA for localization but want the 
   map published for localization using the :ref:`particle filter method <localization-only>` or for your own purposes,
   e.g. for visualization in RViz, processing in a custom node, etc.
   In this case, one ROS topic will be published for each map layer, as described in the package documentation.
   See also :ref:`this tutorial <tutorial-pub-map-server-to-ros>`.

2. During a live map building process (e.g. MOLA-LO).

.. dropdown:: Topics

   Using the default MOLA LiDAR odometry pipeline, only one map topic will be generated during live mapping:

   * Name: ``/lidar_odometry/localmap_points``
   * Type: ``sensor_msgs/PointCloud2``


3. If using MOLA-LO for localization-only, it will send out the loaded map.
   In this case, there will be as many topics as map layers in the ``*.mm`` file.
   See also :ref:`this tutorial <tutorial-pub-map-server-to-ros>`.

In cases (2)-(3), ``BridgeROS2`` will look for modules implementing
:ref:`MapSourceBase <doxid-classmola_1_1_map_source_base>` and will publish
one **topic** named ``<METHOD>/<LAYER_NAME>`` for each map layer.
The metric map layer C++ class will determine the ROS topic type to use.
Map topics are "latched" (so that new subscribers will receive the last
published map immediately after subscribing), and will be re-published only
if mapping is enabled and the map has changed since the last publication.

|

----

|

.. _ros2api_runtime_params:

8. Runtime dynamic reconfiguration
----------------------------------------
MOLA modules may expose a subset of their parameters through an interface that allows
runtime reconfiguration via ROS 2 service requests:

7.1. Runtime parameters for ``mola_lidar_odometry``
======================================================

List all existing parameters:

   .. code-block:: bash

      ros2 service call /mola_runtime_param_get mola_msgs/srv/MolaRuntimeParamGet

.. dropdown:: Example output
  :open:

   .. code-block:: bash

      requester: making request: mola_msgs.srv.MolaRuntimeParamGet_Request()

      response:
      mola_msgs.srv.MolaRuntimeParamGet_Response(parameters='mola::LidarOdometry:lidar_odom:\n  active: true\n  generate_simplemap: false\n  mapping_enabled: true\n')

   Returned ``parameters`` as YAML:

   .. code-block:: yaml

      mola::LidarOdometry:lidar_odom:
        active: true
        generate_simplemap: false
        mapping_enabled: true

Documented parameters:

- ``active``: Whether MOLA-LO should process incoming sensor data (``active: true``)
  or ignore them (``active: false``).

.. dropdown:: Copy & paste commands for ``active``

   .. code-block:: bash

      # active: true
      ros2 service call /mola_runtime_param_set mola_msgs/srv/MolaRuntimeParamSet \
         "{parameters: \"mola::LidarOdometry:lidar_odom:\n  active: true\n\"}"

   .. code-block:: bash

      # active: false
      ros2 service call /mola_runtime_param_set mola_msgs/srv/MolaRuntimeParamSet \
         "{parameters: \"mola::LidarOdometry:lidar_odom:\n  active: false\n\"}"


- ``mapping_enabled``: Whether MOLA-LO should update the localmap (``true``) or just use
  it in localization-only mode (``false``).

.. dropdown:: Copy & paste commands for ``mapping_enabled``

   .. code-block:: bash

      # mapping_enabled: true
      ros2 service call /mola_runtime_param_set mola_msgs/srv/MolaRuntimeParamSet \
         "{parameters: \"mola::LidarOdometry:lidar_odom:\n  mapping_enabled: true\n\"}"

   .. code-block:: bash

      # mapping_enabled: false
      ros2 service call /mola_runtime_param_set mola_msgs/srv/MolaRuntimeParamSet \
         "{parameters: \"mola::LidarOdometry:lidar_odom:\n  mapping_enabled: false\n\"}"


- ``generate_simplemap``: Whether MOLA-LO should build the keyframes-based map (apart of the local metric map),
  so you end up with a ``*.simplemap`` file.

.. dropdown:: Copy & paste commands for ``generate_simplemap``

   .. code-block:: bash

      # generate_simplemap: true
      ros2 service call /mola_runtime_param_set mola_msgs/srv/MolaRuntimeParamSet \
         "{parameters: \"mola::LidarOdometry:lidar_odom:\n  generate_simplemap: true\n\"}"

   .. code-block:: bash

      # generate_simplemap: false
      ros2 service call /mola_runtime_param_set mola_msgs/srv/MolaRuntimeParamSet \
         "{parameters: \"mola::LidarOdometry:lidar_odom:\n  generate_simplemap: false\n\"}"

- ``reset_state``: This is actually not a real state variable, but a trigger to request MOLA-LO to
  reset its state, effectively restarting mapping from scratch. It resets the internal local map, the
  simplemap (keyframe map). The state estimator, since it is in a different independent module, is not
  affected.

.. dropdown:: Copy & paste commands to reset map

   .. code-block:: bash

      ros2 service call /mola_runtime_param_set mola_msgs/srv/MolaRuntimeParamSet \
         "{parameters: \"mola::LidarOdometry:lidar_odom:\n  reset_state: true\n\"}"

----

.. _mola_ros2_initial_localization:

9. Initial localization
--------------------------------------

9.1. Lidar-Odometry (LO)
============================================
When the LO system is started, there are different situations: 

1. The system is started **without any former map**. Here, the default is starting at the identity SE(3) pose,
   that is, at the origin (0,0,0), and that should be enough in most common cases.

2. The system is started **with a former known map**. Here, correctly localizing within that map before trying to update
   it is critical to avoid ruining the map. Also, finding the correct initial pose is a non trivial problem and requires
   specific methods.

For the latter case, it is important to disable mapping at start up (see the ``start_mapping_enabled:=False`` launch 
argument :ref:`above <ros2_node_lo_docs>`) and only enable mapping once the system is correctly localized, and if
the user really wants to update the map. Keeping mapping disabled for the whole run is actually desired for robots
operating in a known, pre-mapped environment.

Then, the user can choose between: 

- Requesting re-localization in a given area or from GNSS readings, as described in
  :ref:`this section <mola_ros2api_relocalization>` above.
- Selecting one of the available initial localization methods directly set in the pipeline configuration file,
  or via a ROS2 launch argument, so that method is used straight away at startup.

These are the available initial localization methods, that can be used in the launch argument 
``initial_localization_method:=xxxx`` launch argument (listed :ref:`above <ros2_node_lo_docs>`):

.. dropdown:: How to select initial localization without ROS API

   If LO is launched independently of a ROS2 system, e.g. using the 
   :ref:`command-line <mola_lidar_odometry_cli>` or :ref:`GUI <mola_lo_apps>` LO tools,
   the initial localization method can be set via the environment variable
   ``MOLA_LO_INITIAL_LOCALIZATION_METHOD`` which should be set to any of the options
   listed below. For example, to set the initial localization method to ``FromStateEstimator``:

   .. code-block:: bash

      MOLA_LO_INITIAL_LOCALIZATION_METHOD="InitLocalization::FromStateEstimator" \
      mola_lidar_odometry_cli ... \ # the rest as usual


- ``InitLocalization::FixedPose``: Initializes around a given SE(3) pose with covariance.

- ``InitLocalization::FromStateEstimator``: In combination with the smoother state estimator,
  can be used to initialize based on accumulated evidence of geo-referenced positioning based on low-cost 
  GNSS readings, wheels odometry, IMU, or any sensible combination of sensors. See :ref:`smoother state estimator <mola_sta_est_index>`.

- ``InitLocalization::PitchAndRollFromIMU``: Without using the external state estimator, this method
  uses the IMU to estimate the pitch and roll angles of the robot, and then initializes the localization
  system with that information **assuming sensor is roughly stationary at startup**.
  This is useful for systems that are not perfectly level, such as hand-held devices, drones, etc.
  since it will remove the apparent tilt of the ground plane.
