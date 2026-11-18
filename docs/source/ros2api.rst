.. _mola_ros2api:

======================
ROS 2 interface
======================
This page documents the ROS 2 interface of a MOLA system: the topics it subscribes to and
publishes, its ``/tf`` frames and its services. It is implemented by the
:ref:`BridgeROS2 <doxid-classmola_1_1_bridge_r_o_s2>` module, which connects the
``mola-kernel`` interfaces of the other modules (LiDAR odometry, state estimators, map
servers, ...) with ROS 2.

- The launch file of the LiDAR odometry node and all its arguments: :ref:`ros2_node_lo_docs`.
- Ready-made launch commands for each use case: :ref:`mola_ros2_cookbook`.

.. image:: https://mrpt.github.io/imgs/mola-lo-ros2-launch-demo-kitti.png

.. contents::
   :depth: 1
   :local:
   :backlinks: none

|

.. _ros2api_subscribed_topics:

Subscribed topics (``subscribe`` YAML block)
--------------------------------------------

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
       ``Odometry``, ``Image``. ``Image`` takes raw ``sensor_msgs/Image``, decoded in
       the bridge callbacks: prefer low-rate camera topics and a small ``qos.depth``.
       8-bit Bayer encodings are demosaiced with ``mrpt_libros_bridge`` >= 4.1.0
       (16-bit Bayer is not supported); images that cannot be converted are dropped.
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
   * - ``odometry_as_robot_pose_observation``
     - No
     - ``Odometry`` only. Boolean (default: the bridge parameter of the same name,
       ``true``). ``true``: forwarded as an SE(3)
       robot pose with covariance (LiDAR or visual odometry); ``false``: as planar
       wheel odometry (``CObservationOdometry``), for which the state estimation smoother
       applies its wheel motion model. Lets wheel and 3D odometry sources be fused together.

Example:

.. code-block:: yaml

    subscribe:
      # Slot always active:
      - topic: /imu
        msg_type: Imu
        output_sensor_label: "imu"

      # Wheel odometry, as planar odometry:
      - topic: /wheel_odom
        msg_type: Odometry
        output_sensor_label: "odom_wheels"
        odometry_as_robot_pose_observation: false

      # Optional slot - disabled when ODOM2_TOPIC is unset:
      - topic: ${ODOM2_TOPIC|}
        msg_type: Odometry
        output_sensor_label: "${ODOM2_LABEL|odom2}"

|

.. _ros2api_topics:

Published topics
----------------
``BridgeROS2`` names each output topic after the module that produces it (its ``method``):
``lidar_odometry`` for MOLA-LO, and the module instance name, ``state_estimation``, for the
smoother. Names are relative, so they are pushed into the namespace of the node, if any.
These are the topics of a typical MOLA-LO system:

.. list-table::
   :header-rows: 1
   :widths: 34 24 42

   * - Topic
     - Type
     - Published
   * - ``lidar_odometry/pose``
     - ``nav_msgs/Odometry``
     - Pose and twist of ``base_link`` in ``map``, when MOLA-LO is the localization source
       (no smoother).
   * - ``state_estimation/pose``
     - ``nav_msgs/Odometry``
     - The fused pose, instead of the above, with the smoother.
   * - ``lidar_odometry/pose_quality``, ``state_estimation/pose_quality``
     - ``std_msgs/Float32``
     - Localization quality, in [0,1].
   * - ``lidar_odometry/localmap_points``
     - ``sensor_msgs/PointCloud2``
     - The local map (transient local QoS), unless ``publish_local_map:=False``. Other map
       layers go to ``<method>/<layer>_points``, and occupancy grid layers to
       ``<method>/<layer>_gridmap`` (``nav_msgs/OccupancyGrid``).
   * - ``lidar_odometry/metadata``
     - ``std_msgs/String``
     - Metadata of the local map, as YAML.
   * - ``lidar_odometry/deskewed_scan_points``
     - ``sensor_msgs/PointCloud2``
     - Each scan after de-skew, only with ``MOLA_LO_PUBLISH_DESKEWED_SCANS=true``.
   * - ``<method>/geo_pose``
     - ``geographic_msgs/GeoPoseStamped``
     - The pose in latitude, longitude and height, once the geo-reference is known.
   * - ``<method>/geo_ref_metadata``
     - ``mrpt_nav_interfaces/GeoreferencingMetadata``
     - The geo-reference (transient local QoS): ``lidar_odometry/...`` for a loaded
       geo-referenced map, ``state_estimator/...`` when estimated by the smoother.
   * - ``mola_diagnostics/<module>/status``
     - ``std_msgs/String``
     - Per-module status as YAML (e.g. ICP quality, processing time, dropped scans).
   * - ``/diagnostics``
     - ``diagnostic_msgs/DiagnosticArray``
     - Health statuses, see :ref:`mola_lo_diagnostics`.
   * - ``/tf``, ``/tf_static``
     - ``tf2_msgs/TFMessage``
     - See :ref:`mola_ros2_tf_frames`.

Log messages (``/rosout``)
==========================

``BridgeROS2`` publishes the log messages of all MOLA modules on ``/rosout``
(``rcl_interfaces/msg/Log``), so they can be inspected with ``rqt_console``,
``ros2 topic echo /rosout``, or recorded into bags, like those of any other ROS 2 node.
Each module appears as a logger named ``<node>.<module>``, e.g. ``mola_bridge_ros2.lidar_odom``.
What MOLA prints to the terminal is not affected.

``BridgeROS2`` YAML parameters:

- ``forward_mola_logs_to_rosout`` (default: ``true``): set to ``false`` to disable it.
- ``rosout_min_level`` (default: ``INFO``): minimum level of the forwarded messages
  (``DEBUG``, ``INFO``, ``WARN``, ``ERROR``). This is independent of the modules'
  ``verbosity_level`` (what they print), but note that ``DEBUG`` makes all modules
  format their debug messages, at some CPU cost.

|

.. _mola_ros2_tf_frames:

Published ``/tf`` frames
------------------------
The frames of reference (``frame_id`` values) at work when using MOLA depend on
your system configuration:

- Using just ``mola_lidar_odometry``: Two situations here depending on the ROS :ref:`launch argument <ros2_node_lo_docs>` ``publish_localization_following_rep105``:

  - (A) Strictly following `ROS REP-105 <https://www.ros.org/reps/rep-0105.html>`_ in systems with wheels (encoders-based) high-frequency odometry, or
  - (B) Not following ``REP-105`` (e.g. if you do not have wheels odometry).

- (C) Using :ref:`state estimation data fusion <mola_sta_est_index>`: the smoother publishes
  ``map → base_link`` by default, or ``map → odom`` to follow REP-105 (see the note in that tab).

And orthogonal to all of them, whether the map is :ref:`geo-referenced <geo-referencing>` or not.

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
      - ``map → odom``: :ref:`Localization <localization-only>` module, which corrects the odometry drift.
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

      - ``map → base_link``: :ref:`Localization <localization-only>` module.
      - ``enu → {map, utm}``: Published by ``mrpt_map_server`` (`github <https://github.com/mrpt-ros-pkg/mrpt_navigation/tree/ros2/mrpt_map_server/>`_)
        or ``mola_lidar_odometry`` :ref:`map loading service <map_loading_saving>` if fed with a geo-referenced metric map (``.mm``) file.

   .. tab-item:: (C) Data fusion

         When using :ref:`state estimation data fusion <mola_sta_est_index>`: applicable if having just one LiDAR sensor,
         or LiDAR + wheel odometry, or several odometry sources, optionally GNSS (GPS) and IMU, etc.

      .. figure:: https://mrpt.github.io/imgs/mola_mrpt_ros_frames_fusion.png
         :width: 500
         :align: center

      This is who is responsible of publishing each transformation:

      - ``map → base_link``: Published by :ref:`state estimation data fusion <mola_sta_est_index>`.
        The odometry sources (``odom_{i}``) are fused from their topics, not published on ``/tf``.
      - Alternatively, ``map → odom`` for one odometry source whose driver publishes
        ``odom → base_link`` (REP-105): see :ref:`this example <localization-lo_georef_gnss_imu_kinematics>`.
      - ``enu → {map, utm}``: Published by ``mrpt_map_server`` (`github <https://github.com/mrpt-ros-pkg/mrpt_navigation/tree/ros2/mrpt_map_server/>`_)
        or ``mola_lidar_odometry`` :ref:`map loading service <map_loading_saving>` if fed with a geo-referenced metric map (``.mm``) file.

      .. note::

         External odometry sources must enter MOLA as ``nav_msgs/Odometry``
         topics (consumed as observations), **not** via /tf. If a wheel
         driver also broadcasts ``odom → base_link`` to /tf while the
         state estimator publishes ``map → base_link``, ``base_link``
         ends up with two parents and tf2 rejects the tree. See the
         :ref:`mola_ros2_cookbook_4_4`.

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

Map publishing
--------------
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

.. _map_loading_saving:

Map loading / saving
--------------------
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

.. _mola_ros2api_relocalization:

Re-localization
---------------
There are two ROS services that can be used to enforce the MOLA subsystem to relocalize, for example,
to address the problem of initial localization:


Specify the new localization and its initial uncertainty
========================================================

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


Request automatic relocalization from GNSS (GPS)
================================================

In geo-referenced maps, it is possible to request MOLA-LO to use incoming GPS readings to
bootstrap LiDAR-based localization. 

.. note::
   
   This method requires the use of the :ref:`smoother state estimator <mola_sta_est_index>`.

Request a relocalization now with:

   .. code-block:: bash

      ros2 service call /relocalize_from_state_estimator  mola_msgs/srv/RelocalizeFromStateEstimator "{}"


Depending on the parameters, it may take some time for the re-localization to take effect.

|

.. _ros2api_runtime_params:

Runtime dynamic reconfiguration
-------------------------------
MOLA modules may expose a subset of their parameters through an interface that allows
runtime reconfiguration via ROS 2 service requests:

Runtime parameters for ``mola_lidar_odometry``
==============================================

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


- ``generate_simplemap``: Whether MOLA-LO should build the keyframes-based map (apart from the local metric map),
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
