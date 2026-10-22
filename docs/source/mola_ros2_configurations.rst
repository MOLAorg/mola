.. _mola_ros2_cookbook:

========================
ROS 2 configurations
========================

This page walks through every supported way of launching MOLA-LO / MOLA-SLAM
in a ROS 2 workflow. Each configuration shows a block diagram, the exact
command line, and the expected ``/tf`` tree.

.. admonition:: Live parameter editor

   The :ref:`parameter editor <mola_ros2_cookbook_form>` lets you set your own
   robot frame, topic names, bag path, and namespace; the command blocks
   below will be rewritten automatically and persisted in your browser.

____________________________________________

.. contents::
   :depth: 2
   :local:
   :backlinks: none

____________________________________________

|

.. _mola_ros2_cookbook_axes:

The five axes of configuration
------------------------------------

A MOLA ROS 2 deployment is defined by five independent choices:

.. list-table::
   :header-rows: 1
   :widths: 18 30 52

   * - Axis
     - Choices
     - What it affects
   * - **1. Input**
     - Offline ``rosbag2`` (``mola-lo-cli-rosbag2`` or
       ``mola-lo-gui-rosbag2``) vs live **ROS 2 node**
     - Which module provides sensor data:
       :ref:`Rosbag2Dataset <doxid-classmola_1_1_rosbag2_dataset>` vs
       :ref:`BridgeROS2 <doxid-classmola_1_1_bridge_r_o_s2>`.
   * - **2. Namespace**
     - Plain ``/tf``/topics vs namespaced (``/robot1/tf``, ``/robot1/scan``, …)
     - Whether the launch uses ``use_namespace:=True`` (online) or sets
       ``MOLA_TF_TOPIC`` / ``MOLA_TF_STATIC_TOPIC`` (offline).
   * - **3. State estimator**
     - :ref:`StateEstimationSimple <mola_sta_est_index>` (default) vs
       :ref:`StateEstimationSmoother <mola_sta_est_index>`
     - Which module publishes the final fused pose.
   * - **4. REP-105**
     - Publish ``map → odom`` (REP-105) vs direct ``map → base_link``
     - Whether an external wheel / visual odometry source is expected.
       **REP-105 is only compatible with the Simple estimator.**
   * - **5. Extra sensors**
     - IMU (for LIO / gravity), GNSS (logging, live geo-ref, relocalization),
       attitude-only IMU (map geo-referencing), multiple odometry
       sources (Smoother only)
     - Topics subscribed, factors added, output TFs.

.. hint::

   The most common pitfall is ``REP-105 + Smoother``: the smoother
   publishes ``map → base_link`` directly and cannot split the transform.

|

.. _mola_ros2_cookbook_decision:

Choosing a configuration
-------------------------

Each box names a section below.

.. mermaid::

   flowchart TD
     Start([Where does your data come from?]) --> Q1{Live ROS 2<br/>or rosbag file?}
     Q1 -- rosbag --> Off[Offline rosbag2:<br/>GUI or batch CLI]
     Q1 -- live --> Q2{Namespaced<br/>topics / tf?}
     Q2 -- yes --> NS[Namespaced robot]
     Q2 -- no --> Q3{Using GNSS?}
     Q3 -- live geo-referencing --> LG[Smoother + live<br/>geo-referencing]
     Q3 -- localize in a<br/>geo-referenced map --> RL[Smoother +<br/>relocalization]
     Q3 -- build a map to<br/>geo-reference later --> Geo[Recording a map for<br/>later geo-referencing]
     Q3 -- no --> Q4{External wheel / VIO<br/>odometry?}
     Q4 -- no --> S2[Simple estimator,<br/>no external odometry]
     Q4 -- yes, on /tf --> S1[Simple estimator<br/>+ REP-105]
     Q4 -- yes, as topics --> SM[Smoother + external<br/>odometry fusion]
     Off -. GNSS + AHRS .-> Geo

|

.. _mola_ros2_cookbook_form:

Parameter editor
----------------------

.. raw:: html

   <div id="mola-cookbook-form" class="mola-cookbook-form">
     <p><em>Loading editor… if this message stays, JavaScript is disabled and
     the commands below show default placeholder values.</em></p>
   </div>

Each knob below has two spellings: a ``ros2 launch`` argument for the live
node, and an environment variable read by both offline tools
(``mola-lo-gui-rosbag2`` and ``mola-lo-cli-rosbag2``).

.. list-table::
   :header-rows: 1
   :widths: 22 16 31 31

   * - Concept
     - Default
     - Live: ``ros2 launch`` argument
     - Offline: environment variable
   * - Robot base frame
     - ``base_link``
     - ``mola_tf_base_link:=``
     - ``MOLA_TF_BASE_LINK``
   * - Map frame
     - ``map``
     - ``mola_state_estimator_reference_frame:=``
     - (no ``/tf`` is published)
   * - Odom frame (REP-105 only)
     - ``odom``
     - ``mola_bridge_odometry_frame:=``
     - (no ``/tf`` is published)
   * - LiDAR topic
     - ``/ouster/points``
     - ``lidar_topic_name:=`` (+ ``lidar_topic_type:=LaserScan`` for 2D)
     - ``MOLA_LIDAR_TOPIC``
   * - IMU topic
     - ``/imu``
     - ``imu_topic_name:=``
     - ``MOLA_IMU_TOPIC``
   * - GNSS topic
     - ``/gps``
     - ``gnss_topic_name:=`` (``NavSatFix``),
       ``gpsfix_topic_name:=`` (``gps_msgs/GPSFix``)
     - ``MOLA_GNSS_TOPIC`` (either type, auto-detected)
   * - Attitude-only IMU (AHRS), see :ref:`mola_ros2_cookbook_5_4`
     - (off)
     - ``attitude_imu_topic_name:=``
     - ``MOLA_ATTITUDE_IMU_TOPIC``
   * - External odom (``/tf``)
     - (off)
     - ``forward_ros_tf_odom_to_mola:=True``
     - (not available)
   * - External odom (topic)
     - (off)
     - ``odom_topic_name:=`` (+ ``odom_sensor_label:=``)
     - ``MOLA_ODOMETRY_TOPIC`` (+ ``MOLA_ODOM_SENSOR_LABEL``,
       ``MOLA_ODOMETRY_OBS_CLASS``)
   * - Ignore sensor pose from ``/tf``
     - ``false``
     - ``ignore_lidar_pose_from_tf:=``, ``ignore_imu_pose_from_tf:=``
     - ``MOLA_USE_FIXED_LIDAR_POSE``, ``MOLA_USE_FIXED_IMU_POSE``,
       ``MOLA_USE_FIXED_GNSS_POSE`` (pose in ``LIDAR_POSE_X``... ``_ROLL``, etc.)
   * - ``/tf`` and ``/tf_static`` topics
     - ``/tf``, ``/tf_static``
     - ``use_namespace:=True`` (see :ref:`mola_ros2_cookbook_4_3`)
     - ``MOLA_TF_TOPIC``, ``MOLA_TF_STATIC_TOPIC``
   * - Save a ``.simplemap``
     - (off)
     - ``generate_simplemap:=True``
     - CLI: ``--output-simplemap``; GUI: ``MOLA_GENERATE_SIMPLEMAP=true``
       (+ ``MOLA_SIMPLEMAP_OUTPUT``)
   * - LiDAR/IMU subscription QoS
     - ``best_effort`` / depth ``50``
     - ``lidar_qos_reliability:=``, ``lidar_qos_depth:=``,
       ``imu_qos_reliability:=``, ``imu_qos_depth:=``
     - (not applicable)

.. note::

   Subscription QoS for sensor topics defaults to REP-2003 (``best_effort``,
   ``volatile``) with a queue depth of 50. For high-rate sensors consumed
   together with a heavy SLAM pipeline (e.g. a 640 Hz IMU), set
   ``imu_qos_reliability:=reliable`` and increase ``imu_qos_depth`` (e.g.
   ``1000``) so the bridge subscription matches the publisher and can
   absorb executor stalls. See the launch arguments documented in
   :ref:`the ROS 2 node reference <ros2_node_lo_docs>` and the per-topic ``qos:`` block accepted by
   ``BridgeROS2``'s ``subscribe`` list.

|

.. _mola_ros2_cookbook_online:

Live ROS 2 node
------------------------

All sections in this group use
``ros2 launch mola_lidar_odometry ros2-lidar-odometry.launch.py``. See
:ref:`all launch arguments <mola_lo_ros_launch_arguments>` for the full list.

.. _mola_ros2_cookbook_4_1:

Simple estimator + REP-105 (external odometry on /tf)
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

Use this when the robot already publishes a high-frequency
``odom → base_link`` transform from wheel encoders (or any other odometry
source). MOLA-LO only corrects long-term drift by publishing ``map → odom``.

.. mermaid::

   flowchart LR
     subgraph ROS2[ROS 2 topics / tf]
       lidar[/"LiDAR topic"/]
       odomtf[/'tf: odom → base_link'/]
       mapodom[/'tf: map → odom'/]
     end
     wheel[Wheel odometry driver] -->|'tf'| odomtf
     lidar --> bridge[BridgeROS2]
     bridge --> lo[mola::LidarOdometry]
     lo --> bridge
     se --> lo
     lo --> se[StateEstimationSimple]
     bridge -->|odom| se
     bridge -->|map → odom| mapodom

.. container:: mola-tpl

   .. code-block:: bash

      # LiDAR only odometry (LO): No IMU
      ros2 launch mola_lidar_odometry ros2-lidar-odometry.launch.py \
        lidar_topic_name:=__LIDAR_TOPIC__ \
        mola_tf_base_link:=__BASE_LINK__ \
        forward_ros_tf_odom_to_mola:=True \
        publish_localization_following_rep105:=True

Resulting ``/tf`` tree::

   map --→ odom --→ base_link --→ (sensor frames)
        ^        ^
        |        └── wheel odometry (external)
        └── MOLA-LO (corrects drift)

.. dropdown:: Variant: LIO (LiDAR + IMU), REP-105
   :icon: code-review
   :open:


   Adds IMU-based scan deskew. Gravity alignment is on by default.

   .. mermaid::

      flowchart LR
        lidar[/"LiDAR topic"/] --> bridge[BridgeROS2]
        imu[/"IMU topic"/] --> bridge
        bridge --> lo[mola::LidarOdometry]
        lo --> bridge
        lo --> se[StateEstimationSimple]
        se --> lo
        bridge -->|map → odom| tfout[/'tf'/]
        bridge -->|odom| se

   .. container:: mola-tpl

      .. code-block:: bash

         # LiDAR inertial odometry (LIO): with IMU
         ros2 launch mola_lidar_odometry ros2-lidar-odometry.launch.py \
           lidar_topic_name:=__LIDAR_TOPIC__ \
           imu_topic_name:=__IMU_TOPIC__ \
           mola_tf_base_link:=__BASE_LINK__ \
           use_imu_for_lio:=True \
           forward_ros_tf_odom_to_mola:=True \
           publish_localization_following_rep105:=True

|

.. _mola_ros2_cookbook_4_2:

Simple estimator, no external odometry
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

When no external odometry exists (e.g. handheld mapping, drone), MOLA-LO
publishes ``map → base_link`` directly.

.. mermaid::

   flowchart LR
     lidar[/"LiDAR topic"/] --> bridge[BridgeROS2]
     bridge --> lo[mola::LidarOdometry]
     lo --> se[StateEstimationSimple]
     lo --> bridge
     se --> lo
     bridge -->|map → base_link| tfout[/'tf'/]

.. container:: mola-tpl

   .. code-block:: bash

      # LiDAR only odometry (LO): No IMU
      ros2 launch mola_lidar_odometry ros2-lidar-odometry.launch.py \
        lidar_topic_name:=__LIDAR_TOPIC__ \
        mola_tf_base_link:=__BASE_LINK__ \
        publish_localization_following_rep105:=False

Resulting ``/tf`` tree::

   map --→ base_link --→ (sensor frames)

.. dropdown:: Variant: LIO (LiDAR + IMU), no REP-105
   :icon: code-review
   :open:

   Adds IMU-based scan deskew. Gravity alignment is on by default.

   .. mermaid::

      flowchart LR
        lidar[/"LiDAR topic"/] --> bridge[BridgeROS2]
        imu[/"IMU topic"/] --> bridge
        bridge --> lo[mola::LidarOdometry]
        bridge -->|imu| se[StateEstimationSimple]
        lo --> se
        lo --> bridge
        se --> lo
        bridge -->|map → base_link| tfout[/'tf'/]


   .. container:: mola-tpl

      .. code-block:: bash

         # LiDAR inertial odometry (LIO): with IMU
         ros2 launch mola_lidar_odometry ros2-lidar-odometry.launch.py \
           lidar_topic_name:=__LIDAR_TOPIC__ \
           imu_topic_name:=__IMU_TOPIC__ \
           mola_tf_base_link:=__BASE_LINK__ \
           use_imu_for_lio:=True \
           publish_localization_following_rep105:=False


.. dropdown:: Variant: no sensor ``/tf`` available
   :icon: alert

   If the driver does not publish ``base_link → lidar`` on ``/tf``,
   either configure it via a ``robot_state_publisher`` or force an
   identity sensor pose:

   .. container:: mola-tpl

      .. code-block:: bash

         ros2 launch mola_lidar_odometry ros2-lidar-odometry.launch.py \
           lidar_topic_name:=__LIDAR_TOPIC__ \
           mola_tf_base_link:=__BASE_LINK__ \
           ignore_lidar_pose_from_tf:=True \
           publish_localization_following_rep105:=False

|

.. _mola_ros2_cookbook_4_3:

Namespaced robot (``/robot1/tf``, ``/robot1/scan``, ...)
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

When your robot publishes everything under a ROS 2 namespace, the launch
file pushes the whole MOLA system into that namespace and remaps ``/tf`` /
``/tf_static`` so tf2 inside the namespace is transparent to MOLA.

.. mermaid::

   flowchart LR
     subgraph NS["robot namespace"]
       lidar[/"LiDAR topic"/]
       tfns["/tf (namespaced)/"]
       bridge[BridgeROS2]
       lo[mola::LidarOdometry]
       se[StateEstimationSimple]
     end
     lidar --> bridge
     bridge --> lo --> se --> bridge
     bridge --> tfns

.. container:: mola-tpl

   .. code-block:: bash

      ros2 launch mola_lidar_odometry ros2-lidar-odometry.launch.py \
        lidar_topic_name:=__LIDAR_TOPIC__ \
        mola_tf_base_link:=__BASE_LINK__ \
        use_namespace:=True \
        namespace:=__NS__

.. note::

   The ``lidar_topic_name`` should be given **relative** (no leading ``/``).
   It gets prefixed by the namespace automatically. The launch file remaps
   ``/tf`` ↔ ``tf`` internally so tf2 subscribes to
   ``/__NS__/tf`` without any extra configuration. If only the sensor
   topics are namespaced but ``/tf`` is global, add
   ``use_namespaced_tf:=False``.

|

.. _mola_ros2_cookbook_4_4:

Smoother + external odometry fusion (wheels, VIO)
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

Adds an external odometry source to the smoother. The smoother estimates
a distinct ``T_map_to_odom`` per source and fuses their deltas with LO
(and IMU, if enabled).

.. important::

   With the Smoother, external odometry must arrive as a
   ``nav_msgs/Odometry`` topic (``odom_topic_name:=``). The Smoother
   publishes ``map → base_link`` itself, so an external ``odom → base_link``
   on ``/tf`` would give ``base_link`` two parents; the launch file rejects
   ``forward_ros_tf_odom_to_mola:=True`` together with
   ``use_state_estimator:=True``.

The upstream driver publishes a ``nav_msgs/Odometry`` message on a topic.
BridgeROS2 subscribes directly, converts each message to a 3D
``CObservationRobotPose`` (6×6 covariance) and forwards it under
``odom_sensor_label``. This preserves Z / pitch / roll and allows fusing
**multiple** labeled sources.

.. mermaid::

   flowchart LR
     lidar[/"LiDAR topic"/] --> bridge[BridgeROS2]
     imu[/"IMU topic"/] --> bridge
     wheel[/wheel odom topic/] --> bridge
     bridge --> lo[mola::LidarOdometry]
     bridge --> smoother[StateEstimationSmoother]
     lo --> smoother
     smoother --> bridge
     bridge -->|map → base_link| tfout[/'tf'/]

.. container:: mola-tpl

   .. code-block:: bash

      ros2 launch mola_lidar_odometry ros2-lidar-odometry.launch.py \
        lidar_topic_name:=__LIDAR_TOPIC__ \
        imu_topic_name:=__IMU_TOPIC__ \
        mola_tf_base_link:=__BASE_LINK__ \
        use_state_estimator:=True \
        use_imu_for_lio:=True \
        odom_topic_name:=__ODOM_TOPIC__ \
        odom_sensor_label:=odom_wheels

For multiple external sources (e.g. wheels + VIO), add extra
``subscribe`` entries to ``lidar_odometry_ros2.yaml`` with distinct
``output_sensor_label`` values.

.. list-table:: Odometry ingress: when to use which
   :header-rows: 1
   :widths: 28 36 36

   * -
     - Via ``/tf``
     - Via ``/odom`` topic
   * - Launch arg
     - ``forward_ros_tf_odom_to_mola:=True``
     - ``odom_topic_name:=/your_odom``
   * - Upstream must publish
     - ``odom → base_link`` TF
     - ``nav_msgs/Odometry`` message
   * - Observation type fed to MOLA
     - ``CObservationOdometry`` (2D, no covariance)
     - ``CObservationRobotPose`` (3D, 6×6 covariance)
   * - Sensor label
     - Hardcoded ``odom``
     - ``odom_sensor_label:=...`` (per source)
   * - Multiple sources
     - No (single TF chain)
     - Yes (multiple subscribe entries / labels)
   * - Rate
     - BridgeROS2 ``execution_rate`` (20 Hz)
     - Publisher rate
   * - Compatible with
     - **Simple estimator + REP-105 only**
       (:ref:`mola_ros2_cookbook_4_1`). Rejected with the Smoother.
     - Simple estimator (no REP-105) and **Smoother** (recommended).

See the :ref:`"Fusing two Odometry sources" tutorial <mola_sta_est_index>`
for a full worked example with fake publishers.

|

.. _mola_ros2_cookbook_4_5:

Smoother + live geo-referencing (GNSS)
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

The smoother estimates the ``enu → map`` transform online from incoming
GNSS fixes. It publishes ``enu`` and ``utm`` TFs once convergence is
reached.

.. mermaid::

   flowchart LR
     lidar[/"LiDAR topic"/] --> bridge[BridgeROS2]
     imu[/"IMU topic"/] --> bridge
     gnss[/"GNSS topic"/] --> bridge
     bridge --> lo[mola::LidarOdometry]
     bridge --> smoother[StateEstimationSmoother]
     lo --> smoother
     smoother --> bridge
     bridge -->|'map → base_link, enu → map'| tfout[/'tf'/]

.. container:: mola-tpl

   .. code-block:: bash

      ros2 launch mola_lidar_odometry ros2-lidar-odometry.launch.py \
        lidar_topic_name:=__LIDAR_TOPIC__ \
        imu_topic_name:=__IMU_TOPIC__ \
        gnss_topic_name:=__GNSS_TOPIC__ \
        mola_tf_base_link:=__BASE_LINK__ \
        use_state_estimator:=True \
        use_imu_for_lio:=True \
        gnss_mode:=live_georef

|

.. _mola_ros2_cookbook_4_6:

Smoother + relocalization in a geo-referenced map
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

Load a previously built geo-referenced ``.mm`` map. MOLA-LO stays idle
until the smoother has converged on an initial pose from GNSS, then
switches to localization-only mapping.

.. mermaid::

   flowchart LR
     mm[(.mm map)] --> lo
     lidar[/"LiDAR topic"/] --> bridge[BridgeROS2]
     imu[/"IMU topic"/] --> bridge
     gnss[/"GNSS topic"/] --> bridge
     bridge --> lo[mola::LidarOdometry]
     bridge --> smoother[StateEstimationSmoother]
     lo --> smoother
     smoother --> bridge
     bridge -->|'map → base_link, enu → map'| tfout[/'tf'/]

.. container:: mola-tpl

   .. code-block:: bash

      ros2 launch mola_lidar_odometry ros2-lidar-odometry.launch.py \
        lidar_topic_name:=__LIDAR_TOPIC__ \
        imu_topic_name:=__IMU_TOPIC__ \
        gnss_topic_name:=__GNSS_TOPIC__ \
        mola_tf_base_link:=__BASE_LINK__ \
        mola_initial_map_mm_file:=__MM_PATH__ \
        use_state_estimator:=True \
        use_imu_for_lio:=True \
        start_mapping_enabled:=False \
        gnss_mode:=relocalize

|

.. _mola_ros2_cookbook_offline:

Offline rosbag2
------------------------

.. _mola_ros2_cookbook_5_1:
.. _mola_ros2_cookbook_5_2:
.. _mola_ros2_cookbook_5_3:

Odometry and mapping from a bag (GUI or batch CLI)
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

Two wrappers replay a bag, configured through the **same environment
variables** (see the offline column of the
:ref:`parameter table <mola_ros2_cookbook_form>`):

- ``mola-lo-gui-rosbag2``: replay with the MolaViz GUI, to watch and
  control the run.
- ``mola-lo-cli-rosbag2``: no GUI, as fast as the CPU allows. Best for long
  bags, batch runs and reproducible benchmarks. Extra arguments after the
  bag path go to ``mola-lidar-odometry-cli`` (run it with ``--help`` for
  all output options).

.. mermaid::

   flowchart LR
     bag[(rosbag2)] --> ds[Rosbag2Dataset]
     ds --> lo[mola::LidarOdometry]
     ds --> se[StateEstimationSimple / Smoother]
     lo --> se
     se --> lo
     lo --> out[[GUI, trajectory.tum, map.simplemap]]

.. tab-set::

    .. tab-item:: mola-lo-gui-rosbag2 (GUI)
        :sync: gui

        .. tab-set::

            .. tab-item:: Lidar only (LO)
                :sync: lo

                .. container:: mola-tpl

                  .. code-block:: bash

                      MOLA_LIDAR_TOPIC=__LIDAR_TOPIC__ \
                      MOLA_TF_BASE_LINK=__BASE_LINK__ \
                        mola-lo-gui-rosbag2 __BAG_PATH__

            .. tab-item:: Lidar-Inertial (LIO)
                :sync: lio
                :selected:

                .. container:: mola-tpl

                  .. code-block:: bash

                      MOLA_DESKEW_METHOD=MotionCompensationMethod::IMU \
                      MOLA_LO_INITIAL_LOCALIZATION_METHOD=InitLocalization::PitchAndRollFromIMU \
                      MOLA_LIDAR_TOPIC=__LIDAR_TOPIC__ \
                      MOLA_IMU_TOPIC=__IMU_TOPIC__ \
                      MOLA_TF_BASE_LINK=__BASE_LINK__ \
                        mola-lo-gui-rosbag2 __BAG_PATH__

            .. tab-item:: LIO + Smoother
                :sync: smoother

                .. container:: mola-tpl

                  .. code-block:: bash

                      MOLA_DESKEW_METHOD=MotionCompensationMethod::IMU \
                      MOLA_LIDAR_TOPIC=__LIDAR_TOPIC__ \
                      MOLA_IMU_TOPIC=__IMU_TOPIC__ \
                      MOLA_TF_BASE_LINK=__BASE_LINK__ \
                      MOLA_STATE_ESTIMATOR=mola::state_estimation_smoother::StateEstimationSmoother \
                      MOLA_STATE_ESTIMATOR_YAML="$(ros2 pkg prefix mola_state_estimation_smoother)/share/mola_state_estimation_smoother/params/state-estimation-smoother.yaml" \
                        mola-lo-gui-rosbag2 __BAG_PATH__

    .. tab-item:: mola-lo-cli-rosbag2 (batch, no GUI)
        :sync: cli

        .. tab-set::

            .. tab-item:: Lidar only (LO)
                :sync: lo

                .. container:: mola-tpl

                  .. code-block:: bash

                      MOLA_LIDAR_TOPIC=__LIDAR_TOPIC__ \
                      MOLA_TF_BASE_LINK=__BASE_LINK__ \
                        mola-lo-cli-rosbag2 __BAG_PATH__ \
                          --output-tum-path trajectory.tum \
                          --output-simplemap map.simplemap

            .. tab-item:: Lidar-Inertial (LIO)
                :sync: lio
                :selected:

                .. container:: mola-tpl

                  .. code-block:: bash

                      MOLA_DESKEW_METHOD=MotionCompensationMethod::IMU \
                      MOLA_LO_INITIAL_LOCALIZATION_METHOD=InitLocalization::PitchAndRollFromIMU \
                      MOLA_LIDAR_TOPIC=__LIDAR_TOPIC__ \
                      MOLA_IMU_TOPIC=__IMU_TOPIC__ \
                      MOLA_TF_BASE_LINK=__BASE_LINK__ \
                        mola-lo-cli-rosbag2 __BAG_PATH__ \
                          --output-tum-path trajectory.tum \
                          --output-simplemap map.simplemap

            .. tab-item:: LIO + Smoother
                :sync: smoother

                .. container:: mola-tpl

                  .. code-block:: bash

                      MOLA_DESKEW_METHOD=MotionCompensationMethod::IMU \
                      MOLA_LIDAR_TOPIC=__LIDAR_TOPIC__ \
                      MOLA_IMU_TOPIC=__IMU_TOPIC__ \
                      MOLA_TF_BASE_LINK=__BASE_LINK__ \
                      MOLA_STATE_ESTIMATOR=mola::state_estimation_smoother::StateEstimationSmoother \
                      MOLA_STATE_ESTIMATOR_YAML="$(ros2 pkg prefix mola_state_estimation_smoother)/share/mola_state_estimation_smoother/params/state-estimation-smoother.yaml" \
                        mola-lo-cli-rosbag2 __BAG_PATH__ \
                          --output-tum-path trajectory.tum \
                          --output-simplemap map.simplemap


.. dropdown:: Variant: bag recorded with a ROS 2 namespace
   :icon: alert

   Rosbag2Dataset looks up topics by their literal name (there is no tf2
   remap as in the :ref:`live node <mola_ros2_cookbook_4_3>`), so both the
   sensor topics and the ``/tf`` topics must carry the namespace. Enter the
   sensor topics with their prefix in the form above (e.g.
   ``/__NS__/ouster/points``) and add the two ``/tf`` variables to any
   command above:

   .. container:: mola-tpl

      .. code-block:: bash

         MOLA_TF_TOPIC=/__NS__/tf \
         MOLA_TF_STATIC_TOPIC=/__NS__/tf_static \
         MOLA_LIDAR_TOPIC=__LIDAR_TOPIC__ \
         MOLA_TF_BASE_LINK=__BASE_LINK__ \
           mola-lo-gui-rosbag2 __BAG_PATH__

   ``MOLA_TF_BASE_LINK`` is a **frame id** inside the TF messages, so it is
   **not** namespaced. Requires ``mola_input_rosbag2 >= 1.12``.

.. dropdown:: Variant: calling ``mola-lidar-odometry-cli`` directly
   :icon: code-review

   ``mola-lo-cli-rosbag2`` is a thin wrapper that fills in the pipeline and
   state estimator files. The equivalent raw call, with every input given
   as a flag, is:

   .. container:: mola-tpl

      .. code-block:: bash

         mola-lidar-odometry-cli \
           -c $(ros2 pkg prefix mola_lidar_odometry)/share/mola_lidar_odometry/pipelines/lidar3d-default.yaml \
           --state-estimator-param-file $(ros2 pkg prefix mola_lidar_odometry)/share/mola_lidar_odometry/state-estimator-params/state-estimation-simple.yaml \
           --input-rosbag2 __BAG_PATH__ \
           --lidar-sensor-label __LIDAR_TOPIC__ \
           --base-link-frame-id __BASE_LINK__ \
           --output-tum-path trajectory.tum

|

.. _mola_ros2_cookbook_5_4:

Recording a map for later geo-referencing (GNSS + attitude IMU)
--------------------------------------------------------------------

To :ref:`geo-reference <geo-referencing>` a map afterwards with
``mola-sm-georeferencing``, each ``.simplemap`` keyframe must carry the
nearest GNSS fix and IMU reading. The IMU reading gives the azimuth
observation, so it must hold an **absolute orientation**: the IMU built
into most LiDARs is fine for de-skew but has none.

If the robot has a separate AHRS (or a dual-antenna GNSS publishing a
``sensor_msgs/Imu`` attitude), name it as the attitude IMU topic. Its
readings are then the ones stored into keyframes, while the main IMU keeps
doing de-skew and gravity alignment. Without it, keyframes store readings
from the main IMU, which is the right choice when that IMU already reports
an absolute orientation.

.. mermaid::

   flowchart LR
     src[(rosbag2 or live topics)] -->|LiDAR| lo[mola::LidarOdometry]
     src -->|IMU: deskew| lo
     src -->|attitude IMU| lo
     src -->|GNSS| lo
     lo --> sm[[map.simplemap]]
     sm --> geo[mola-sm-georeferencing]

.. tab-set::

    .. tab-item:: mola-lo-gui-rosbag2 (GUI)
        :sync: gui

        .. container:: mola-tpl

          .. code-block:: bash

              MOLA_DESKEW_METHOD=MotionCompensationMethod::IMU \
              MOLA_LIDAR_TOPIC=__LIDAR_TOPIC__ \
              MOLA_IMU_TOPIC=__IMU_TOPIC__ \
              MOLA_GNSS_TOPIC=__GNSS_TOPIC__ \
              MOLA_ATTITUDE_IMU_TOPIC=__ATTITUDE_IMU_TOPIC__ \
              MOLA_TF_BASE_LINK=__BASE_LINK__ \
              MOLA_GENERATE_SIMPLEMAP=true \
              MOLA_SIMPLEMAP_OUTPUT=map.simplemap \
                mola-lo-gui-rosbag2 __BAG_PATH__

    .. tab-item:: mola-lo-cli-rosbag2 (batch, no GUI)
        :sync: cli

        .. container:: mola-tpl

          .. code-block:: bash

              MOLA_DESKEW_METHOD=MotionCompensationMethod::IMU \
              MOLA_LIDAR_TOPIC=__LIDAR_TOPIC__ \
              MOLA_IMU_TOPIC=__IMU_TOPIC__ \
              MOLA_GNSS_TOPIC=__GNSS_TOPIC__ \
              MOLA_ATTITUDE_IMU_TOPIC=__ATTITUDE_IMU_TOPIC__ \
              MOLA_TF_BASE_LINK=__BASE_LINK__ \
                mola-lo-cli-rosbag2 __BAG_PATH__ \
                  --output-simplemap map.simplemap

    .. tab-item:: Live ROS 2 node
        :sync: live

        .. container:: mola-tpl

          .. code-block:: bash

              MOLA_SIMPLEMAP_OUTPUT=map.simplemap \
              ros2 launch mola_lidar_odometry ros2-lidar-odometry.launch.py \
                lidar_topic_name:=__LIDAR_TOPIC__ \
                imu_topic_name:=__IMU_TOPIC__ \
                gnss_topic_name:=__GNSS_TOPIC__ \
                attitude_imu_topic_name:=__ATTITUDE_IMU_TOPIC__ \
                mola_tf_base_link:=__BASE_LINK__ \
                use_imu_for_lio:=True \
                generate_simplemap:=True

        The map file is written when the node shuts down.

.. note::

   The attitude IMU also reaches the state estimator. With the Smoother,
   exclude its label ``imu_attitude`` via ``do_process_imu_labels_re``
   unless fusing its absolute attitude is intended.

|

.. _mola_ros2_cookbook_fixes:

Troubleshooting
------------------------

Moved to its own page: :ref:`troubleshooting`. It covers the frame and topic
problems that used to be listed here, plus the failure modes that are not
specific to ROS 2.
