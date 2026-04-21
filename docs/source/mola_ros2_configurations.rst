.. _mola_ros2_cookbook:

========================
ROS 2 configurations
========================

This page walks through every supported way of launching MOLA-LO / MOLA-SLAM
in a ROS 2 workflow. Each configuration shows a block diagram, the exact
command line, and the expected ``/tf`` tree.

.. admonition:: Live parameter editor

   The form at :ref:`§3 <mola_ros2_cookbook_form>` lets you set your own
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

1. The five axes of configuration
------------------------------------

A MOLA ROS 2 deployment is defined by five independent choices:

.. list-table::
   :header-rows: 1
   :widths: 18 30 52

   * - Axis
     - Choices
     - What it affects
   * - **1. Input**
     - Offline ``rosbag2`` (``mola-lidar-odometry-cli`` or
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
       multiple odometry sources (Smoother only)
     - Topics subscribed, factors added, output TFs.

.. hint::

   The most common pitfall is ``REP-105 + Smoother``: the smoother
   publishes ``map → base_link`` directly and cannot split the transform.

|

.. _mola_ros2_cookbook_decision:

2. Decision flowchart
-------------------------

.. mermaid::

   flowchart TD
     Start([Where does your data come from?]) --> Q1{Live ROS 2<br/>or rosbag file?}
     Q1 -- rosbag --> QO1{GUI or fastest<br/>offline run?}
     Q1 -- Live --> Q2{Namespaced<br/>topics/tf?}
     QO1 -- GUI --> QO2[§5.1 mola-lo-gui-rosbag2]
     QO1 -- fastest --> QO3[§5.3 mola-lidar-odometry-cli]
     Q2 -- yes --> Q3ns[§4.3 namespaced online]
     Q2 -- no --> Q3{Need sensor fusion<br/>GNSS / multi-odom?}
     Q3 -- no --> Q4{Wheel / external<br/>odometry available?}
     Q3 -- yes --> Q5[§4.4 … §4.6]
     Q4 -- yes --> Q4a[§4.1 Simple + REP-105]
     Q4 -- no --> Q4b[§4.2 Simple, no REP-105]

|

.. _mola_ros2_cookbook_form:

3. Parameter editor
----------------------

.. raw:: html

   <div id="mola-cookbook-form" class="mola-cookbook-form">
     <p><em>Loading editor… if this message stays, JavaScript is disabled and
     the commands below show default placeholder values.</em></p>
   </div>

.. list-table::
   :header-rows: 1
   :widths: 28 22 50

   * - Concept
     - Typical value
     - Where it is set
   * - Robot base frame
     - ``base_link``
     - ``mola_tf_base_link:=`` (online) / ``MOLA_TF_BASE_LINK`` (offline)
   * - Map frame
     - ``map``
     - ``mola_state_estimator_reference_frame:=`` / ``MOLA_TF_MAP``
   * - Odom frame (REP-105 only)
     - ``odom``
     - ``mola_lo_reference_frame:=`` / ``MOLA_TF_ESTIMATED_ODOMETRY``
   * - LiDAR topic
     - ``/ouster/points``
     - ``lidar_topic_name:=`` / ``MOLA_LIDAR_TOPIC``
   * - IMU topic
     - ``/imu``
     - ``imu_topic_name:=`` / ``MOLA_IMU_TOPIC``
   * - GNSS topic (NavSatFix)
     - ``/gps``
     - ``gnss_topic_name:=`` / ``MOLA_GNSS_TOPIC``
   * - External odom (``/tf``)
     - (off)
     - ``forward_ros_tf_odom_to_mola:=True`` / ``MOLA_FORWARD_ROS_TF_ODOM_TO_MOLA``
   * - External odom (topic)
     - ``/wheel_odom``
     - ``odom_topic_name:=`` / ``MOLA_ODOM_TOPIC`` (+ ``odom_sensor_label:=`` / ``MOLA_ODOM_SENSOR_LABEL``)
   * - ``/tf`` topic in bag
     - ``/tf``
     - ``MOLA_TF_TOPIC`` (rosbag2 YAML ``tf_topic``)
   * - ``/tf_static`` topic in bag
     - ``/tf_static``
     - ``MOLA_TF_STATIC_TOPIC`` (rosbag2 YAML ``tf_static_topic``)

|

.. _mola_ros2_cookbook_online:

4. Live ROS 2 node
------------------------

All sections in this group use
``ros2 launch mola_lidar_odometry ros2-lidar-odometry.launch.py``. See
:ref:`all launch arguments <mola_lo_ros_launch_arguments>` for the full list.

.. _mola_ros2_cookbook_4_1:

4.1. Simple SE, LO or LIO, REP-105 (wheel / external odometry available)
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

Use this when the robot already publishes a high-frequency
``odom → base_link`` transform from wheel encoders (or any other odometry
source). MOLA-LO only corrects long-term drift by publishing ``map → odom``.

.. mermaid::

   flowchart LR
     subgraph ROS2[ROS 2 topics / tf]
       lidar[/"__LIDAR_TOPIC__"/]
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
        lidar[/"__LIDAR_TOPIC__"/] --> bridge[BridgeROS2]
        imu[/"__IMU_TOPIC__"/] --> bridge
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

4.2. Simple SE, no REP-105: LO or LIO, no external wheels odometry
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

When no external odometry exists (e.g. handheld mapping, drone), MOLA-LO
publishes ``map → base_link`` directly.

.. mermaid::

   flowchart LR
     lidar[/"__LIDAR_TOPIC__"/] --> bridge[BridgeROS2]
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
        lidar[/"__LIDAR_TOPIC__"/] --> bridge[BridgeROS2]
        imu[/"__IMU_TOPIC__"/] --> bridge
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

4.3. Namespaced robot (``/robot1/tf``, ``/robot1/scan``, …)
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

When your robot publishes everything under a ROS 2 namespace, the launch
file pushes the whole MOLA system into that namespace and remaps ``/tf`` /
``/tf_static`` so tf2 inside the namespace is transparent to MOLA.

.. mermaid::

   flowchart LR
     subgraph NS["namespace: __NS__"]
       lidar[/"__LIDAR_TOPIC__"/]
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
   ``/__NS__/tf`` without any extra configuration.

|

.. _mola_ros2_cookbook_4_4:

4.4. Smoother SE + external (wheel / VIO) odometry fusion
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

Adds an external odometry source to the smoother. The smoother estimates
a distinct ``T_map_to_odom`` per source and fuses their deltas with LO
(and IMU, if enabled).

.. important::

   With the Smoother, external odometry **must** be consumed via a
   ``nav_msgs/Odometry`` topic (``odom_topic_name:=...``).
   ``forward_ros_tf_odom_to_mola:=True`` (the /tf pathway) is only valid
   with the Simple estimator + REP-105 (§4.1), where the bridge splits
   its output into ``map → odom``. The Smoother instead publishes
   ``map → base_link`` directly, so an external ``odom → base_link`` on
   /tf would give ``base_link`` two parents and break the tf2 tree.
   The launch file enforces this and fails fast if both
   ``forward_ros_tf_odom_to_mola:=True`` and ``use_state_estimator:=True``
   are set.

The upstream driver publishes a ``nav_msgs/Odometry`` message on a topic.
BridgeROS2 subscribes directly, converts each message to a 3D
``CObservationRobotPose`` (6×6 covariance) and forwards it under
``odom_sensor_label``. This preserves Z / pitch / roll and allows fusing
**multiple** labeled sources.

.. mermaid::

   flowchart LR
     lidar[/"__LIDAR_TOPIC__"/] --> bridge[BridgeROS2]
     imu[/"__IMU_TOPIC__"/] --> bridge
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

.. dropdown:: Why not the /tf pathway here?
   :icon: question

   ``forward_ros_tf_odom_to_mola:=True`` makes BridgeROS2 query
   ``odom → base_link`` from /tf at its execution rate and inject a 2D
   ``CObservationOdometry``. That pathway **relies on an external
   publisher** (wheel driver, ``robot_state_publisher``) putting
   ``odom → base_link`` on /tf. With the Simple estimator in REP-105
   mode, the bridge publishes ``map → odom`` and the tree is well
   formed: ``map → odom → base_link``.

   With the Smoother, the bridge publishes ``map → base_link`` directly.
   Adding an external ``odom → base_link`` on top means two edges
   pointing at ``base_link`` (from ``map`` and from ``odom``), which
   violates the tf2 tree invariant. For Smoother fusion the only
   viable pathway is the ``/odom`` topic, whose observations do not
   participate in /tf.

.. list-table:: Odometry ingress — when to use which
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
     - **Simple SE + REP-105 only** (§4.1). Incompatible with Smoother —
       rejected by the launch file.
     - Simple SE (no REP-105) and **Smoother SE** (recommended).

See the :ref:`"Fusing two Odometry sources" tutorial <mola_sta_est_index>`
for a full worked example with fake publishers.

|

.. _mola_ros2_cookbook_4_5:

4.5. Smoother SE + live geo-referencing (GNSS)
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

The smoother estimates the ``enu → map`` transform online from incoming
GNSS fixes. It publishes ``enu`` and ``utm`` TFs once convergence is
reached.

.. mermaid::

   flowchart LR
     lidar[/"__LIDAR_TOPIC__"/] --> bridge[BridgeROS2]
     imu[/"__IMU_TOPIC__"/] --> bridge
     gnss[/"__GNSS_TOPIC__"/] --> bridge
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

4.6. Smoother SE + relocalization in a geo-referenced map
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

Load a previously built geo-referenced ``.mm`` map. MOLA-LO stays idle
until the smoother has converged on an initial pose from GNSS, then
switches to localization-only mapping.

.. mermaid::

   flowchart LR
     mm[(__MM_PATH__ .mm map)] --> lo
     lidar[/"__LIDAR_TOPIC__"/] --> bridge[BridgeROS2]
     imu[/"__IMU_TOPIC__"/] --> bridge
     gnss[/"__GNSS_TOPIC__"/] --> bridge
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

5. Offline rosbag2
------------------------

.. _mola_ros2_cookbook_5_1:

5.1. mola-lo-gui-rosbag2 (Simple SE, the default)
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

GUI replay of a bag. All configuration is via environment variables.

.. mermaid::

   flowchart LR
     bag[(__BAG_PATH__)] --> ds[Rosbag2Dataset]
     ds --> lo[mola::LidarOdometry]
     lo --> se[StateEstimationSimple]
     ds --> se
     se --> lo
     lo --> gui[MolaViz GUI]

.. tab-set::

    .. tab-item:: Lidar only (LO)

        .. container:: mola-tpl

          .. code-block:: bash

              MOLA_LIDAR_TOPIC=__LIDAR_TOPIC__ \
              MOLA_TF_BASE_LINK=__BASE_LINK__ \
                mola-lo-gui-rosbag2 __BAG_PATH__

    .. tab-item:: Lidar-Inertial (LIO)
      :selected:

        .. container:: mola-tpl

          .. code-block:: bash

              MOLA_DESKEW_METHOD=MotionCompensationMethod::IMU \
              MOLA_LO_INITIAL_LOCALIZATION_METHOD=InitLocalization::PitchAndRollFromIMU \
              MOLA_LIDAR_TOPIC=__LIDAR_TOPIC__ \
              MOLA_IMU_TOPIC=__IMU_TOPIC__ \
              MOLA_TF_BASE_LINK=__BASE_LINK__ \
                mola-lo-gui-rosbag2 __BAG_PATH__

.. dropdown:: Variant: bag recorded with a ROS 2 namespace
   :icon: alert

   When the bag was recorded from a namespaced robot, **both** the sensor
   topics and the ``/tf`` topics are prefixed with the namespace. Unlike
   the live ROS 2 node (§4.3), there is no tf2 remap at play here:
   Rosbag2Dataset looks up topics by literal name, so you must set them
   explicitly.

   Requires ``mola_input_rosbag2 ≥ 1.12`` (exposes ``tf_topic`` /
   ``tf_static_topic`` params). Edit the live form above so
   ``LiDAR topic`` (and ``IMU topic`` / ``GNSS topic`` if used) already
   carry the namespace prefix, e.g. ``/__NS__/ouster/points``:

   .. container:: mola-tpl

      .. code-block:: bash

         MOLA_LIDAR_TOPIC=__LIDAR_TOPIC__ \
         MOLA_IMU_TOPIC=__IMU_TOPIC__ \
         MOLA_TF_BASE_LINK=__BASE_LINK__ \
         MOLA_TF_TOPIC=/__NS__/tf \
         MOLA_TF_STATIC_TOPIC=/__NS__/tf_static \
           mola-lo-gui-rosbag2 __BAG_PATH__

   .. note::

      ``MOLA_TF_BASE_LINK`` is the **frame id** inside the TF messages
      (e.g. ``base_link``); it is **not** namespaced. Only the bag
      *topic names* carry the namespace.

|

.. _mola_ros2_cookbook_5_2:

5.2. mola-lo-gui-rosbag2 (Smoother SE)
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

Same as §5.1 but force the smoother and LIO-style deskew:

.. container:: mola-tpl

   .. code-block:: bash

      MOLA_LIDAR_TOPIC=__LIDAR_TOPIC__ \
      MOLA_IMU_TOPIC=__IMU_TOPIC__ \
      MOLA_TF_BASE_LINK=__BASE_LINK__ \
      MOLA_DESKEW_METHOD=MotionCompensationMethod::IMU \
      MOLA_STATE_ESTIMATOR=mola::state_estimation_smoother::StateEstimationSmoother \
      MOLA_STATE_ESTIMATOR_YAML="$(ros2 pkg prefix mola_state_estimation_smoother)/share/mola_state_estimation_smoother/params/state-estimation-smoother.yaml" \
        mola-lo-gui-rosbag2 __BAG_PATH__

.. dropdown:: Variant: bag recorded with a ROS 2 namespace
   :icon: alert

   Same caveat as §5.1: Rosbag2Dataset resolves topics by literal name,
   so namespaced bags need explicit ``MOLA_TF_TOPIC`` /
   ``MOLA_TF_STATIC_TOPIC``, and the sensor topics in the form above
   must already carry the namespace prefix (e.g.
   ``/__NS__/ouster/points``, ``/__NS__/imu``):

   .. container:: mola-tpl

      .. code-block:: bash

         MOLA_LIDAR_TOPIC=__LIDAR_TOPIC__ \
         MOLA_IMU_TOPIC=__IMU_TOPIC__ \
         MOLA_TF_BASE_LINK=__BASE_LINK__ \
         MOLA_TF_TOPIC=/__NS__/tf \
         MOLA_TF_STATIC_TOPIC=/__NS__/tf_static \
         MOLA_DESKEW_METHOD=MotionCompensationMethod::IMU \
         MOLA_STATE_ESTIMATOR=mola::state_estimation_smoother::StateEstimationSmoother \
         MOLA_STATE_ESTIMATOR_YAML="$(ros2 pkg prefix mola_state_estimation_smoother)/share/mola_state_estimation_smoother/params/state-estimation-smoother.yaml" \
           mola-lo-gui-rosbag2 __BAG_PATH__

|

.. _mola_ros2_cookbook_5_3:

5.3. mola-lidar-odometry-cli (fastest, no GUI)
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

Best option for large batch runs and reproducible benchmarks.

.. mermaid::

   flowchart LR
     bag[(__BAG_PATH__)] --> ds[Rosbag2Dataset]
     ds --> lo[mola::LidarOdometry]
     lo --> se[StateEstimationSimple / Smoother]
     se --> trj[[trajectory.tum]]
     se --> sm[[map.simplemap]]

.. container:: mola-tpl

   .. code-block:: bash

      mola-lidar-odometry-cli \
        -c $(ros2 pkg prefix mola_lidar_odometry)/share/mola_lidar_odometry/pipelines/lidar3d-default.yaml \
        --input-rosbag2 __BAG_PATH__ \
        --lidar-sensor-label __LIDAR_TOPIC__ \
        --base-link-frame-id __BASE_LINK__ \
        --output-tum-path trajectory.tum \
        --output-simplemap map.simplemap

.. dropdown:: Variant: bag with a ROS 2 namespace
   :icon: alert

   .. container:: mola-tpl

      .. code-block:: bash

         mola-lidar-odometry-cli \
           -c $(ros2 pkg prefix mola_lidar_odometry)/share/mola_lidar_odometry/pipelines/lidar3d-default.yaml \
           --input-rosbag2 __BAG_PATH__ \
           --lidar-sensor-label __LIDAR_TOPIC__ \
           --base-link-frame-id __BASE_LINK__ \
           --tf-topic /__NS__/tf \
           --tf-static-topic /__NS__/tf_static \
           --output-tum-path trajectory.tum

.. dropdown:: Variant: use the Smoother
   :icon: code-review

   .. container:: mola-tpl

      .. code-block:: bash

         mola-lidar-odometry-cli \
           -c $(ros2 pkg prefix mola_lidar_odometry)/share/mola_lidar_odometry/pipelines/lidar3d-default.yaml \
           --state-estimator "mola::state_estimation_smoother::StateEstimationSmoother" \
           --state-estimator-param-file $(ros2 pkg prefix mola_state_estimation_smoother)/share/mola_state_estimation_smoother/params/state-estimation-smoother.yaml \
           --load-plugins libmola_state_estimation_smoother.so \
           --input-rosbag2 __BAG_PATH__ \
           --lidar-sensor-label __LIDAR_TOPIC__ \
           --base-link-frame-id __BASE_LINK__ \
           --output-tum-path trajectory.tum

|

.. _mola_ros2_cookbook_fixes:

6. Troubleshooting
------------------------

.. list-table::
   :header-rows: 1
   :widths: 35 65

   * - Symptom
     - Most common cause
   * - ``"base_link" passed to lookupTransform ... does not exist``
     - The bag/driver is not publishing a ``base_link`` frame, or the
       launch ``mola_tf_base_link`` does not match the one in ``/tf``.
   * - LO starts but ``/tf`` is empty / map not visible in RViz2
     - Forgot to set ``publish_localization_following_rep105:=False`` when
       no external odometry is running. The launch waits for
       ``odom → base_link`` that never arrives.
   * - Smoother set, but ``map → odom`` appears (and not ``map →
       base_link``)
     - REP-105 is incompatible with the smoother. Set
       ``publish_localization_following_rep105:=False`` or simply do not
       set it (the launch default already matches when
       ``use_state_estimator:=True``).
   * - Namespaced bag silently ignored on ``/tf``
     - Before ``mola_input_rosbag2`` gained ``tf_topic`` / ``tf_static_topic``
       params, ``/tf`` was hardcoded. Update the package and set
       ``MOLA_TF_TOPIC`` / ``MOLA_TF_STATIC_TOPIC`` (see §5.1).
   * - Smoother converges slowly to geo-reference
     - The vehicle must move non-trivially for GNSS factors to constrain
       ``enu → map``. Drive at least a few meters.
