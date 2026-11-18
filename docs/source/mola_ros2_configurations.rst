.. _mola_ros2_cookbook:

========================
ROS 2 configurations
========================

Copy-paste recipes to run MOLA LiDAR odometry (MOLA-LO / MOLA-LIO) with ROS 2:
on a live robot, on your own bags, and on public datasets. The most common
cases come first; special cases are at the end.

There are two ways to feed data to MOLA:

- **Live**: ``ros2 launch mola_lidar_odometry ros2-lidar-odometry.launch.py``
  subscribes to ROS 2 topics and publishes ``/tf``, odometry and map topics.
  It is configured with launch arguments (``name:=value``).
- **Offline**: ``mola-lo-gui-rosbag2`` (with GUI) or ``mola-lo-cli-rosbag2``
  (batch, as fast as possible) read a bag file directly, without playing it
  and without ROS 2 communication. They are configured with environment
  variables (``MOLA_*``). See :ref:`gui_vs_cli` to choose between them.

All the knobs used below, with both spellings, are listed in
:ref:`the parameter reference <mola_ros2_cookbook_axes>` at the end.

.. _mola_ros2_cookbook_form:

.. admonition:: Your topic names, frames and paths

   .. raw:: html

      <div id="mola-cookbook-form" class="mola-cookbook-form">
        <p><em>Loading editor... if this message stays, JavaScript is disabled
        and the commands below show placeholder values.</em></p>
      </div>

|

.. _mola_ros2_cookbook_decision:

Pick a recipe
-------------

.. list-table::
   :header-rows: 1
   :widths: 50 50

   * - Your situation
     - Recipe
   * - Live robot, no wheel odometry (handheld, drone, legged robot, ...)
     - :ref:`mola_ros2_cookbook_4_2`
   * - Live robot whose driver publishes ``odom → base_link`` on ``/tf``
     - :ref:`mola_ros2_cookbook_4_1`
   * - Fuse LiDAR odometry with an IMU, wheel odometry or GNSS in a smoother
     - :ref:`mola_ros2_cookbook_smoother`, :ref:`mola_ros2_cookbook_4_4`
   * - No ``/tf`` for the sensors: you give their poses by hand
     - :ref:`mola_ros2_cookbook_no_tf`
   * - Topics and ``/tf`` inside a namespace (``/robot1/tf``, ...)
     - :ref:`mola_ros2_cookbook_4_3`
   * - Build a map from your own ROS 2 bag
     - :ref:`mola_ros2_cookbook_5_1`
   * - Run a public dataset (KITTI, MulRan, VBR, ...)
     - :ref:`mola_ros2_cookbook_datasets`
   * - Globally consistent map with loop closure
     - :ref:`mola_ros2_cookbook_mapper`
   * - What is published, and how to save the map and trajectory
     - :ref:`mola_ros2_cookbook_outputs`
   * - GNSS: geo-referenced maps, live geo-referencing, relocalization
     - :ref:`mola_ros2_cookbook_gnss`

|

.. _mola_ros2_cookbook_online:

Live robot (ROS 2 topics)
-------------------------

All recipes in this section use the same launch file. Its full list of
arguments is in :ref:`the ROS 2 node reference <mola_lo_ros_launch_arguments>`.
``lidar_topic_name`` is the only mandatory argument.

**LO or LIO?** Each recipe comes in two flavors:

- **LO** (LiDAR odometry): only the LiDAR is needed. Scans are de-skewed with a
  constant velocity model.
- **LIO** (LiDAR-inertial odometry): add ``imu_topic_name:=...`` and
  ``use_imu_for_lio:=True`` to de-skew scans with the IMU. Prefer it for
  handheld, legged or fast-rotating platforms. Giving ``imu_topic_name``
  without ``use_imu_for_lio`` still uses the IMU accelerometer to keep pitch
  and roll aligned with gravity, but not for de-skew.

.. _mola_ros2_cookbook_4_2:

LiDAR only, no wheel odometry
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

**When to use:** the most common case. Nothing else publishes the robot
motion, so MOLA publishes ``map → base_link`` directly. The LiDAR (and IMU)
poses on the robot are read from ``/tf``.

.. tab-set::

   .. tab-item:: LO
      :sync: lo

      .. container:: mola-tpl

         .. code-block:: bash

            ros2 launch mola_lidar_odometry ros2-lidar-odometry.launch.py \
              lidar_topic_name:=__LIDAR_TOPIC__ \
              mola_tf_base_link:=__BASE_LINK__ \
              publish_localization_following_rep105:=False

   .. tab-item:: LIO (with IMU)
      :sync: lio
      :selected:

      .. container:: mola-tpl

         .. code-block:: bash

            ros2 launch mola_lidar_odometry ros2-lidar-odometry.launch.py \
              lidar_topic_name:=__LIDAR_TOPIC__ \
              imu_topic_name:=__IMU_TOPIC__ \
              use_imu_for_lio:=True \
              mola_tf_base_link:=__BASE_LINK__ \
              publish_localization_following_rep105:=False

Resulting ``/tf`` tree::

   map ──> base_link ──> (sensor frames, from your robot description)

.. important::

   ``publish_localization_following_rep105`` defaults to ``True``. Without
   ``:=False`` and without any ``odom → base_link`` on ``/tf``, nothing is
   published to ``/tf``: the bridge waits for an odometry transform that never
   arrives.

|

.. _mola_ros2_cookbook_4_1:

Wheel odometry on ``/tf`` (REP-105)
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

**When to use:** a ground robot whose driver already publishes a high-rate
``odom → base_link`` on ``/tf`` (wheel encoders, VIO, ...). Following
`REP-105 <https://www.ros.org/reps/rep-0105.html>`_, MOLA only publishes the
drift correction ``map → odom``. ``forward_ros_tf_odom_to_mola:=True`` also
feeds that odometry to MOLA as a motion prior between scans (optional).

.. tab-set::

   .. tab-item:: LO
      :sync: lo

      .. container:: mola-tpl

         .. code-block:: bash

            ros2 launch mola_lidar_odometry ros2-lidar-odometry.launch.py \
              lidar_topic_name:=__LIDAR_TOPIC__ \
              mola_tf_base_link:=__BASE_LINK__ \
              publish_localization_following_rep105:=True \
              forward_ros_tf_odom_to_mola:=True

   .. tab-item:: LIO (with IMU)
      :sync: lio
      :selected:

      .. container:: mola-tpl

         .. code-block:: bash

            ros2 launch mola_lidar_odometry ros2-lidar-odometry.launch.py \
              lidar_topic_name:=__LIDAR_TOPIC__ \
              imu_topic_name:=__IMU_TOPIC__ \
              use_imu_for_lio:=True \
              mola_tf_base_link:=__BASE_LINK__ \
              publish_localization_following_rep105:=True \
              forward_ros_tf_odom_to_mola:=True

Resulting ``/tf`` tree::

   map ──> odom ──> base_link ──> (sensor frames)
       ^        ^
       |        └── your odometry driver
       └── MOLA (drift correction)

If your odometry frame is not called ``odom``, add
``mola_bridge_odometry_frame:=<name>``. ``forward_ros_tf_odom_to_mola`` feeds a
planar odometry, and cannot be combined with ``odom_topic_name`` nor with the
smoother (the launch file refuses both): for those, use the
:ref:`smoother recipe <mola_ros2_cookbook_4_4>`.

|

.. _mola_ros2_cookbook_smoother:

State estimator: simple or smoother
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

MOLA-LO asks a :ref:`state estimator <mola_sta_est_index>` for the motion
prediction between scans. Two are available:

- **Simple** (``mola_state_estimation_simple``, the default): a light
  constant-velocity filter, optionally propagated with the IMU. ``/tf`` and the
  odometry topic come directly from the LiDAR odometry. Use it unless you need
  the smoother.
- **Smoother** (``mola_state_estimation_smoother``, ``use_state_estimator:=True``):
  a sliding-window factor graph that fuses LiDAR odometry with IMU, wheel or
  visual odometry and GNSS. ``/tf`` and the odometry topic then come from the
  smoother. Needed for GNSS geo-referencing and for
  :ref:`fusing external odometry <mola_ros2_cookbook_4_4>`. It is a separate
  package (``ros-$ROS_DISTRO-mola-state-estimation-smoother``).

**When to use the smoother:** you have more sensors than a LiDAR and an IMU to
fuse, or you want :ref:`GNSS <mola_ros2_cookbook_gnss>` geo-referencing.

.. container:: mola-tpl

   .. code-block:: bash

      # LIO + smoother (no external odometry): map -> base_link
      ros2 launch mola_lidar_odometry ros2-lidar-odometry.launch.py \
        lidar_topic_name:=__LIDAR_TOPIC__ \
        imu_topic_name:=__IMU_TOPIC__ \
        use_imu_for_lio:=True \
        mola_tf_base_link:=__BASE_LINK__ \
        use_state_estimator:=True \
        publish_localization_following_rep105:=False

The smoother parameters (``state-estimation-smoother.yaml``) can be tuned with
their environment variables, or replaced with
``state_estimator_config_yaml:=/path/to/your.yaml``. See
:ref:`mola_sta_est_smoother`.

|

.. _mola_ros2_cookbook_4_4:

Smoother + external odometry (wheels, VIO)
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

**When to use:** you have a ``nav_msgs/Odometry`` topic (wheel encoders, visual
or leg odometry) and want it fused with the LiDAR odometry and IMU. The
external odometry enters as a topic (``odom_topic_name``), never through
``/tf``, and the smoother estimates its drift with respect to ``map``.

For wheel odometry, also export ``MOLA_ODOM_AS_ROBOT_POSE_OBS=false``: the
odometry is then fused as planar odometry with the smoother's wheel motion
model (errors that grow with distance and rotation), instead of with the tight
sigmas meant for LiDAR or visual odometry. When an odometry topic is given, the
launch file already loosens those odometry sigmas and turns IMU preintegration
off, unless you set them yourself.

.. tab-set::

   .. tab-item:: Driver does not publish odom → base_link

      .. container:: mola-tpl

         .. code-block:: bash

            MOLA_ODOM_AS_ROBOT_POSE_OBS=false \
            ros2 launch mola_lidar_odometry ros2-lidar-odometry.launch.py \
              lidar_topic_name:=__LIDAR_TOPIC__ \
              imu_topic_name:=__IMU_TOPIC__ \
              use_imu_for_lio:=True \
              mola_tf_base_link:=__BASE_LINK__ \
              use_state_estimator:=True \
              odom_topic_name:=__ODOM_TOPIC__ \
              publish_localization_following_rep105:=False

      Resulting ``/tf`` tree: ``map → base_link``, from the smoother.

   .. tab-item:: Driver also publishes odom → base_link (REP-105)

      Here ``base_link`` already has a parent (``odom``), so the smoother must
      publish ``map → odom``. It is taken from the smoother's own estimate of
      the odometry frame (``MOLA_PUBLISH_MAP_TO_ODOM_TF``), selected by the
      odometry label (``odom_sensor_label``, default ``odom_wheels``) and
      published under your ``odom`` frame name:

      .. container:: mola-tpl

         .. code-block:: bash

            MOLA_ODOM_AS_ROBOT_POSE_OBS=false \
            MOLA_PUBLISH_MAP_TO_ODOM_TF=true \
            MOLA_MAP_TO_ODOM_FRAME=odom_wheels \
            MOLA_MAP_TO_ODOM_CHILD_FRAME=odom \
            ros2 launch mola_lidar_odometry ros2-lidar-odometry.launch.py \
              lidar_topic_name:=__LIDAR_TOPIC__ \
              imu_topic_name:=__IMU_TOPIC__ \
              use_imu_for_lio:=True \
              mola_tf_base_link:=__BASE_LINK__ \
              use_state_estimator:=True \
              odom_topic_name:=__ODOM_TOPIC__ \
              localization_publish_tf_source:=state_estimation/map_odom

      Resulting ``/tf`` tree: ``map → odom → base_link``. A complete example
      with GNSS is in :ref:`localization-lo_georef_gnss_imu_kinematics`.

To fuse **several** odometry sources (e.g. wheels and VIO), add one
``subscribe`` entry per source to ``mola-cli-launchs/lidar_odometry_ros2.yaml``
with ``msg_type: Odometry`` and a distinct ``output_sensor_label``. Each entry
accepts its own ``odometry_as_robot_pose_observation: true|false`` (see
:ref:`ros2api_subscribed_topics`). The
:ref:`simulated sensors demo <mola_sta_est_try_it>` shows two odometries fused
by the smoother, without any robot.

|

.. _mola_ros2_cookbook_no_tf:

No ``/tf``: sensor poses given by hand
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

**When to use:** nothing publishes ``base_link → lidar`` (no
``robot_state_publisher``, or a bag recorded without ``/tf``). Each sensor pose
on the robot is then given with environment variables: ``x y z`` in meters and
``yaw pitch roll`` in degrees, all ``0`` by default (sensor at ``base_link``).
Only the axes that are not zero need to be given.

.. list-table::
   :header-rows: 1
   :widths: 20 40 40

   * - Sensor
     - Ignore ``/tf`` (live / offline)
     - Pose variables
   * - LiDAR
     - ``ignore_lidar_pose_from_tf:=True`` / ``MOLA_USE_FIXED_LIDAR_POSE=true``
     - ``LIDAR_POSE_X``, ``_Y``, ``_Z``, ``_YAW``, ``_PITCH``, ``_ROLL``
   * - IMU
     - ``ignore_imu_pose_from_tf:=True`` / ``MOLA_USE_FIXED_IMU_POSE=true``
     - ``IMU_POSE_X``, ... ``_ROLL``
   * - GNSS
     - ``MOLA_USE_FIXED_GNSS_POSE=true`` (both)
     - ``GNSS_POSE_X``, ... ``_ROLL``

.. tab-set::

   .. tab-item:: Live
      :sync: live

      .. container:: mola-tpl

         .. code-block:: bash

            # Example: LiDAR 0.30 m above base_link, IMU rotated 90 deg in yaw
            LIDAR_POSE_Z=0.30 \
            IMU_POSE_YAW=90 \
            ros2 launch mola_lidar_odometry ros2-lidar-odometry.launch.py \
              lidar_topic_name:=__LIDAR_TOPIC__ \
              imu_topic_name:=__IMU_TOPIC__ \
              use_imu_for_lio:=True \
              ignore_lidar_pose_from_tf:=True \
              ignore_imu_pose_from_tf:=True \
              mola_tf_base_link:=__BASE_LINK__ \
              publish_localization_following_rep105:=False

   .. tab-item:: Offline (bag)
      :sync: offline

      .. container:: mola-tpl

         .. code-block:: bash

            MOLA_USE_FIXED_LIDAR_POSE=true LIDAR_POSE_Z=0.30 \
            MOLA_USE_FIXED_IMU_POSE=true IMU_POSE_YAW=90 \
            MOLA_DESKEW_METHOD=MotionCompensationMethod::IMU \
            MOLA_LIDAR_TOPIC=__LIDAR_TOPIC__ \
            MOLA_IMU_TOPIC=__IMU_TOPIC__ \
              mola-lo-gui-rosbag2 __BAG_PATH__

``mola_tf_base_link`` (``MOLA_TF_BASE_LINK``) is then only the name of the robot
frame in MOLA outputs. The :ref:`mapper launchers <mola_ros2_cookbook_mapper>`
do not support fixed sensor poses: they need ``/tf`` in the bag.

|

.. _mola_ros2_cookbook_4_3:

Namespaced robot (``/robot1/tf``, ``/robot1/...``)
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

**When to use:** the robot publishes its topics and its ``/tf`` /
``/tf_static`` inside a ROS 2 namespace, e.g. in multi-robot systems.

**Live:** ``use_namespace:=True`` pushes the whole MOLA node (and RViz) into the
namespace and remaps ``/tf`` and ``/tf_static`` to ``/<namespace>/tf`` and
``/<namespace>/tf_static``. Give topic names **relative** (no leading ``/``) so
the namespace is prepended to them; absolute names are used as they are.

.. container:: mola-tpl

   .. code-block:: bash

      # Reads /__NS__/ouster/points, /__NS__/imu, /__NS__/tf, /__NS__/tf_static
      ros2 launch mola_lidar_odometry ros2-lidar-odometry.launch.py \
        use_namespace:=True \
        namespace:=__NS__ \
        lidar_topic_name:=ouster/points \
        imu_topic_name:=imu \
        use_imu_for_lio:=True \
        mola_tf_base_link:=__BASE_LINK__ \
        publish_localization_following_rep105:=False

If only the sensor topics are namespaced and ``/tf`` is global, add
``use_namespaced_tf:=False``. Frame ids are not namespaced by ROS: if your
robot prefixes them (e.g. ``robot1/base_link``), give that name to
``mola_tf_base_link``.

**Offline:** a bag stores topics by their full name, so give the sensor topics
with the namespace, and the two ``/tf`` topics explicitly. Frame ids, again, are
used as they appear in the ``/tf`` messages:

.. container:: mola-tpl

   .. code-block:: bash

      MOLA_TF_TOPIC=/__NS__/tf \
      MOLA_TF_STATIC_TOPIC=/__NS__/tf_static \
      MOLA_LIDAR_TOPIC=/__NS__/ouster/points \
      MOLA_TF_BASE_LINK=__BASE_LINK__ \
        mola-lo-gui-rosbag2 __BAG_PATH__

|

.. _mola_ros2_cookbook_offline:

Offline: bags and datasets
--------------------------

.. _mola_ros2_cookbook_5_1:
.. _mola_ros2_cookbook_5_2:
.. _mola_ros2_cookbook_5_3:

Your own ROS 2 bag
~~~~~~~~~~~~~~~~~~

**When to use:** to build a map or to evaluate odometry from a recorded
``.mcap`` / ``.db3`` bag (or a directory with its ``metadata.yaml``). Reading
the bag directly is faster and more reproducible than replaying it into the
live node. Two wrappers take the same environment variables:

- ``mola-lo-gui-rosbag2``: replay with the MolaViz GUI, to watch and control
  the run.
- ``mola-lo-cli-rosbag2``: no GUI, as fast as the CPU allows. Arguments after
  the bag path go to ``mola-lidar-odometry-cli`` (run it with ``--help`` for all
  output options).

Run either wrapper without arguments to print its usage and variables.

.. tab-set::

   .. tab-item:: GUI: mola-lo-gui-rosbag2
      :sync: gui

      .. tab-set::

         .. tab-item:: LO
            :sync: lo

            .. container:: mola-tpl

               .. code-block:: bash

                  MOLA_LIDAR_TOPIC=__LIDAR_TOPIC__ \
                  MOLA_TF_BASE_LINK=__BASE_LINK__ \
                    mola-lo-gui-rosbag2 __BAG_PATH__

         .. tab-item:: LIO
            :sync: lio
            :selected:

            .. container:: mola-tpl

               .. code-block:: bash

                  MOLA_DESKEW_METHOD=MotionCompensationMethod::IMU \
                  MOLA_LIDAR_TOPIC=__LIDAR_TOPIC__ \
                  MOLA_IMU_TOPIC=__IMU_TOPIC__ \
                  MOLA_TF_BASE_LINK=__BASE_LINK__ \
                    mola-lo-gui-rosbag2 __BAG_PATH__

         .. tab-item:: LIO + smoother
            :sync: smoother

            .. container:: mola-tpl

               .. code-block:: bash

                  MOLA_STATE_ESTIMATOR_YAML=$(ros2 pkg prefix mola_state_estimation_smoother)/share/mola_state_estimation_smoother/params/state-estimation-smoother.yaml \
                  MOLA_LINK_FIRST_POSE_SIGMA=1e-6 \
                  MOLA_DESKEW_METHOD=MotionCompensationMethod::IMU \
                  MOLA_LIDAR_TOPIC=__LIDAR_TOPIC__ \
                  MOLA_IMU_TOPIC=__IMU_TOPIC__ \
                  MOLA_TF_BASE_LINK=__BASE_LINK__ \
                    mola-lo-gui-rosbag2 __BAG_PATH__

   .. tab-item:: Batch: mola-lo-cli-rosbag2
      :sync: cli

      .. tab-set::

         .. tab-item:: LO
            :sync: lo

            .. container:: mola-tpl

               .. code-block:: bash

                  MOLA_LIDAR_TOPIC=__LIDAR_TOPIC__ \
                  MOLA_TF_BASE_LINK=__BASE_LINK__ \
                    mola-lo-cli-rosbag2 __BAG_PATH__ \
                      --output-tum-path trajectory.tum \
                      --output-simplemap map.simplemap

         .. tab-item:: LIO
            :sync: lio
            :selected:

            .. container:: mola-tpl

               .. code-block:: bash

                  MOLA_DESKEW_METHOD=MotionCompensationMethod::IMU \
                  MOLA_LIDAR_TOPIC=__LIDAR_TOPIC__ \
                  MOLA_IMU_TOPIC=__IMU_TOPIC__ \
                  MOLA_TF_BASE_LINK=__BASE_LINK__ \
                    mola-lo-cli-rosbag2 __BAG_PATH__ \
                      --output-tum-path trajectory.tum \
                      --output-simplemap map.simplemap

         .. tab-item:: LIO + smoother
            :sync: smoother

            .. container:: mola-tpl

               .. code-block:: bash

                  MOLA_STATE_ESTIMATOR_YAML=$(ros2 pkg prefix mola_state_estimation_smoother)/share/mola_state_estimation_smoother/params/state-estimation-smoother.yaml \
                  MOLA_LINK_FIRST_POSE_SIGMA=1e-6 \
                  MOLA_DESKEW_METHOD=MotionCompensationMethod::IMU \
                  MOLA_LIDAR_TOPIC=__LIDAR_TOPIC__ \
                  MOLA_IMU_TOPIC=__IMU_TOPIC__ \
                  MOLA_TF_BASE_LINK=__BASE_LINK__ \
                    mola-lo-cli-rosbag2 __BAG_PATH__ \
                      --output-tum-path trajectory.tum \
                      --output-simplemap map.simplemap

The estimator class follows ``MOLA_STATE_ESTIMATOR_YAML`` (the default is the
simple one). ``MOLA_LINK_FIRST_POSE_SIGMA=1e-6`` pins the first pose of the
smoother to the map origin, as the dataset launchers do when they select it.
The GUI wrapper saves files only if asked to: see
:ref:`mola_ros2_cookbook_outputs`. For a bag without ``/tf``, see
:ref:`mola_ros2_cookbook_no_tf`; for a namespaced bag,
:ref:`mola_ros2_cookbook_4_3`. More details:
:ref:`the rosbag2 launcher <mola_lo_gui_rosbag2>`.

|

.. _mola_ros2_cookbook_datasets:

Public datasets
~~~~~~~~~~~~~~~

**When to use:** to run a supported public dataset without working out its
topics, frames, sensor poses or tuning. Each dataset has a
``mola-lo-gui-<dataset>`` and a ``mola-lo-cli-<dataset>`` launcher (see the
:ref:`list of datasets <supported_datasets>`); both read the same dataset
profile, so they run the same configuration:

.. code-block:: bash

   # GUI replay of one sequence:
   mola-lo-gui-mulran KAIST01

   # The same sequence, offline, saving the trajectory:
   mola-lo-cli-mulran KAIST01 --output-tum-path mulran-kaist01.tum

Each launcher prints its usage and dataset-specific variables when run without
arguments. Any ``MOLA_*`` variable you export takes precedence over the
profile's value, with one catch: **an empty value is ignored** (the profile
uses its default), so to turn something off, set another non-empty value.
For example, LO instead of LIO on a dataset whose profile enables IMU de-skew:

.. code-block:: bash

   MOLA_DESKEW_METHOD=MotionCompensationMethod::Linear mola-lo-gui-mulran KAIST01

Some profiles select the smoother. To use the simple estimator instead, point
``MOLA_STATE_ESTIMATOR_YAML`` to its parameters:
``$(ros2 pkg prefix mola_lidar_odometry)/share/mola_lidar_odometry/state-estimator-params/state-estimation-simple.yaml``.

|

.. _mola_ros2_cookbook_mapper:

Global map with loop closure (mola_mapper)
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

**When to use:** large or looping trajectories where odometry drift becomes
visible, and you want a globally consistent map. :ref:`mola_mapper` adds
loop closure and a global pose graph (with GNSS, gravity and gyroscope factors)
on top of LIO and the smoother. It runs on recorded data only: its launch files
read a bag or dataset and publish nothing to ROS 2. Results are written to
files when the run ends.

.. container:: mola-tpl

   .. code-block:: bash

      # Your own ROS 2 bag (needs /tf in the bag):
      MOLA_DESKEW_METHOD=MotionCompensationMethod::IMU \
      MOLA_LIDAR_TOPIC=__LIDAR_TOPIC__ \
      MOLA_IMU_TOPIC=__IMU_TOPIC__ \
      MOLA_GNSS_TOPIC=__GNSS_TOPIC__ \
      MOLA_TF_BASE_LINK=__BASE_LINK__ \
      MOLA_MAPPER_OUTPUT_TUM=mapper.tum \
      MOLA_MAPPER_OUTPUT_SIMPLEMAP=mapper.simplemap \
      MOLA_MAPPER_OUTPUT_GEOREF=mapper-georef.yaml \
        mola-mapper-gui-rosbag2 __BAG_PATH__

      # A public dataset with a profile (same arguments as its mola-lo-gui-* launcher):
      mola-mapper-gui-vbr /path/to/vbr/colosseo_train0/

``MOLA_GNSS_TOPIC`` is optional. For a headless run, add ``MOLA_WITH_GUI=false
MOLA_QUIT_ON_DATASET_END=true``. Mapper launchers exist for the datasets listed
in :ref:`mola_mapper_datasets`, and its parameters are in
:ref:`mola_mapper_params`.

Alternatively, loop closure can be run afterwards on a ``.simplemap`` built by
any recipe above, with ``mola-sm-lc-cli``: see
:ref:`tutorial-ouster-mapping-lc`.

|

.. _mola_ros2_cookbook_outputs:

Results: ``/tf``, topics and saved files
----------------------------------------

**Live node.** What is published depends on the estimator and on REP-105:

.. list-table::
   :header-rows: 1
   :widths: 34 33 33

   * -
     - Simple estimator
     - Smoother (``use_state_estimator:=True``)
   * - ``/tf``, with ``publish_localization_following_rep105:=False``
     - ``map → base_link``
     - ``map → base_link``
   * - ``/tf``, REP-105 (driver publishes ``odom → base_link``)
     - ``map → odom``
     - ``map → odom`` (see :ref:`mola_ros2_cookbook_4_4`)
   * - Pose (``nav_msgs/Odometry``, in ``map``)
     - ``/lidar_odometry/pose``
     - ``/state_estimation/pose``
   * - Local map (``sensor_msgs/PointCloud2``)
     - ``/lidar_odometry/localmap_points``
     - ``/lidar_odometry/localmap_points``

Also published: ``/lidar_odometry/pose_quality`` (ICP quality, 0 to 1),
``/diagnostics``, and, once the map is geo-referenced, ``enu → map`` and
``enu → utm`` on ``/tf`` plus a ``geo_pose`` and a ``geo_ref_metadata`` topic
(see :ref:`mola_ros2_cookbook_gnss`). Inside a namespace, topics and services
are under it, except ``/diagnostics`` and the ``/initialpose`` subscription.
Disable the local map topic with ``publish_local_map:=False``, and the GUIs
with ``use_mola_gui:=False use_rviz:=False``. Full reference: :ref:`mola_ros2api`
and :ref:`mola_ros2_tf_frames`.

**Saving the map and trajectory:**

.. list-table::
   :header-rows: 1
   :widths: 25 75

   * - Tool
     - How
   * - Live node
     - ``generate_simplemap:=True`` keeps keyframes and writes them when the node
       shuts down, to ``MOLA_SIMPLEMAP_OUTPUT`` (default ``final_map.simplemap``
       in the working directory). ``MOLA_SAVE_TRAJECTORY=true`` (and
       ``MOLA_TUM_TRAJECTORY_OUTPUT``) saves the trajectory in TUM format. The
       map can also be saved at any time with the ``/map_save`` service (see
       :ref:`map_loading_saving`).
   * - ``mola-lo-gui-*``
     - Same variables: ``MOLA_GENERATE_SIMPLEMAP=true MOLA_SIMPLEMAP_OUTPUT=map.simplemap``,
       ``MOLA_SAVE_TRAJECTORY=true MOLA_TUM_TRAJECTORY_OUTPUT=trajectory.tum``.
       Files are written when you close the GUI (or use its "save" buttons).
   * - ``mola-lo-cli-*``
     - ``--output-simplemap map.simplemap``, ``--output-tum-path trajectory.tum``.

A ``.simplemap`` is turned into a metric map (``.mm``) for localization with
``sm2mm``: see :ref:`building-maps`.

|

.. _mola_ros2_cookbook_gnss:

GNSS and geo-referencing
------------------------

Three ways of using a GNSS receiver (``sensor_msgs/NavSatFix`` on
``gnss_topic_name``, or ``gps_msgs/GPSFix`` on ``gpsfix_topic_name``), selected
live with ``gnss_mode``. The antenna position on the robot comes from ``/tf``
(or see :ref:`mola_ros2_cookbook_no_tf`). Background: :ref:`geo-referencing`.

.. _mola_ros2_cookbook_5_4:

Record a map to geo-reference it afterwards
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

**When to use:** to build a geo-referenced ``.mm`` map, for later localization
in it. The GNSS does not change the odometry: each keyframe of the
``.simplemap`` stores the closest GNSS fix and IMU reading, and
``mola-sm-georeferencing`` computes the geo-reference afterwards (see
:ref:`geo-referencing`).

The IMU reading gives the azimuth, so it must hold an **absolute orientation**:
the IMU built into most LiDARs is fine for de-skew but has none. If the robot
has a separate AHRS (or a dual-antenna GNSS publishing a ``sensor_msgs/Imu``
attitude), name it as the attitude IMU topic: its readings are then the ones
stored into keyframes, while the main IMU keeps doing de-skew and gravity
alignment. Without it, keyframes store readings from the main IMU, which is
right when that IMU already reports an absolute orientation.

.. tab-set::

   .. tab-item:: Offline: mola-lo-cli-rosbag2
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

   .. tab-item:: Offline: mola-lo-gui-rosbag2
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

   .. tab-item:: Live
      :sync: live

      .. container:: mola-tpl

         .. code-block:: bash

            # gnss_mode:=log_only also enables generate_simplemap.
            # The map is written when the node shuts down.
            MOLA_SIMPLEMAP_OUTPUT=map.simplemap \
            ros2 launch mola_lidar_odometry ros2-lidar-odometry.launch.py \
              lidar_topic_name:=__LIDAR_TOPIC__ \
              imu_topic_name:=__IMU_TOPIC__ \
              use_imu_for_lio:=True \
              gnss_topic_name:=__GNSS_TOPIC__ \
              attitude_imu_topic_name:=__ATTITUDE_IMU_TOPIC__ \
              mola_tf_base_link:=__BASE_LINK__ \
              publish_localization_following_rep105:=False \
              gnss_mode:=log_only

Drop the attitude IMU line if you do not have one. The attitude IMU also
reaches the state estimator: with the smoother, export
``MOLA_NAVSTATE_IMU_SENSOR_NAME=imu`` (a regex of the IMU labels it uses) unless
fusing its absolute attitude is intended.

|

.. _mola_ros2_cookbook_4_5:

Live geo-referencing (smoother)
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

**When to use:** no prior map, but you want poses in geodetic or UTM
coordinates while the robot moves. The smoother estimates the ``enu → map``
transform online from the GNSS fixes and, once converged, publishes
``enu → map`` and ``enu → utm`` on ``/tf``, the geo-reference on
``/state_estimator/geo_ref_metadata``, and the vehicle latitude, longitude and
height on ``/state_estimation/geo_pose``. The robot must move a few meters
before the azimuth converges, unless an attitude IMU gives it.

.. container:: mola-tpl

   .. code-block:: bash

      ros2 launch mola_lidar_odometry ros2-lidar-odometry.launch.py \
        lidar_topic_name:=__LIDAR_TOPIC__ \
        imu_topic_name:=__IMU_TOPIC__ \
        use_imu_for_lio:=True \
        gnss_topic_name:=__GNSS_TOPIC__ \
        mola_tf_base_link:=__BASE_LINK__ \
        use_state_estimator:=True \
        publish_localization_following_rep105:=False \
        gnss_mode:=live_georef

A complete setup (wheel odometry, AHRS, GNSS covariances, expected accuracy) is
described in :ref:`localization-lo_georef_gnss_imu_kinematics`. Offline, the
same estimation is enabled with ``MOLA_ESTIMATE_GEO_REF=true`` on top of the
:ref:`smoother recipe <mola_ros2_cookbook_5_1>`.

|

.. _mola_ros2_cookbook_4_6:

Relocalize with GNSS in a geo-referenced map
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

**When to use:** localization only, in a geo-referenced ``.mm`` map built
before, starting anywhere in it. MOLA-LO waits until the smoother has a
confident pose from GNSS, starts from it, and does not modify the map
(``start_mapping_enabled:=False``).

.. container:: mola-tpl

   .. code-block:: bash

      ros2 launch mola_lidar_odometry ros2-lidar-odometry.launch.py \
        lidar_topic_name:=__LIDAR_TOPIC__ \
        imu_topic_name:=__IMU_TOPIC__ \
        use_imu_for_lio:=True \
        gnss_topic_name:=__GNSS_TOPIC__ \
        mola_tf_base_link:=__BASE_LINK__ \
        mola_initial_map_mm_file:=__MM_PATH__ \
        start_mapping_enabled:=False \
        use_state_estimator:=True \
        publish_localization_following_rep105:=False \
        gnss_mode:=relocalize

With a receiver without RTK, the GNSS pose may be too uncertain to start ICP
from it: enable the search around it with ``MOLA_LO_INIT_GLOBAL_SEARCH=true``
(see the initial localization variables in :ref:`mola_lo_pipelines`). To start
from a known pose instead of GNSS, drop ``gnss_mode`` and give
``initial_pose:="[x, y, z, yaw_deg, pitch_deg, roll_deg]"``. More on
localization: :ref:`localization-only` and
:ref:`tutorial-mola-lo-map-and-localize`.

|

.. _mola_ros2_cookbook_advanced:

Special cases
-------------

.. dropdown:: Replay a bag through the live node (simulated time)
   :icon: play

   To test the live node with a bag, enable simulated time in MOLA and publish
   the bag clock. Both are required together:

   .. container:: mola-tpl

      .. code-block:: bash

         # Terminal 1:
         ros2 launch mola_lidar_odometry ros2-lidar-odometry.launch.py \
           lidar_topic_name:=__LIDAR_TOPIC__ \
           mola_tf_base_link:=__BASE_LINK__ \
           publish_localization_following_rep105:=False \
           use_sim_time:=true

         # Terminal 2:
         ros2 bag play __BAG_PATH__ --clock

   To just process a bag, the :ref:`offline wrappers <mola_ros2_cookbook_5_1>`
   are faster and reproducible.

.. dropdown:: 2D LiDAR (``sensor_msgs/LaserScan``)
   :icon: dot

   Select the topic type and the 2D pipeline:

   .. code-block:: bash

      ros2 launch mola_lidar_odometry ros2-lidar-odometry.launch.py \
        lidar_topic_name:=/scan \
        lidar_topic_type:=LaserScan \
        mola_lo_pipeline:=../pipelines/lidar2d.yaml

   Add ``publish_localization_following_rep105:=False`` if there is no wheel
   odometry on ``/tf``.

.. dropdown:: Other frame names (map, odom, base_link)
   :icon: dot

   - Robot frame: ``mola_tf_base_link:=<name>``.
   - Odometry frame (REP-105): ``mola_bridge_odometry_frame:=<name>``.
   - Map frame: set both ``mola_state_estimator_reference_frame:=<name>`` (the
     ROS 2 bridge and the smoother) and ``mola_lo_reference_frame:=<name>`` (the
     LiDAR odometry). With the simple estimator, REP-105 is only applied when
     both are equal.

.. dropdown:: Subscription QoS for high-rate sensors
   :icon: dot

   Sensor subscriptions default to REP-2003 (``best_effort``, ``volatile``) with
   a queue depth of 50. For a high-rate IMU (e.g. 640 Hz) under a heavy pipeline,
   match a reliable publisher and absorb executor stalls with
   ``imu_qos_reliability:=reliable imu_qos_depth:=1000``. The same exists for
   the LiDAR: ``lidar_qos_reliability``, ``lidar_qos_depth``. Other topics take a
   per-topic ``qos:`` block in the ``subscribe`` list of
   ``lidar_odometry_ros2.yaml`` (see :ref:`ros2api_subscribed_topics`).

.. dropdown:: Camera images in the keyframes
   :icon: dot

   Raw ``sensor_msgs/Image`` topics (low rate) can be stored into the
   ``.simplemap`` keyframes, e.g. for visual place recognition:

   - Live: ``camera_topic_name:=/cam0/image,/cam1/image`` (up to three) with
     ``generate_simplemap:=True``.
   - ``mola-lo-gui-rosbag2``: ``MOLA_CAMERA_TOPIC``, ``MOLA_CAMERA_TOPIC_1``,
     ``MOLA_CAMERA_TOPIC_2`` with ``MOLA_SIMPLEMAP_ATTACH_LABEL='camera.*'``.
   - ``mola-lo-cli-rosbag2``: ``MOLA_CAMERA_TOPIC_0``, ``_1``, ``_2`` with
     ``MOLA_SIMPLEMAP_ATTACH_LABEL='image_.*'``.

   To add images to a map already built, use ``mola-sm-attach-from-rosbag2``.

.. dropdown:: Calling ``mola-lidar-odometry-cli`` directly
   :icon: terminal

   ``mola-lo-cli-rosbag2`` only fills in the pipeline and state estimator
   files. The equivalent call, with every input given as a flag:

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

.. _mola_ros2_cookbook_axes:

Parameter reference
-------------------

Each knob has a ``ros2 launch`` argument for the live node, and an environment
variable read by both offline wrappers (``mola-lo-gui-rosbag2`` and
``mola-lo-cli-rosbag2``). Environment variables also work in front of
``ros2 launch``, for knobs without a launch argument.

.. list-table::
   :header-rows: 1
   :widths: 22 39 39

   * - Concept
     - Live: ``ros2 launch`` argument
     - Offline: environment variable
   * - LiDAR topic
     - ``lidar_topic_name:=`` (mandatory), plus ``lidar_topic_type:=LaserScan``
       for 2D scans
     - ``MOLA_LIDAR_TOPIC``
   * - IMU topic
     - ``imu_topic_name:=`` (default ``imu``)
     - ``MOLA_IMU_TOPIC``
   * - LIO (IMU de-skew)
     - ``use_imu_for_lio:=True``
     - ``MOLA_DESKEW_METHOD=MotionCompensationMethod::IMU``
   * - Initial pitch and roll from the IMU (sensor still at start)
     - ``initial_localization_method:=InitLocalization::PitchAndRollFromIMU``
     - ``MOLA_LO_INITIAL_LOCALIZATION_METHOD=InitLocalization::PitchAndRollFromIMU``
   * - Smoother state estimator
     - ``use_state_estimator:=True``
     - ``MOLA_STATE_ESTIMATOR_YAML=<smoother params YAML>``
       (see :ref:`mola_ros2_cookbook_5_1`)
   * - Robot frame
     - ``mola_tf_base_link:=`` (default ``base_link``)
     - ``MOLA_TF_BASE_LINK``
   * - Map frame
     - ``mola_state_estimator_reference_frame:=`` and
       ``mola_lo_reference_frame:=`` (default ``map``)
     - (no ``/tf`` is published)
   * - Odometry frame (REP-105)
     - ``mola_bridge_odometry_frame:=`` (default ``odom``)
     - (no ``/tf`` is published)
   * - Publish ``map → odom`` instead of ``map → base_link``
     - ``publish_localization_following_rep105:=`` (default ``True``)
     - (no ``/tf`` is published)
   * - ``/tf`` and ``/tf_static`` topics
     - ``use_namespace:=True namespace:=<ns>`` (and ``use_namespaced_tf:=False``
       for a global ``/tf``)
     - ``MOLA_TF_TOPIC``, ``MOLA_TF_STATIC_TOPIC``
   * - Sensor poses given by hand
     - ``ignore_lidar_pose_from_tf:=True``, ``ignore_imu_pose_from_tf:=True``,
       plus the pose variables
     - ``MOLA_USE_FIXED_LIDAR_POSE``, ``MOLA_USE_FIXED_IMU_POSE``,
       ``MOLA_USE_FIXED_GNSS_POSE`` (see :ref:`mola_ros2_cookbook_no_tf`)
   * - GNSS topic
     - ``gnss_topic_name:=`` (``NavSatFix``, default ``gps``),
       ``gpsfix_topic_name:=`` (``gps_msgs/GPSFix``)
     - ``MOLA_GNSS_TOPIC`` (either type)
   * - GNSS usage
     - ``gnss_mode:=none|log_only|live_georef|relocalize``
     - ``MOLA_ESTIMATE_GEO_REF=true`` (smoother) for live geo-referencing
   * - Attitude-only IMU (AHRS)
     - ``attitude_imu_topic_name:=``
     - ``MOLA_ATTITUDE_IMU_TOPIC``
   * - External odometry topic (``nav_msgs/Odometry``)
     - ``odom_topic_name:=`` (and ``odom_sensor_label:=``, default
       ``odom_wheels``); planar wheel model with
       ``MOLA_ODOM_AS_ROBOT_POSE_OBS=false``
     - ``MOLA_ODOMETRY_TOPIC`` (and ``MOLA_ODOM_SENSOR_LABEL``); planar by
       default, 3D with ``MOLA_ODOMETRY_OBS_CLASS=CObservationRobotPose``
   * - External odometry from ``/tf``
     - ``forward_ros_tf_odom_to_mola:=True`` (simple estimator only)
     - (not available)
   * - Initial map, localization only
     - ``mola_initial_map_mm_file:=``, ``start_mapping_enabled:=False``
     - (not available)
   * - Save a ``.simplemap``
     - ``generate_simplemap:=True`` (and ``MOLA_SIMPLEMAP_OUTPUT``)
     - CLI: ``--output-simplemap``; GUI: ``MOLA_GENERATE_SIMPLEMAP=true``
       (and ``MOLA_SIMPLEMAP_OUTPUT``)
   * - Save the trajectory
     - ``MOLA_SAVE_TRAJECTORY=true`` (and ``MOLA_TUM_TRAJECTORY_OUTPUT``)
     - CLI: ``--output-tum-path``; GUI: as live
   * - Camera images in keyframes
     - ``camera_topic_name:=`` (up to three, comma-separated)
     - ``MOLA_CAMERA_TOPIC``, ``_1``, ``_2`` (GUI) or ``MOLA_CAMERA_TOPIC_0``,
       ``_1``, ``_2`` (CLI), plus ``MOLA_SIMPLEMAP_ATTACH_LABEL``
   * - LiDAR / IMU subscription QoS
     - ``lidar_qos_reliability:=``, ``lidar_qos_depth:=``,
       ``imu_qos_reliability:=``, ``imu_qos_depth:=``
       (default ``best_effort``, ``50``)
     - (not applicable)
   * - Simulated time
     - ``use_sim_time:=true``
     - (not applicable)
   * - GUIs
     - ``use_mola_gui:=``, ``use_rviz:=``
     - ``MOLA_WITH_GUI`` (GUI wrapper)

More environment variables: :ref:`sensor inputs <mola_lo_ros_mola-cli-env-vars>`
(live node and bags) and :ref:`odometry pipeline <pipelines_env_vars>`.

|

.. _mola_ros2_cookbook_fixes:

Troubleshooting
---------------

See :ref:`troubleshooting`: it covers the frame and topic problems of ROS 2
setups (missing ``base_link``, empty ``/tf``, namespaced ``/tf``, ...) and the
failure modes that are not specific to ROS 2.
