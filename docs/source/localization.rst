.. _localization-only:

======================
Localization
======================
Localization differs from SLAM in that the map or world model is **not updated** while
it is being used to keep a robot or vehicle accurately localized within a pre-mapped environment :cite:`blanco2025mola_lo`.

Once :ref:`a map is built <building-maps>`, the result is a simple-map and/or a metric map (``*.mm``), optionally :ref:`georeferenced <geo-referencing>`.
This map can then be used to enable autonomous navigation ("Go from A to B"), for which localization is a fundamental
requirement ("Where is B?").

At present, this framework provides two fundamentally different algorithms for **localization**:

* **Based on optimization**, by using the :ref:`LiDAR odometry module <mola_lidar_odometry>`, without map updates.
* **Based on particle filtering**, by using any combination of metric maps (grid maps, point clouds, etc.) and wheels odometry.

____________________________________________

.. contents:: Table of Contents
   :depth: 1
   :local:
   :backlinks: none

____________________________________________

|

.. _localization-only_common:


1. Common aspects
--------------------------------------
Independently of the localization algorithm, the problem of finding the *rough* localization
of the vehicle at start up is quite special due to the large **initial uncertainty**.

MOLA provides a specific module for this task (:ref:`doxid-group__mola__relocalization__grp`)
which can be used by all position tracking algorithms to solve the initial relocalization problem.

Also, implementations below offer two ways to specify the **initial guess** about the robot
pose in the environment:

* From **a pose with a covariance**, which needs to be given manually either from a ROS topic, RViz/FoxGlove, or a parameter; and
* From **GNSS readings** ("Automatic re-localization via GPS"). This version requires :ref:`georeferenced metric maps <geo-referencing>`, naturally.

|

----

2. Localization with LiDAR odometry (LO) and a reference map
---------------------------------------------------------------
See the step-by-step instructions in :ref:`this tutorial <tutorial-mola-lo-map-and-localize>`.
As a summary, you need to:

* Launch MOLA-LO in localization-only mode.
* Load a local map into it. In MOLA-LO, a local map comprises two files, a ``*.mm`` metric map and the keyframes in a ``*.simplemap`` file.
  (In fact: keyframes are only required if you want to extend the map, i.e. multi-session mapping).
* Request relocalization via a service call or via the topic ``/initialpose``.

This video shows the steps in the tutorial:

.. raw:: html

    <div style="margin-top:10px;">
      <iframe width="560" height="315" src="https://www.youtube.com/embed/CXiU_2vYMQE" title="YouTube video player" frameborder="0" allow="accelerometer; autoplay; clipboard-write; encrypted-media; gyroscope; picture-in-picture; web-share" referrerpolicy="strict-origin-when-cross-origin" allowfullscreen></iframe>
    </div>

|

----

.. _localization-lo_gnss_kinematics:

3. LO/LIO + GNSS + kinematics localization
---------------------------------------------------------------
Based on the smoother state estimator, this solution allows for improved localization in a prebuilt map
by fusing the output of LiDAR odometry (LO or LIO) with GNSS and kinematic data.

Write me!

|

----

.. _localization-lo_georef_gnss_imu_kinematics:

4. Map-less georeferenced localization: LO/LIO + GNSS + IMU + kinematics localization
-----------------------------------------------------------------------------------------
Based on the smoother state estimator, this solution allows to localize a vehicle in geodetic or UTM coordinates without the need of a prebuilt map.

MOLA-LO runs in **mapping mode with a local map that follows the vehicle** (old points are dropped, so
memory and run time stay bounded however long the session is), and its poses go to the
:ref:`smoother state estimator <mola_sta_est_smoother>` as **one more odometry**, together with wheel odometry,
an IMU and a GNSS receiver. The smoother corrects the drift of the LiDAR odometry with GNSS, bridges GNSS
outages (under trees, indoors) with the LiDAR and wheel odometry, and **estimates the geo-reference**
(:math:`T_{\text{enu}}^{\text{map}}`) **online**. The result is a pose that is locally as smooth as LiDAR odometry
and globally as accurate as the GNSS, given both in the ``map`` frame (where the vehicle started) and in
latitude/longitude, e.g. to give navigation goals on satellite imagery.

**How the sources are combined**

- **LiDAR odometry** publishes its poses in its own frame (``mola_lo_reference_frame:=lidar_odom``), so the
  smoother fuses only their increments, with the accuracy ``relative_pose_increment_sigma_lin/ang``
  (``MOLA_REL_POSE_INCR_SIGMA_LIN/ANG``, per pose, as a random walk), and estimates how that frame drifts with
  respect to ``map``. Its local map is not geo-referenced and never needs to be: use a local map class with a
  maximum size, e.g. ``mola::IncrementalPointCloud`` (the default of the GICP pipelines;
  ``MOLA_INCREMENTAL_MAP_MAX_SIZE`` in meters).
- **Wheel odometry** (``odom_topic_name``) is better forwarded as planar odometry
  (``MOLA_ODOM_AS_ROBOT_POSE_OBS=false``), so the smoother weights it with its own motion model
  (``odom_motion_model_a1..a4``: error proportional to distance and rotation) instead of the much tighter
  LiDAR odometry increment sigmas, which would let wheel slippage (skid-steer turns!) drag the heading.
- **IMU**: the LiDAR IMU de-skews scans (``use_imu_for_lio:=True``) and gives gravity. An optional
  **absolute attitude** IMU (magnetometer heading, ``attitude_imu_topic_name``) gives an azimuth from the
  start; give it a realistic sigma (``MOLA_IMU_ATTITUDE_SIGMA``) and an azimuth bias state
  (``MOLA_IMU_ATTITUDE_YAW_BIAS_SIGMA``), since magnetometers often carry a few degrees of bias. Without it,
  the azimuth comes from the GNSS track once the vehicle moves (see below).
- **GNSS** (``gnss_mode:=live_georef``, ``gnss_topic_name``): each fix is an ENU position factor with a robust
  (Huber) kernel. Its ``NavSatFix`` covariance must reflect the real accuracy: some receivers report
  centimeter-level sigmas also in RTK float or DGNSS solutions, which are tens of centimeters or meters off.
  Republish them with a covariance per solution type if needed (e.g. at least 3 cm for RTK fixed, 0.5 m for
  RTK float, 3 m otherwise). The antenna position on the vehicle (lever arm) comes from ``/tf``: measure or
  calibrate it, since with the antenna high above the vehicle frame, tilting moves it.

**Geo-reference convergence**: the ``map`` frame is the first vehicle pose. The geo-reference is published
once its uncertainty is below ``MOLA_SE_CONVERGENCE_POS_SIGMA`` (m) and ``MOLA_SE_CONVERGENCE_ORI_SIGMA``
(deg). With an attitude IMU and RTK, that takes about a second. Without attitude, the azimuth is unknown until
the GNSS fixes span a baseline of about :math:`\sqrt{2}\,\sigma_{xy} / \sigma_{\text{conv}}` (0.5 m with 3 cm fixes
and 5 deg): drive a few meters.

**Outputs** (besides the usual ones, see :ref:`ROS 2 API <mola_ros2api>`):

- ``/tf``: ``map -> odom``, straight from the smoother estimate of the wheel odometry frame
  (``localization_publish_tf_source:=state_estimation/map_odom`` and ``MOLA_PUBLISH_MAP_TO_ODOM_TF``, see
  :ref:`TF frames <mola_ros2_tf_frames>`), so the robot ``odom -> base_link`` chain is kept (REP 105); and, once
  converged, ``enu -> map`` and ``enu -> utm``.
- ``state_estimator/geo_ref_metadata`` (``mrpt_nav_interfaces/GeoreferencingMetadata``, transient local): the
  geo-reference (ENU origin latitude/longitude/height, :math:`T_{\text{enu}}^{\text{map}}`, UTM zone).
- ``state_estimation/geo_pose`` (``geographic_msgs/GeoPoseStamped``): the vehicle pose in latitude, longitude
  and height, with its ENU orientation.
- ``state_estimation/pose`` (``nav_msgs/Odometry``): the fused pose in ``map``.

A goal given in latitude/longitude can be converted to the ``map`` frame with the geo-reference
(geodetic to ENU at its origin, then :math:`(T_{\text{enu}}^{\text{map}})^{-1}`), or sent in the ``enu`` frame to a
planner that transforms goals with ``/tf``.

**Example**, for a vehicle with an Ouster LiDAR, wheel odometry on ``/odom`` (``odom -> base_link`` on
``/tf``), an AHRS on ``/imu_ahrs`` and an RTK GNSS on ``/gps``:

.. code-block:: bash

    MOLA_INCREMENTAL_MAP_MAX_SIZE=60.0 \
    MOLA_REL_POSE_INCR_SIGMA_LIN=0.005 MOLA_REL_POSE_INCR_SIGMA_ANG=0.0005 \
    MOLA_ODOM_AS_ROBOT_POSE_OBS=false \
    MOLA_PUBLISH_MAP_TO_ODOM_TF=true \
    MOLA_MAP_TO_ODOM_FRAME=odom_wheels MOLA_MAP_TO_ODOM_CHILD_FRAME=odom \
    MOLA_IMU_ATTITUDE_SIGMA=3.0 MOLA_IMU_ATTITUDE_YAW_BIAS_SIGMA=10.0 \
    ros2 launch mola_lidar_odometry ros2-lidar-odometry.launch.py \
      mola_lo_pipeline:=$(ros2 pkg prefix mola_lidar_odometry)/share/mola_lidar_odometry/pipelines/lidar3d-gicp.yaml \
      lidar_topic_name:=/ouster/points \
      imu_topic_name:=/ouster/imu use_imu_for_lio:=True \
      use_state_estimator:=True \
      gnss_mode:=live_georef gnss_topic_name:=/gps \
      attitude_imu_topic_name:=/imu_ahrs \
      odom_topic_name:=/odom \
      mola_lo_reference_frame:=lidar_odom \
      publish_localization_following_rep105:=True \
      localization_publish_tf_source:=state_estimation/map_odom

``MOLA_MAP_TO_ODOM_FRAME`` is the smoother name of the wheel odometry source (``odom_sensor_label``, default
``odom_wheels``) and ``MOLA_MAP_TO_ODOM_CHILD_FRAME`` the ``/tf`` frame its driver publishes ``odom -> base_link``
for. Without wheel odometry, drop those variables and ``odom_topic_name``, and use
``localization_publish_tf_source:=state_estimation`` (``map -> base_link``).

**Expected accuracy**: on an outdoor ground robot at ~1 m/s with an RTK receiver whose fixes fell back to
float or no-RTK for 1-2 minutes at a time under trees, replays against an RTK-aligned LiDAR odometry
trajectory gave ~10 cm horizontal error with RTK fixed and ~25 cm median through the outages (where the raw
fixes were 0.5-2 m off), with ~1 deg heading error. With a consumer (non-RTK) receiver the geo-reference
inherits its bias (meters), while the trajectory stays locally as accurate as the LiDAR odometry.

|

----


5. Localization with particle filtering and a reference map
------------------------------------------------------------
To localize with a particle filter (PF) you will need:

1. A metric map (the ``.mm`` file) of the environment. Built as described in: :ref:`building-maps`.
2. To publish that map using `mrpt_map_server <https://github.com/mrpt-ros-pkg/mrpt_navigation/tree/ros2/mrpt_map_server/>`_.
3. Launch `mrpt_pf_localization <https://github.com/mrpt-ros-pkg/mrpt_navigation/tree/ros2/mrpt_pf_localization>`_ and configure it
   to read from wheels odometry (if they exist), the input sensors (e.g. LiDAR), GPS (GNSS) if present. This node will subscribe to
   the published metric map and use the sensors to publish updates on the robot pose in the `map` frame.

.. note::

   Do **NOT** set the raw LiDAR pointcloud as input for the PF. It is just too large and it must be subsampled first.
   We can use `mrpt_pointcloud_pipeline <https://github.com/mrpt-ros-pkg/mrpt_navigation/tree/ros2/mrpt_pointcloud_pipeline>`_
   for such task, as can be seen in the tutorial below.

5.1. PF localization tutorial
================================
A complete demonstration has been put together on: https://github.com/MOLAorg/mola_warehouse_pf_tutorial

.. raw:: html

   <div style="width: 100%; overflow: hidden;">
     <video controls autoplay loop muted style="width: 512px;">
       <source src="https://mrpt.github.io/videos/mola-localization-pf-demo-warehouse.mp4" type="video/mp4">
     </video>
   </div>

Key points of this tutorial:

- The ROS 2 `launch file <https://github.com/MOLAorg/mola_warehouse_pf_tutorial/blob/develop/launch/tutorial_launch.py>`_.
- A `custom sm2mm pipeline file <https://github.com/MOLAorg/mola_warehouse_pf_tutorial/tree/develop/sm2mm-config>`_ to create
  a reference point cloud map sparse enough so localizing with a PF is not too slow.
- A `pipeline for mrpt_pointcloud_pipeline <https://github.com/MOLAorg/mola_warehouse_pf_tutorial/blob/develop/params/point-cloud-pipeline.yaml>`_
  to decimate the input raw 3D scan as input to the PF.


.. dropdown:: How to run the tutorial

   Clone the tutorial package (which already includes a prebuilt ``.mm``), make sure of having all dependencies,
   build and run it:

   .. code-block:: bash

         cd ~/ros2_ws/src
         git clone https://github.com/MOLAorg/mola_warehouse_pf_tutorial.git

         cd ~/ros2_ws
         rosdep install --from-paths src --ignore-src -r -y

         colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo
         . install/setup.bash

         ros2 launch mola_warehouse_pf_tutorial tutorial_launch.py



