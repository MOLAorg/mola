.. _geo-referencing:

======================
Georeferencing
======================
Geo-referencing, i.e. finding the transformation between a map and the Earth (latitude, longitude
and height), can be done live, while the vehicle moves, or afterwards on a recorded map. Both are
implemented in the :ref:`mola_state_estimation <mola_sta_est_index>` package.

The concept of using simple-maps as intermediary map format together with the layered metric map format (see :cite:`blanco2025mola_lo`)
enables embedding georeferenced coordinates to any kind of map typically used in robotics: grid maps, voxel maps, point clouds, etc.

|

.. image:: https://mrpt.github.io/imgs/kaist01_georef_sample.png
   :width: 250
   :align: right

|

Frames for geo-referenced maps
--------------------------------------------
When working with ROS ``tf`` (transformations), MOLA packages use the following frame convention, 
which extends the standard `REP-105 <https://www.ros.org/reps/rep-0105.html>`_ with additional
``enu`` and ``utm`` frames:

.. figure:: https://mrpt.github.io/imgs/mola_mrpt_ros_geo_referenced_utm_frames.png
   :width: 500
   :align: center

The existing frames are :ref:`explained here <mola_ros2_tf_frames>`.

|

.. _localization-lo_georef_gnss_imu_kinematics:

Live geo-referencing, without a prebuilt map
-----------------------------------------------
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
  **absolute attitude** IMU (AHRS with magnetometer, ``attitude_imu_topic_name``) gives an azimuth before
  the vehicle moves; give it a realistic sigma (``MOLA_IMU_ATTITUDE_SIGMA``) and an azimuth bias state
  (``MOLA_IMU_ATTITUDE_YAW_BIAS_SIGMA``), since magnetometers often carry a few degrees of bias. Its sigma
  also applies to roll and pitch, which on a vehicle that accelerates or vibrates follow those
  accelerations: with an accurate (RTK) receiver it may be better left out, and the azimuth then comes
  from the GNSS track once the vehicle moves (see below). With a consumer receiver (meters of noise) the
  GNSS baseline needed is long (tens of meters), so an attitude IMU is worth having.
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
``/tf``), an AHRS on ``/imu_ahrs`` and a GNSS on ``/gps`` (with an RTK receiver, consider dropping the AHRS
lines, see above):

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
for. Without wheel odometry, drop those variables and ``odom_topic_name``, use
``localization_publish_tf_source:=state_estimation`` and set
``publish_localization_following_rep105:=False`` (``map -> base_link``): with REP-105 on and no ``odom``
frame, the ROS 2 bridge has no ``odom -> base_link`` to compose with.

**Expected accuracy**: on an outdoor skid-steer ground robot at ~1 m/s with an RTK receiver whose fixes
fell back to float or no-RTK for 1-2 minutes at a time under trees, replays against an RTK-aligned LiDAR
odometry trajectory gave ~6 cm horizontal error with RTK fixed and ~12 cm median through the outages (where
the raw fixes were 0.5-2 m off), with ~1 deg heading error, without the AHRS (with it: ~11 and ~27 cm). With
a consumer (non-RTK) receiver the geo-reference inherits its bias (meters), while the trajectory stays
locally as accurate as the LiDAR odometry.

|

Geo-referencing a map afterwards
--------------------------------------------

A map built without live geo-referencing can be geo-referenced afterwards, if its keyframes
carry GNSS readings (see :ref:`mola_ros2_cookbook_5_4`), with these applications:


.. include:: ../../../mola_state_estimation/docs/georef-apps.rst

Geo-referenced maps in mm-viewer
----------------------------------------
:ref:`mm-viewer <app_mm-viewer>` shows the geo-reference of a ``.mm`` map, if it has one:

- **Apply georeferenced pose**: a checkbox that draws the map in the ``enu`` frame
  (applying :math:`T_{\text{enu}}^{\text{map}}`), with the ENU axes next to the map ones.
- **Mouse coordinates** in ``map``, ``enu`` or latitude/longitude units, with a button to
  copy them, e.g. to look up a point in a GIS tool.
- **Overlay of a polygon** given in latitude/longitude (e.g. a parcel or the boundary of the
  working area), from a text file with one ``lat,lon`` vertex (WGS84 degrees) per line:

  .. code-block:: bash

      mm-viewer myMap.mm --georef-polygon area.txt

|

View GNSS readings on a geo-referenced map
--------------------------------------------
Once you have published a georeferenced map (via `mrpt_map_server` or directly from `mola_lidar_odometry`),
you can visualize in RViz or FoxGlove the localization from the GNSS sensor and its covariance uncertainty
by installing `mola_gnss_to_markers <https://github.com/MOLAorg/mola_gnss_to_markers>`_ and then:


.. code-block:: bash

   ros2 launch mola_gnss_to_markers mola_gnss_to_markers_launch.py

