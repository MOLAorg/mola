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


.. _localization-only_common:


Common aspects
--------------
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

Localization with LiDAR odometry (LO) and a reference map
---------------------------------------------------------
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

.. _mola_ros2_initial_localization:

Initial localization
=====================
When the LO system is started, there are different situations: 

1. The system is started **without any former map**. Here, the default is starting at the identity SE(3) pose,
   that is, at the origin (0,0,0), and that should be enough in most common cases.

2. The system is started **with a former known map**. Here, correctly localizing within that map before trying to update
   it is critical to avoid ruining the map. Also, finding the correct initial pose is a non trivial problem and requires
   specific methods.

For the latter case, it is important to disable mapping at start up (see the ``start_mapping_enabled:=False`` :ref:`launch argument <ros2_node_lo_docs>`) and only enable mapping once the system is correctly localized, and if
the user really wants to update the map. Keeping mapping disabled for the whole run is actually desired for robots
operating in a known, pre-mapped environment.

Then, the user can choose between: 

- Requesting re-localization in a given area or from GNSS readings, with :ref:`these services <mola_ros2api_relocalization>`.
- Selecting one of the available initial localization methods directly set in the pipeline configuration file,
  or via a ROS2 launch argument, so that method is used straight away at startup.

These are the available initial localization methods, that can be used in the launch argument 
``initial_localization_method:=xxxx`` launch argument (see :ref:`the launch arguments <ros2_node_lo_docs>`):

.. dropdown:: How to select initial localization without ROS API

   If LO is launched independently of a ROS2 system, e.g. using the 
   :ref:`command-line <mola_lidar_odometry_cli>` or :ref:`GUI <mola_lo_apps>` LO tools,
   the initial localization method can be set via the environment variable
   ``MOLA_LO_INITIAL_LOCALIZATION_METHOD`` which should be set to any of the options
   listed below. For example, to set the initial localization method to ``FromStateEstimator``:

   .. code-block:: bash

      MOLA_LO_INITIAL_LOCALIZATION_METHOD="InitLocalization::FromStateEstimator" \
      mola-lidar-odometry-cli [...]  # the rest as usual


- ``InitLocalization::FixedPose``: Initializes around a given SE(3) pose with covariance.

- ``InitLocalization::FromStateEstimator``: In combination with the smoother state estimator,
  can be used to initialize based on accumulated evidence of geo-referenced positioning based on low-cost 
  GNSS readings, wheels odometry, IMU, or any sensible combination of sensors. See :ref:`smoother state estimator <mola_sta_est_index>`.

- ``InitLocalization::PitchAndRollFromIMU``: Without using the external state estimator, this method
  uses the IMU to estimate the pitch and roll angles of the robot, and then initializes the localization
  system with that information **assuming sensor is roughly stationary at startup**.
  This is useful for systems that are not perfectly level, such as hand-held devices, drones, etc.
  since it will remove the apparent tilt of the ground plane.

|

----

.. _localization-lo_gnss_kinematics:

LO/LIO + GNSS + kinematics localization
---------------------------------------
Based on the smoother state estimator, this solution allows for improved localization in a prebuilt map
by fusing the output of LiDAR odometry (LO or LIO) with GNSS and kinematic data.

MOLA-LO runs in localization-only mode against a **geo-referenced** map, and the smoother fuses its
poses with wheel odometry and GNSS. The GNSS fixes give the initial pose in the map, so no manual
initial guess is needed: MOLA-LO stays idle until the smoother has converged on it
(``gnss_mode:=relocalize``), and then keeps the pose of the smoother as its motion prior. Wheel odometry
carries the pose through stretches where the scans match the map poorly.

This combines two :ref:`ROS 2 configuration recipes <mola_ros2_cookbook>`:
:ref:`relocalization with GNSS <mola_ros2_cookbook_4_6>` and
:ref:`external odometry in the smoother <mola_ros2_cookbook_4_4>`:

.. code-block:: bash

    MOLA_ODOM_AS_ROBOT_POSE_OBS=false \
    ros2 launch mola_lidar_odometry ros2-lidar-odometry.launch.py \
      lidar_topic_name:=/ouster/points \
      imu_topic_name:=/ouster/imu use_imu_for_lio:=True \
      gnss_topic_name:=/gps \
      odom_topic_name:=/odom \
      mola_initial_map_mm_file:=/path/to/geo-referenced-map.mm \
      use_state_estimator:=True \
      start_mapping_enabled:=False \
      gnss_mode:=relocalize \
      publish_localization_following_rep105:=False

``MOLA_ODOM_AS_ROBOT_POSE_OBS=false`` fuses the wheel odometry as planar odometry, with its own motion
model. This publishes ``map -> base_link``; to keep the ``odom -> base_link`` of the wheel odometry driver
instead (REP-105), publish ``map -> odom`` from the smoother as explained in
:ref:`localization-lo_georef_gnss_imu_kinematics`.

|

----

Other ways of localizing
--------------------------

- **Without a prebuilt map**, in geodetic coordinates, fusing LiDAR odometry with GNSS: see
  :ref:`localization-lo_georef_gnss_imu_kinematics`.
- **With a particle filter** and any combination of metric maps (grid maps, point clouds,
  ...) and wheel odometry: see :ref:`tutorial-pf-localization`.
