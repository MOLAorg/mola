.. _troubleshooting:

=================
Troubleshooting
=================

Symptoms that come up repeatedly, and what is usually behind them.

.. contents::
   :depth: 1
   :local:
   :backlinks: none


Nothing happens, or the odometry stops
---------------------------------------

``unknown class name`` when using the smoother
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

The smoother state estimator lives in a plugin that the CLI does not load by
default, so the class factory cannot find it. Load it explicitly:

.. code-block:: bash

   mola-lidar-odometry-cli -l libmola_state_estimation_smoother.so ...


The trajectory freezes part-way through and never resumes
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

If the IMU stops publishing mid-run, every later scan waits for data that
will never arrive. MOLA bounds that wait with
``max_time_to_wait_for_imu`` (default 0.5 s) and degrades to LiDAR-only,
picking the IMU back up by itself if it returns. If you have set that
parameter to ``0``, the wait is unbounded by design and a dead IMU stalls the
run permanently and silently.

Check whether the IMU actually covers the whole bag before assuming the
odometry is at fault.


The map looks wrong
--------------------

Every scan is smeared or skewed
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

The clouds probably carry no **per-point timestamps**, so the whole scan is
treated as captured at one instant and no deskewing can happen. Most drivers
publish them; some do not. At road speed the distortion is plainly visible,
not a rounding error.


The trajectory tracks the sensor, not the vehicle
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

There is no ``base_link`` to LiDAR transform in ``/tf_static``, so MOLA has
nothing to compose with. Either publish it, or state it yourself with
``MOLA_USE_FIXED_LIDAR_POSE=true`` and the ``LIDAR_POSE_*`` variables. See
:ref:`your_own_rosbag`.


Two runs of the same data give different answers
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

Expected, if you used the GUI: it paces in real time and drops scans. Use the
offline CLI, pin it with ``taskset -c N``, and verify with ``md5sum`` that
two runs are bit-identical before comparing anything. :ref:`gui_vs_cli`
explains why.


ROS 2 frames and topics
------------------------

.. list-table::
   :header-rows: 1
   :widths: 35 65

   * - Symptom
     - Most common cause
   * - ``"base_link" passed to lookupTransform ... does not exist``
     - The bag or driver is not publishing a ``base_link`` frame, or the
       launch ``mola_tf_base_link`` does not match the one in ``/tf``.
   * - LO starts but ``/tf`` is empty, or the map is not visible in RViz2
     - ``publish_localization_following_rep105:=False`` was not set while no
       external odometry is running. The launch is waiting for an
       ``odom -> base_link`` that never arrives.
   * - Smoother is set, but ``map -> odom`` appears instead of
       ``map -> base_link``
     - REP-105 is incompatible with the smoother. Set
       ``publish_localization_following_rep105:=False``, or simply do not set
       it: the launch default already matches when
       ``use_state_estimator:=True``.
   * - Namespaced bag silently ignored on ``/tf``
     - Older ``mola_input_rosbag2`` hardcoded ``/tf``. Update the package and
       set ``MOLA_TF_TOPIC`` and ``MOLA_TF_STATIC_TOPIC``.
   * - External odometry is fused, but ``map -> odom`` points at the wrong
       frame
     - The smoother keys each odometry source by its **sensor label**, and
       the ``map -> odom`` child frame must equal the REP-105 odom frame the
       external driver publishes. Either set the smoother's
       ``map_to_odom_child_frame`` to that frame, or set
       ``MOLA_ODOM_SENSOR_LABEL`` to it.
   * - Smoother converges slowly to the geo-reference
     - GNSS factors only constrain ``enu -> map`` once the vehicle has moved
       non-trivially. Drive at least a few meters.

The full matrix of live-node, namespaced, geo-referenced and
external-odometry configurations, with complete launch lines, is in
:ref:`ROS 2 configurations <mola_ros2_cookbook>`.


Results that look fine but are not
-----------------------------------

Two checks worth making before trusting any error number:

- **Read the run summary.** ``Run totals: registrations=N no_motion_model=K
  icp_rejected=M``. A non-zero ``no_motion_model`` means part of the
  trajectory was registered from a standstill guess with no motion prior at
  all, whatever the error metric says.
- **Confirm the estimate spans the whole ground truth.** A run that stopped
  early can score very well on any alignment-based metric.
