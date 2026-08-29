.. _your_own_rosbag:

================================
Run MOLA on your own ROS 2 bag
================================

You have a bag with a 3D LiDAR in it and you want a trajectory and a map out
of it. This page answers the five questions you will hit, in the order you
hit them.

The short version, if your bag is well behaved:

.. code-block:: bash

   MOLA_LIDAR_TOPIC=/your/points \
     mola-lo-gui-rosbag2 /path/to/your_bag.mcap

A directory containing ``metadata.yaml`` works in place of the ``.mcap`` file.


1. Which topic is my LiDAR?
----------------------------

.. code-block:: bash

   ros2 bag info /path/to/your_bag.mcap

Find the topic whose type is ``sensor_msgs/msg/PointCloud2``, and pass it as
``MOLA_LIDAR_TOPIC``. The default is ``/ouster/points``, which is almost
certainly not what your bag calls it.

Set ``MOLA_IMU_TOPIC`` too if the bag carries a ``sensor_msgs/msg/Imu``. It
is optional, but an IMU improves deskewing and, on some rigs, is what makes
the run work at all.

.. note::
   MOLA needs **per-point timestamps** to deskew a spinning LiDAR properly.
   Most drivers publish them, but not all, and a cloud without them is
   processed as if every point were captured at the same instant. On a
   vehicle at road speed that is a visible distortion, not a rounding error.


2. Do I have ``/tf``?
----------------------

This decides whether MOLA tracks your **vehicle** or your **sensor**.

If the bag has a ``/tf_static`` giving the transform from ``base_link`` to
the LiDAR frame, MOLA uses it and the trajectory is your vehicle's. Nothing
to configure.

If it does not, which is common when only a driver node was recorded, tell
MOLA where the sensor sits:

.. code-block:: bash

   MOLA_USE_FIXED_LIDAR_POSE=true \
   LIDAR_POSE_X=0.0 LIDAR_POSE_Y=0.0 LIDAR_POSE_Z=1.6 \
   LIDAR_POSE_YAW=0 LIDAR_POSE_PITCH=0 LIDAR_POSE_ROLL=0 \
   MOLA_LIDAR_TOPIC=/your/points \
     mola-lo-gui-rosbag2 /path/to/your_bag.mcap

Positions are in meters, angles in degrees. Leaving all six at zero is a
valid choice: it means "track the sensor, and I know it".

For a namespaced bag (``/robot1/tf``), also set ``MOLA_TF_TOPIC`` and
``MOLA_TF_STATIC_TOPIC``, and ``MOLA_TF_BASE_LINK`` if your base frame is not
called ``base_link``.


3. GUI or CLI?
---------------

Use ``mola-lo-gui-rosbag2`` the first few times: you want to watch the cloud
and see whether the trajectory looks sane.

Switch to ``mola-lo-cli-rosbag2`` for anything you intend to measure. The GUI
drops scans under real-time pacing and is not reproducible. This matters more
than it sounds and is worth reading :ref:`gui_vs_cli` before you publish a
number.


4. Which pipeline?
-------------------

Do not choose one yet. The default is tuned to be reasonable across very
different sensors and environments, and it is the right starting point for a
bag nobody has seen before.

Change it only once you have a run to compare against, and see
:ref:`mola_lo_pipelines` for what the alternatives actually do. The
:ref:`per-dataset pages <supported_datasets>` are worth reading here too:
each records which pipeline that dataset needed and the measurements behind
the choice, which is a better guide than picking one by name.


5. Where did my map go?
------------------------

By default, nowhere. MOLA-LO computes a trajectory; saving a map is opt-in:

.. code-block:: bash

   MOLA_GENERATE_SIMPLEMAP=true \
   MOLA_SIMPLEMAP_OUTPUT=my_map.simplemap \
   MOLA_LIDAR_TOPIC=/your/points \
     mola-lo-cli-rosbag2 /path/to/your_bag.mcap

A *simple-map* is the keyframe poses plus their raw observations. It is the
input to map building, not the finished map: turning it into a metric map you
can localize against is covered in :ref:`building-maps`.

You can also toggle this from the ``mola_lidar_odometry`` sub-window while
the GUI is running.


When it does not work
----------------------

See :ref:`troubleshooting`. The failure modes are few and mostly about
``/tf``, topic names and timestamps.

For the full matrix of live-node, namespaced, geo-referenced and
external-odometry setups, see :ref:`ROS 2 configurations
<mola_ros2_cookbook>`.
