.. _geo-referencing:

======================
Georeferencing
======================
Georeferencing trajectories and metric maps is implemented in the :ref:`mola_state_estimation <mola_licenses>` package.

The concept of using simple-maps as intermediary map format together with the layered metric map format (see :cite:`blanco2025mola_lo`)
enables embedding georeferenced coordinates to any kind of map typically used in robotics: grid maps, voxel maps, point clouds, etc.

|

.. image:: https://mrpt.github.io/imgs/kaist01_georef_sample.png
   :width: 250
   :align: right

.. contents:: Table of Contents
   :depth: 2
   :local:
   :backlinks: none

|

1. Frames for geo-referenced maps
--------------------------------------------
When working with ROS ``tf`` (transformations), MOLA packages use the following frame convention, 
which extends the standard `REP-105 <https://www.ros.org/reps/rep-0105.html>`_ with additional
``enu`` and ``utm`` frames:

.. figure:: https://mrpt.github.io/imgs/mola_mrpt_ros_geo_referenced_utm_frames.png
   :width: 500
   :align: center

The existing frames are :ref:`explained here <mola_ros2_tf_frames>`.

|

2. Geo-referencing related applications
--------------------------------------------

.. include:: ../../../mola_state_estimation/docs/georef-apps.rst

3. Georeferenced maps in mm-viewer
----------------------------------------
Write me!

|

4. View GPS readings on a georeferenced map
--------------------------------------------
Once you have published a georeferenced map (via `mrpt_map_server` or directly from `mola_lidar_odometry`),
you can visualize in RViz or FoxGlove the localization from the GNSS sensor and its covariance uncertainty
by installing `mola_gnss_to_markers <https://github.com/MOLAorg/mola_gnss_to_markers>`_ and then:


.. code-block:: bash

   ros2 launch mola_gnss_to_markers mola_gnss_to_markers_launch.py

