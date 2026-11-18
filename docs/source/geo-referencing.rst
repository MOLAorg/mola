.. _geo-referencing:

======================
Georeferencing
======================
Georeferencing trajectories and metric maps is implemented in the :ref:`mola_state_estimation <mola_sta_est_index>` package.

The concept of using simple-maps as intermediary map format together with the layered metric map format (see :cite:`blanco2025mola_lo`)
enables embedding georeferenced coordinates to any kind of map typically used in robotics: grid maps, voxel maps, point clouds, etc.

|

.. image:: https://mrpt.github.io/imgs/kaist01_georef_sample.png
   :width: 250
   :align: right

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

4. View GPS readings on a georeferenced map
--------------------------------------------
Once you have published a georeferenced map (via `mrpt_map_server` or directly from `mola_lidar_odometry`),
you can visualize in RViz or FoxGlove the localization from the GNSS sensor and its covariance uncertainty
by installing `mola_gnss_to_markers <https://github.com/MOLAorg/mola_gnss_to_markers>`_ and then:


.. code-block:: bash

   ros2 launch mola_gnss_to_markers mola_gnss_to_markers_launch.py

