.. _supported_sensors:

======================
Sensors and datasets
======================

This page lists the possibilities to feed SLAM algorithms with sensory data
in the MOLA framework.


1) Off-line: read from datasets
--------------------------------

Parsing data from a dataset is the easiest way to test, debug, and benchmark
any kind of localization, mapping, or classification algorithm.
Existing options are:

From a ROS bag
==============

Using :ref:`mola-input-ros1` as a bridge between ROS->MOLA, then
replaying the ROS bag using standard ROS tools, i.e. `rosbag play`.


From an MRPT rawlog
===================

Using :ref:`mola-input-rawlog` to parse a .rawlog file and have all its observations
delivered to the rest of MOLA modules (sensor front-ends).


From public robotics datasets
==============================

Specific MOLA modules exist to directly parsing popular public datasets:

 - :ref:`mola-input-euroc-dataset`
 - :ref:`mola-input-kaist-dataset`
 - :ref:`mola-input-kitti-dataset`


2) On-line: direct sensor connection
-------------------------------------

There exist two alternatives:

 - to use ROS drivers for sensors and then use :ref:`mola-input-ros1` as a bridge between ROS->MOLA.
 - to directly connect to any sensor supported by mrpt-hwdrivers using the MOLA native :ref:`mola-input-hwdrivers`.

In general, using mrpt-hwdrivers should be more efficient and convenient.
A non-exhaustive list of sensors supported by this library is:
 - **2D lidars**: Hokuyo, SICK, RP-Lidar, Ibeo.
 - **3D lidars**: Velodyne.
 - **Cameras**: Any camera supported by OpenCV and ffmpeg. Bumblebee2 stereo cameras.
 - **Depth sensors**: Original XBox Kinect, any camera using OpenNI2.
 - **IMUs**: xSens, KVH DSP3000.
 - **GNSS (GPS)**: Any sensor providing a standard NMEA interface. Novatel receivers.

.. index::
   single: supported sensors
