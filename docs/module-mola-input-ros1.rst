========================================
Module: mola-input-ros1
========================================

RawDataSource acting as a bridge: ROS1 -> MOLA

Can be used to interface a real sensor using a ROS driver node, or a dataset
in rosbag format; at present, datasets must be replayed externally
using `rosbag play`.


Build dependencies
--------------------
Building this module requires some ROS1 packages to be installed.
This can be done either from **official Debian/Ubuntu** repositories (i.e. with libraries under `/usr/`):

.. code-block:: bash

    sudo apt install libroscpp-dev libsensor-msgs-dev

or by installing a specific ROS distribution, and its `setup.bash` script
being sourced **before** invoking CMake to configure MOLA (i.e. with libraries under `/opt/`).


.. index::
   single: mola-input-ros1
   module: mola-input-ros1
