.. _demos_rawlog_velodyne_slam:

========================================================
Demo: Velodyne dataset in Rawlog format, 3D-LiDAR SLAM
========================================================

This demo shows how to launch a 3D LiDAR SLAM system reading Velodyne scans
from an MRPT .rawlog file.

To run this demo, download the example dataset: `sample_rawlog1`_ (part of `sample_dataset`_).

.. image:: demo-rawlog-velodyne-slam.png

Usage
--------

.. code-block:: bash

    cd mola/demos
    MOLA_INPUT_RAWLOG=map1_test1.rawlog  mola-cli rawlog_odom_and_lidar.yml -p


.. _sample_rawlog1: https://ingmec.ual.es/datasets/lidar3d-pf-benchmark/map1_test1_cut.rawlog
.. _sample_dataset: https://ingmec.ual.es/datasets/lidar3d-pf-benchmark/


Configuration file, explained
--------------------------------

.. literalinclude:: ../../demos/rawlog_odom_and_lidar.yml
   :language: yaml
   :linenos:
