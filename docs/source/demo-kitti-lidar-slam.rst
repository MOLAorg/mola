.. _demos_kitti_lidar:

======================================
Demo: KITTI dataset, 3D-LiDAR SLAM
======================================


Usage
--------

.. code-block:: bash

    export KITTI_SEQ=00  # Select KITTI odometry sequence: 00-
    export KITTI_BASE_DIR=$HOME/dataset-kitti/  # set to your local copy!

    cd mola/demos
    mola-cli kitti_lidar_slam.yml -p


Configuration file, explained
--------------------------------

.. literalinclude:: ../../demos/kitti_lidar_slam.yml
   :language: yaml
   :linenos:
