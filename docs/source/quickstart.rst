.. _quickstart:

=====================
MOLA in five minutes
=====================

Install MOLA, run 3D LiDAR odometry on a real sequence, and see a map. No
configuration, no dataset download, nothing to edit.

If you already know you want to run MOLA on **your own** data, skip this and
go to :ref:`your_own_rosbag`.


1. Install
-----------

.. code-block:: bash

   sudo apt install \
     ros-$ROS_DISTRO-mola \
     ros-$ROS_DISTRO-mola-lidar-odometry \
     ros-$ROS_DISTRO-mola-test-datasets

The third package is a handful of short dataset extracts, which is what makes
this page a five-minute exercise instead of a download.

Other ways to install, including from source and the exact versions on each
ROS 2 distribution, are on the :ref:`installing <installing>` section of the
home page.


2. Run it
----------

.. code-block:: bash

   mola-lo-gui-rawlog \
     $(ros2 pkg prefix mola_test_datasets)/share/mola_test_datasets/datasets/mulran/mulran_KAIST01_extract.rawlog

That is a fragment of the MulRan ``KAIST01`` sequence: a car with an Ouster
OS1-64 driving about 50 m through Daejeon.

A window opens, the point cloud starts scrolling past, and a trajectory is
traced behind the vehicle. The run ends by itself after about eight seconds
of sensor time.


3. What just happened
----------------------

MOLA registered each incoming LiDAR scan against a local map built from the
previous ones, and the accumulated chain of those registrations is the
trajectory you watched being drawn. That is LiDAR odometry: no loop closure,
no global optimization, no prior map.

The last line before shutdown is the run summary:

.. code-block:: text

   Run totals: registrations=79 no_motion_model=0 (0.00%) icp_rejected=0 (0.00%)

``registrations`` is how many scans were aligned. The other two are the
health indicators worth knowing about early: ``no_motion_model`` counts scans
that had to be registered from a standstill guess because the state estimator
had nothing to offer, and ``icp_rejected`` counts alignments that were thrown
out. Both should be at or near zero on a well-behaved sequence, and a
non-zero value tells you something about the run that the trajectory alone
will not.


4. Where to go next
--------------------

.. list-table::
   :widths: 45 55
   :header-rows: 0

   * - **Run it on your own ROS 2 bag**
     - :ref:`your_own_rosbag` walks through the five things you have to
       decide, in the order you hit them.
   * - **Run it on a public benchmark dataset**
     - :ref:`supported_datasets` has a page per dataset, each with the exact
       command and the tuning that dataset needs.
   * - **Actually build and save a map**
     - :ref:`building-maps` covers the full pipeline: simple-map, metric map,
       and the tools that consume them.
   * - **Compare MOLA against another method**
     - Read :ref:`gui_vs_cli` first. The GUI you just used is not the tool to
       benchmark with, and the reason is not obvious.
   * - **Something went wrong**
     - :ref:`troubleshooting`.
