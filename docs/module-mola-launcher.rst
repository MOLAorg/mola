======================
Module: mola-launcher
======================

.. toctree::
  :maxdepth: 2

----

.. index::
   single: mola-cli

Application: ``mola-cli``
##########################

``mola-cli``  is the most-common entry point to the MOLA system for users.
Refer to :ref:`demos` for real examples to launch and run SLAM systems.

-------------
SYNOPSIS
-------------

.. code-block:: none

    USAGE:

       mola-cli  [--rtti-children-of <mp2p_icp::ICP_Base>] [--rtti-list-all]
                 [--profiler-whole] [-p] [-v <INFO>] [-c <demo.yml>] [--]
                 [--version] [-h]


    Where:

       --rtti-children-of <mp2p_icp::ICP_Base>
         Loads all MOLA modules, then list all known classes that inherit from
         the given one

       --rtti-list-all
         Loads all MOLA modules, then list all classes registered via
         mrpt::rtti

       --profiler-whole
         Enable whole-history time profiler in all modules (Default: NO). **DO
         NOT** use in production, only to benchmark short runs (unbounded
         memory usage)

       -p,  --profiler
         Enable time profiler by default in all modules (Default: NO)

       -v <INFO>,  --verbosity <INFO>
         Verbosity level: ERROR|WARN|INFO|DEBUG (Default: INFO)

       -c <demo.yml>,  --config <demo.yml>
         Input YAML config file (required) (*.yml)

       --,  --ignore_rest
         Ignores the rest of the labeled arguments following this flag.

       --version
         Displays version information and exits.

       -h,  --help
         Displays usage information and exits.

Notes:

  - Finer-control of the verbosity for individual modules is possible by using the `verbosity` variable in the YAML launch file, see: :ref:`yaml_slam_cfg_file`.

Example: Launching a SLAM system with performance details at end:

.. code-block:: none

  mola-cli -c kitti_lidar_slam.yml -p


Example: To list all known ICP algorithms:

.. code-block:: none

  mola-cli --rtti-children-of mp2p_icp::ICP_Base

  Listing children of class: mp2p_icp::ICP_Base
  mp2p_icp::ICP_GaussNewton
  mp2p_icp::ICP_Horn_MultiCloud
  mp2p_icp::ICP_OLAE


----

.. index::
   module: mola-launcher

C++ library: `mola-launcher`
#############################

See :ref:`mp2p_icp_grp`.
