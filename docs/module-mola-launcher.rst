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

       mola-cli  [--profiler-whole] [-p] [-v <INFO>] -c <demo.yml> [--]
                 [--version] [-h]


    Where:

       --profiler-whole
         Enable whole-history time profiler in all modules (Default: NO). **DO
         NOT** use in production, only to benchmark short runs (unbounded
         memory usage)

       -p,  --profiler
         Enable time profiler by default in all modules (Default: NO)

       -v <INFO>,  --verbosity <INFO>
         Verbosity level: ERROR|WARN|INFO|DEBUG (Default: INFO)

       -c <demo.yml>,  --config <demo.yml>
         (required)  Input YAML config file (required) (*.yml)

       --,  --ignore_rest
         Ignores the rest of the labeled arguments following this flag.

       --version
         Displays version information and exits.

       -h,  --help
         Displays usage information and exits.

Notes:

  - Finer-control of the verbosity for individual modules is possible by using the `verbosity` variable in the YAML launch file, see: :ref:`yaml_slam_cfg_file`.


----

.. index::
   module: mola-launcher

C++ library: `mola-launcher`
#############################

XX
