.. _mola-launcher:

======================
Module: mola-launcher
======================

.. toctree::
  :maxdepth: 2

----

.. index::
   single: mola-cli

.. _mola_cli:

Application: ``mola-cli``
##########################

``mola-cli`` is the most-common entry point to the MOLA system for users.
Refer to :ref:`demos` for real examples to launch and run SLAM systems.

-------------
SYNOPSIS
-------------

.. code-block:: none

    USAGE:

       mola-cli  [--list-module-shared-dirs] [--list-modules]
                 [--rtti-children-of <mp2p_icp::ICP_Base>] [--rtti-list-all]
                 [--profiler-whole] [-p] [-v <INFO>] [-c <demo.yml>] [--]
                 [--version] [-h]


    Where:

       --list-module-shared-dirs
         Finds all MOLA module source/shared directories, then list them. Paths
         can be added with the environment variable MOLA_MODULES_SHARED_PATH.

       --list-modules
         Loads all MOLA modules, then list them. It also shows the list of
         paths in which the program looks for module dynamic libraries, then
         exits.

       --rtti-children-of <mp2p_icp::ICP_Base>
         Loads all MOLA modules, then list all known classes that inherit from
         the given one, and exits.

       --rtti-list-all
         Loads all MOLA modules, then list all classes registered via
         mrpt::rtti, and exits.

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

  mola-cli kitti_lidar_slam.yml -p


Example: To list all known ICP algorithms:

.. code-block:: none

  mola-cli --rtti-children-of mp2p_icp::ICP_Base

  Listing children of class: mp2p_icp::ICP_Base
  mp2p_icp::ICP_GaussNewton
  mp2p_icp::ICP_Horn_MultiCloud
  mp2p_icp::ICP_OLAE


----

.. index::
   single: mola-dir

.. _mola_dir:

Application: ``mola-dir``
##########################

``mola-dir`` is a CLI program that finds the shared directory
(see :ref:`concepts_module_shared_dir`) of a module.
It is most-commonly useful inside evaluation expression of MOLA definition files (:ref:`yaml_slam_cfg_file`).

-------------
SYNOPSIS
-------------

.. code-block:: none

    USAGE:

       mola-dir  <module_name>


Note: To list all known shared directories of modules, use:

.. code-block:: none

    mola-cli --list-module-shared-dirs

----

.. index::
   single: mola-yaml-parser

.. _mola_yaml_parser:

Application: ``mola-yaml-parser``
###################################

``mola-yaml-parser`` is a CLI program that parses YAML files using MOLA-specific
extensions (see :ref:`yaml_extensions`), and outputs the result to ``cout``.

-------------
SYNOPSIS
-------------

.. code-block:: none

    USAGE:

       mola-yaml-parser  [--no-env-vars] [--no-cmd-runs] [--no-includes] [--]
                         [--version] [-h] <YAML files>

    Where:

       --no-env-vars
         Disables solving YAML `${xxx}`s (Default: NO)

       --no-cmd-runs
         Disables solving YAML `$(cmd)`s (Default: NO)

       --no-includes
         Disables solving YAML `$include{}`s (Default: NO)

       --,  --ignore_rest
         Ignores the rest of the labeled arguments following this flag.

       --version
         Displays version information and exits.

       -h,  --help
         Displays usage information and exits.

       <YAML files>
         (required)  Input YAML file (required) (*.yml)


Example:

.. code-block:: none

    mola-yaml-parser --no-env-vars demos/kitti_lidar_slam.yml



----

.. index::
   module: mola-launcher

C++ library: `mola-launcher`
#############################

See :ref:`mp2p_icp_grp`.
