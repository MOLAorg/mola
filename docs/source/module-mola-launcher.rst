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
Refer to `the example launch files in mola_demos <https://github.com/MOLAorg/mola/tree/develop/mola_demos/mola-cli-launchs>`_ for real examples to launch and run SLAM systems.

-------------
SYNOPSIS
-------------

.. code-block:: none

    USAGE:

       mola-cli [OPTIONS] [config...]


    POSITIONALS:

       config TEXT ...
         Input YAML config file(s) (*.yaml). If more than one file is given,
         all of them are loaded and their `modules` sections are merged
         together into a single running system.

    OPTIONS:

       -h,  --help
         Print this help message and exit.

       -v,  --verbosity TEXT
         Verbosity level: ERROR|WARN|INFO|DEBUG (Default: INFO)

       -p,  --profiler
         Enable time profiler by default in all modules (Default: NO)

       --profiler-whole
         Enable whole-history time profiler in all modules (Default: NO). **DO
         NOT** use in production, only to benchmark short runs (unbounded
         memory usage)

       --rtti-list-all
         Loads all MOLA modules, then list all classes registered via
         mrpt::rtti, and exits.

       --rtti-children-of TEXT
         Loads all MOLA modules, then list all known classes that inherit from
         the given one, and exits.

       --list-modules
         Loads all MOLA modules, then list them. It also shows the list of
         paths in which the program looks for module dynamic libraries, then
         exits.

       --list-module-shared-dirs
         Finds all MOLA module source/shared directories, then list them. Paths
         can be added with the environment variable MOLA_MODULES_SHARED_PATH.


Notes:

  - Finer-control of the verbosity for individual modules is possible by using the `verbosity` variable in the YAML launch file, see: :ref:`yaml_slam_cfg_file`.
  - Each YAML file keeps the same self-contained structure described in
    :ref:`yaml_slam_cfg_file` (a top-level ``modules`` entry). When several
    files are given, ``mola-cli`` processes them in the order given on the
    command line, instantiating the ``modules`` of each one into the same
    running system. Module ``name``\ s must be unique across **all** the given
    files.

Example: Launching a SLAM system with performance details at end:

.. code-block:: none

  mola-cli kitti_lidar_slam.yml -p


Example: Launching a SLAM system whose sensors, pipeline, and visualization
are defined in separate, reusable files:

.. code-block:: none

  mola-cli sensors.yaml pipeline.yaml visualization.yaml


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

       mola-yaml-parser [OPTIONS] YAML_file


    POSITIONALS:

       YAML_file TEXT REQUIRED
         Input YAML file (required) (*.yml)

    OPTIONS:

       -h,  --help
         Print this help message and exit.

       --no-includes
         Disables solving YAML `$include{}`s (Default: NO)

       --no-cmd-runs
         Disables solving YAML `$(cmd)`s (Default: NO)

       --no-env-vars
         Disables solving YAML `${xxx}`s (Default: NO)


Example:

.. code-block:: none

    mola-yaml-parser --no-env-vars demos/kitti_lidar_slam.yml



----

.. index::
   module: mola-launcher

C++ library: `mola-launcher`
#############################

See :ref:`the mola_launcher C++ API <doxid-group__mola__launcher__grp>`.
