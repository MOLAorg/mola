.. _concepts_mola_paths:

=============================================
How MOLA finds its modules
=============================================

.. _concepts_module_lib_dir:

At start up, ``mola-cli`` loads every shared library whose file name contains ``mola``
(``*.so`` on Linux, ``*.dll`` on Windows) found in these directories, in this order:

1. The library output directory of the ``mola_launcher`` build (development builds).
2. The directories in the environment variable ``MOLA_MODULES_LIB_PATH``
   (``path1:path2:...``), for modules installed elsewhere.
3. The directories in ``LD_LIBRARY_PATH``. After sourcing a ROS 2 workspace
   (``source install/setup.bash``), it includes the ``lib`` directory of every installed
   package, so all MOLA modules are found with no further configuration.

Loading a library registers its classes, which can then be used as the ``type`` of a module
in a :ref:`system YAML file <yaml_slam_cfg_file>`. A library that cannot be loaded (e.g. a
missing dependency) is skipped with a warning.

To see the search paths and the modules found, and the classes they provide:

.. code-block:: bash

    mola-cli --list-modules
    mola-cli --rtti-list-all

To refer to a file installed by a ROS 2 package from a YAML file, use its share directory:
``$(ros2 pkg prefix <package>)/share/<package>/...`` (see :ref:`yaml_cmd`).
