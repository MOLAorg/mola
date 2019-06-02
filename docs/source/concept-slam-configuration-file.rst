.. _yaml_slam_cfg_file:

=============================================
SLAM system YAML configuration file format
=============================================


Refer to :ref:`demos` for real examples to launch and run SLAM systems.

File structure
--------------------

.. code-block:: yaml

    # my-slam-system.yml
    modules:
      - type: CLASS_NAME1       # mandatory
        name: INSTANCE_NAME1    # mandatory
        execution_rate: 20      # Hz (optional)
        verbosity_level: INFO   # DEBUG|INFO|WARN|ERROR
        params:                 # Other module-specific parameters
          var1: value1
          var2: value2
          # ...
      - type: CLASS_NAME2
        name: INSTANCE_NAME2
      # ...

Notes:
  - ``CLASS_NAME1``: The C++ class name of one of the modules registered in the MOLA system.
  - ``INSTANCE_NAME1``: Arbitrary name of this instance of the module. All names must be unique in a SLAM system.
