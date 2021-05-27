.. _yaml_slam_cfg_file:

=============================================
SLAM system YAML configuration file format
=============================================

Refer to :ref:`demos` for real examples to launch and run SLAM systems.

File structure
================

A SLAM system is defined via a YAML configuration file, comprising a top-level
``modules`` entry, with one or more children elements with the following
required fields:

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

.. _yaml_extensions:

YAML extensions
=================
MOLA supports the following extensions to standard YAML to ease the maintenance
of configuration files:

Include other files: ``$include{xxx}``
-----------------------------------------
The contents of any entry with the syntax ``$include{/path/to/other/file.yml}``
will be replaced by the actual contents of the given file, which should be given
in either absolute path, or relative to the current YAML file being parsed.

.. code-block:: yaml

    modules:
      - type: CLASS_NAME1       # mandatory
        #...
        params:                 # Other module-specific parameters
          $include{$(mola-dir module-name)/params/xxxx.yaml}
        config:
          $include{params/xxxx.yaml}


Replace environment variables: ``${foo}``
-----------------------------------------------
If any YAML content contains the pattern ``${foo}``, it is replaced by the
current value of the given environment variable. A fatal error is raised if the
variable is not defined.


Replace output of external process: ``$(cmd)``
-----------------------------------------------
The pattern ``$(cmd arg1 arg2...)`` is replaced by the console output of running
the given command with the given arguments. See for example its usage together
with ``$include{xxx}`` above.
