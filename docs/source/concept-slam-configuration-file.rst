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

- ``CLASS_NAME1``: The C++ class name of one of the modules registered in the
  MOLA system.
- ``INSTANCE_NAME1``: Arbitrary name of this instance of the module. All names
  must be unique in a SLAM system.


.. _yaml_extensions:

YAML pre-processing
===================

Before being parsed as standard YAML, every configuration file loaded by MOLA
is run through a lightweight pre-processor that expands special ``$``\ -prefixed
expressions.  The three expansion passes are always applied in the following
fixed order:

1. :ref:`yaml_include` - ``$include{path}``
2. :ref:`yaml_cmd` - ``$(command)``
3. :ref:`yaml_vars` - ``${VAR}`` / ``${VAR|default}``

This ordering means that:

- ``$include{}`` can pull in files whose paths are themselves built with
  ``$()`` or ``${}`` tokens.
- ``$()`` command output can reference environment variables already present in
  the environment at parse time.
- ``${}`` variable expansion sees the fully assembled text - including the
  content of all included files - and can therefore reference variables that
  are defined in those files.

**Comments are never expanded.**  Any ``$include{}``, ``$()``, or ``${}``
token that appears after a ``#`` comment marker on the same line is left
completely unchanged.  This applies to both inline comments and full-line
comments:

.. code-block:: yaml

    v: 1          # $(echo this-is-never-run) ${ALSO_NEVER_EXPANDED}
    # ${THIS_IS_ALSO_SAFE}


.. _yaml_include:

Include other files: ``$include{path}``
-----------------------------------------

A YAML scalar whose value matches the pattern ``$include{/path/to/file.yaml}``
is replaced by the full contents of the referenced file.

- Paths may be **absolute** or **relative**.  A relative path is resolved
  against the directory of the file that contains the ``$include{}`` directive.
  Includes within the included file are in turn resolved relative to *that*
  file's own directory, so any depth of nesting works correctly.
- The path expression inside ``$include{}`` is itself pre-processed by the
  ``$()`` and ``${}`` passes before the file is opened, so the path may be
  constructed dynamically.
- **Circular includes are detected.**  If a file attempts to include itself
  (directly or transitively), a clear error is raised rather than looping
  indefinitely.

.. code-block:: yaml

    modules:
      - type: CLASS_NAME1
        params:
          # Include a file whose path is built from an environment variable:
          $include{${MY_CONFIG_DIR}/sensor_params.yaml}

        config:
          # Include a file relative to this file's own directory:
          $include{params/lidar_odometry.yaml}

          # Include a file whose path comes from a shell command
          # (e.g. a ROS 2 package share directory):
          $include{$(ros2 pkg prefix mola_lidar_odometry)/share/mola_lidar_odometry/config/lo_default.yaml}

.. note::

   The ``$include{}`` directive replaces the *entire scalar node* it appears
   in.  The typical usage is therefore as the sole value of a map key (``params:``,
   ``config:``, etc.), where the included file provides the child map:

   .. code-block:: yaml

       params:
         $include{my_params.yaml}   # ← my_params.yaml contributes all keys here


.. _yaml_cmd:

Run an external command: ``$(command)``
-----------------------------------------

The pattern ``$(command arg1 arg2 ...)`` is replaced by the **standard output**
of the command, with leading and trailing whitespace stripped.  Multi-line
output is preserved; only the boundary whitespace is removed.

A non-zero exit code causes a fatal error.

This is most commonly used together with ``$include{}`` to locate files inside
a ROS 2 package:

.. code-block:: yaml

    params:
      $include{$(ros2 pkg prefix my_package)/share/my_package/config/default.yaml}

It can also set scalar values directly:

.. code-block:: yaml

    hostname: $(hostname)
    git_sha:  $(git -C ${MY_SRC_DIR} rev-parse --short HEAD)

.. warning::

   Shell commands are executed at **parse time**, with the same privileges as
   the process loading the YAML file.  Only use ``$()`` with commands whose
   output you trust.


.. _yaml_vars:

Variables and environment: ``${VAR}`` / ``${VAR|default}``
------------------------------------------------------------

The pattern ``${NAME}`` is replaced by a value looked up in the following
order (first match wins):

1. **Environment variable** - the current value of ``getenv("NAME")``.
2. **Built-in token** ``CURRENT_YAML_FILE_PATH`` - expands to the directory
   of the file currently being parsed (or the value of
   ``YAMLParseOptions::includesBasePath`` when called programmatically).
3. **Caller-supplied variables** - entries in the
   ``YAMLParseOptions::variables`` map, which allows C++ code to inject
   arbitrary name/value pairs at load time.
4. **Inline default** - if the token has the form ``${NAME|default_value}``,
   the literal text after ``|`` is used as a fallback.  The default may be
   empty (``${NAME|}`` resolves to an empty string).
5. **Error** - if none of the above matched, a fatal error is raised.

.. code-block:: yaml

    # Expand an environment variable (fatal if MY_ROBOT is unset):
    robot_name: ${MY_ROBOT}

    # Use a default value when the variable is not set:
    log_dir: ${LOG_DIR|/tmp/mola_logs}

    # Empty-string default - never raises an error:
    optional_suffix: ${SUFFIX|}

    # Refer to the directory of this file (useful for sibling-relative paths):
    calibration_file: ${CURRENT_YAML_FILE_PATH}/calib/lidar.yaml

    # The same variable may appear multiple times in one scalar:
    topic: /${ROBOT_NS}/sensors/${SENSOR_NAME}/points

.. note::

   Substituted values are **not re-expanded**.  If an environment variable
   itself contains a ``${...}`` or ``$(...)`` token, that token is emitted
   verbatim and not processed a second time.  This prevents accidental
   double-expansion and avoids infinite loops.

.. tip::

   You can combine all three mechanisms.  For example, the following snippet
   locates a calibration file in a ROS 2 package, using an environment variable
   to choose the robot variant, with a safe default:

   .. code-block:: yaml

       calibration:
         $include{$(ros2 pkg prefix my_robot_config)/share/my_robot_config/calib/${ROBOT_VARIANT|default}/lidar.yaml}


Mathematical formulas: ``$f{expr}``
-------------------------------------

The pattern ``$f{expr}`` evaluates a mathematical expression using the
`ExprTk library <https://github.com/ArashPartow/exprtk>`_ (via
`mrpt-expr <https://docs.mrpt.org/reference/latest/group_mrpt_expr_grp.html>`_).
Refer to the ExprTk documentation for the full list of built-in functions and
operators.  User-defined variables may be available depending on the context in
which the YAML file is parsed.

.. code-block:: yaml

    half_fov_rad: $f{deg2rad(45.0)}
    diagonal:     $f{sqrt(3.0^2 + 4.0^2)}


.. _yaml_preprocessing_order:

Processing order summary
--------------------------

The table below summarises which pass handles each token, and what happens
when a token falls inside a comment:

.. list-table::
   :header-rows: 1
   :widths: 20 30 25 25

   * - Token
     - Replaced with
     - Pass
     - In a ``#`` comment
   * - ``$include{path}``
     - Contents of ``path``
     - 1st
     - Left verbatim
   * - ``$(command)``
     - stdout of ``command``
     - 2nd
     - Left verbatim (command **not** run)
   * - ``${VAR}``
     - Value of ``VAR`` (error if unset)
     - 3rd
     - Left verbatim
   * - ``${VAR|default}``
     - Value of ``VAR``, or ``default``
     - 3rd
     - Left verbatim
   * - ``$f{expr}``
     - Result of mathematical expression
     - Context-dependent
     - Left verbatim


Debugging pre-processing
--------------------------

Set the environment variable ``VERBOSE=1`` before launching any MOLA
executable to print each ``$include{}`` directive as it is resolved:

.. code-block:: bash

    VERBOSE=1 ros2 launch my_slam_system slam.launch.py

Each include prints a line of the form::

    [mola::parse_yaml] $include{ "/absolute/path/to/file.yaml" }
    [mola::parse_yaml] $include done: "/absolute/path/to/file.yaml"