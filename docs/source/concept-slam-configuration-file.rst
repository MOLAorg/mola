.. _yaml_slam_cfg_file:

=============================================
SLAM system YAML configuration file format
=============================================

Refer to `the example launch files in mola_demos <https://github.com/MOLAorg/mola/tree/develop/mola_demos/mola-cli-launchs>`_ for real examples to launch and run SLAM systems.

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

.. _yaml_multiple_files:

Launching from multiple files
================================

:ref:`mola_cli` accepts more than one YAML file on the command line:

.. code-block:: none

  mola-cli sensors.yaml pipeline.yaml visualization.yaml

Each file keeps the exact same self-contained structure described above (a
top-level ``modules`` entry). ``mola-cli`` loads and pre-processes each file
independently -- so ``$include{}``, ``$()`` and ``${}`` paths inside each file
are still resolved relative to *that* file's own directory -- and then merges
all of their ``modules`` lists into a single running system, in the order the
files were given on the command line.

This is useful to split a SLAM system definition into reusable building
blocks (e.g. one file per sensor, plus a shared pipeline and visualization
file). As with a single file, all module ``name``\ s must be unique across
**all** the given files.


.. _yaml_extensions:

YAML pre-processing
===================

Before being parsed as standard YAML, every configuration file loaded by MOLA
is run through a lightweight pre-processor that expands special ``$``\ -prefixed
expressions.  The three expansion passes are always applied in the following
fixed order:

1. :ref:`yaml_include` - ``$include{path}``, plus the :ref:`yaml_import` and
   :ref:`yaml_define` map directives
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


.. _yaml_import:

Import a base file and override entries: ``$import``
------------------------------------------------------

While ``$include{}`` replaces a *whole* node with the contents of a file, the
``$import`` **map key** lets you load a file as a **base** and then **override**
only a few of its entries inline.  A map containing an ``$import`` key is
replaced by the **deep-merge** of the imported file(s) with the map's
*remaining* keys overlaid on top:

- The ``$import`` value is either a single file path (scalar) or a **sequence**
  of paths.  When several files are given, they are merged in order, so a later
  file overrides an earlier one.
- The **sibling** keys of ``$import`` are then overlaid and therefore **override**
  the imported base.  Nested maps are merged **deeply**, so you can override a
  single deep key without restating its whole subtree; scalars and sequences are
  replaced wholesale.
- Paths follow the same rules as ``$include{}`` (absolute or relative to the
  current file's directory; the path expression itself is pre-processed by the
  ``$()`` / ``${}`` passes), and the same **circular-reference detection** applies.

This is the recommended way to share a common parameter block across several
near-identical launch files while tweaking just a few values, instead of
duplicating the whole block:

.. code-block:: yaml

    modules:
      - type: CLASS_NAME1
        name: instance1
        params:
          $import: shared-params.yaml   # base (a path, or a sequence of paths)
          max_rate_hz: 10.0             # override a single entry of the base
          nested:
            gain: 2.0                   # deep-override one nested key (siblings kept)

Equivalent multi-file base, where ``overrides.yaml`` wins over ``base.yaml``:

.. code-block:: yaml

    params:
      $import:
        - base.yaml
        - overrides.yaml
      extra_key: 123                    # still overrides both imported files

.. note::

   ``$import`` is resolved in the same (first) pass as ``$include{}``.  Use
   ``$include{}`` when a node *is* the file; use ``$import`` when you want the
   file as a base **plus** local overrides.


.. _yaml_define:

Set variables for a subtree: ``$define``
------------------------------------------

Most shared MOLA parameter files already expose their tunable settings as
``${VAR|default}`` hooks (see :ref:`yaml_vars`).  The ``$define`` **map key**
lets a file **bind those variables** for the subtree of the map it appears in --
**including the files pulled in by a sibling** ``$import`` **or**
``$include{}``.

Its value is a map of ``NAME: VALUE`` pairs:

.. code-block:: yaml

    modules:
      - type: mola::LidarOdometry
        name: lidar_odom

        $define:
          MOLA_DESKEW_METHOD: "MotionCompensationMethod::IMU"
          MOLA_LO_INITIAL_LOCALIZATION_METHOD: "InitLocalization::PitchAndRollFromIMU"

        $import: lidar3d-default.yaml

Key properties:

- **Priority is** ``environment > $define > inline |default``.  ``$define`` acts
  as a *file-level default*: a variable exported on the command line still
  overrides it, so ``MOLA_DESKEW_METHOD=... mola-cli my-system.yaml`` keeps
  working as before.
- **It is not** ``setenv``.  The bindings are scoped to the YAML subtree and
  never leak into the real process environment, into ``$()`` sub-processes, or
  into a sibling module's ``$import``.
- **Scope is the map it appears in, and everything below it** -- both the
  imported files and the plain sibling keys.  A nested ``$define`` (in this file
  or in an imported one) **shadows** an outer one for its own subtree.
- ``$define`` **values are themselves expanded**, so they may be built from
  ``${}`` / ``$()`` expressions.  They are expanded against the **outer** scope
  only: entries of the same ``$define`` block cannot reference each other (this
  keeps the result independent of the order in which they are written).
- The ``$define`` key is stripped from the resulting configuration.

.. tip::

   ``$define`` is the way to patch a value that lives **inside a YAML sequence**
   of an imported file.  ``$import``'s deep-merge replaces sequences *wholesale*
   (see :ref:`yaml_import`), so overriding one field of one list item otherwise
   forces you to duplicate the entire sequence verbatim.  If the imported file
   exposes that field as a ``${VAR|default}`` hook, a one-line ``$define``
   replaces the whole copy-paste.


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
   arbitrary name/value pairs at load time.  A :ref:`yaml_define` block binds
   variables into this same slot, from the YAML file itself.
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
   * - ``$import`` (map key)
     - Imported file(s), with sibling keys overriding
     - 1st
     - n/a (a map key, not a scalar token)
   * - ``$define`` (map key)
     - Nothing (binds ``${NAME}`` vars for its subtree)
     - 1st
     - n/a (a map key, not a scalar token)
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
executable to print each external file (``$include{}`` or ``$import``) as it is
resolved:

.. code-block:: bash

    VERBOSE=1 ros2 launch my_slam_system slam.launch.py

Each resolved file prints a line of the form::

    [mola::parse_yaml] loading external YAML: "/absolute/path/to/file.yaml"