.. _mola_cmake_functions:

=========================
MOLA CMake Configuration
=========================

This page documents the main CMake utilities and helper functions provided by the
`mola_common` package. These functions standardize build and installation behavior across all
MOLA modules and ensure consistent configuration for ROS 2 packages built with **colcon** and **CMake**.


.. contents::
   :depth: 1
   :local:
   :backlinks: none


Overview
========

The file :file:`mola_cmake_functions.cmake` is installed as part of the `mola_common` package.
It is referenced in the generated configuration file :file:`mola_common-config.cmake`,
which is automatically created from the template :file:`mola_common-config.cmake.in`
during the build process.

When a user runs:

.. code-block:: bash

   find_package(mola_common REQUIRED)

CMake (or colcon) locates `mola_common` via its package registry or via the ROS 2 ament index.
Internally, this results in `mola_common-config.cmake` being found, which in turn
includes :file:`mola_cmake_functions.cmake`, making the helper functions and macros defined
in this file available to any downstream project.


General Build Configuration
===========================

The following setup actions are performed automatically when including
:file:`mola_cmake_functions.cmake`:

* **Exported Symbols on Windows:**
  Ensures all symbols are exported by default on Windows builds (via `CMAKE_WINDOWS_EXPORT_ALL_SYMBOLS`).

* **Word Size Detection:**
  Detects whether the build is targeting a 32-bit or 64-bit platform, setting:
  
  .. code-block:: cmake
  
     MOLA_WORD_SIZE  # either 32 or 64

* **Standard Output Directories:**  
  Sets the default locations for build outputs, ensuring consistency across modules:

  .. code-block:: cmake

     set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/lib/")
     set(CMAKE_LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/lib/")
     set(CMAKE_RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/bin/")

* **Compiler Identification:**
  Detects and standardizes compiler name identifiers through:
  
  .. code-block:: cmake
  
     set(MOLA_COMPILER_NAME <compiler_id>)

  On MSVC systems, a normalized `msvcXYZ` label is generated.

* **Project Organization:**
  Enables folder grouping in IDE project views with:

  .. code-block:: cmake

     set_property(GLOBAL PROPERTY USE_FOLDERS ON)
     set_property(GLOBAL PROPERTY PREDEFINED_TARGETS_FOLDER "CMakeTargets")

* **Consistent Library Naming:**
  Forces the `lib` prefix on all static, shared, and import libraries, including on Windows:

  .. code-block:: cmake
  
     set(CMAKE_SHARED_LIBRARY_PREFIX "lib")
     set(CMAKE_IMPORT_LIBRARY_PREFIX "lib")
     set(CMAKE_STATIC_LIBRARY_PREFIX "lib")


Defined Functions
=================

The following CMake functions are defined and exported by `mola_common` through this file.

``mola_add_library()``
----------------------

.. code-block:: cmake

   mola_add_library(
       TARGET <name>
       SOURCES <src1> [<src2> ...]
       [PUBLIC_LINK_LIBRARIES <lib1> <lib2> ...]
       [PRIVATE_LINK_LIBRARIES <lib3> <lib4> ...]
       [CMAKE_DEPENDENCIES <dep1> <dep2> ...]
   )

Creates and configures a shared library target within the MOLA build system.

This function wraps the standard `add_library()` command, applying MOLA’s
standard build flags, dependency configuration, and compiler options via
:meth:`mola_set_target_build_options()` and :meth:`mola_configure_library()`.

**Arguments**

* ``TARGET`` - Name of the library target to create.  
* ``SOURCES`` - List of source files for the library.  
* ``PUBLIC_LINK_LIBRARIES`` - List of public dependencies that should be visible
  to consumers of the library.  
* ``PRIVATE_LINK_LIBRARIES`` - List of internal dependencies used only for
  this target’s own build.  
* ``CMAKE_DEPENDENCIES`` - List of CMake packages this library depends on
  (used internally by `mola_configure_library()`).

**Example**

.. code-block:: cmake

   mola_add_library(
       TARGET mola_geometry
       SOURCES src/geometry.cpp src/math_utils.cpp
       PUBLIC_LINK_LIBRARIES mola_common Eigen3::Eigen
       PRIVATE_LINK_LIBRARIES fmt
       CMAKE_DEPENDENCIES Eigen3
   )


``mola_add_executable()``
-------------------------

.. code-block:: cmake

   mola_add_executable(
       TARGET <name>
       SOURCES <src1> [<src2> ...]
       [PUBLIC_LINK_LIBRARIES <lib1> <lib2> ...]
       [PRIVATE_LINK_LIBRARIES <lib3> <lib4> ...]
       [CMAKE_DEPENDENCIES <dep1> <dep2> ...]
   )

Creates and configures an executable target with the standard MOLA conventions.

This function wraps the standard `add_executable()` command and applies uniform
compiler options and dependency handling via
:meth:`mola_set_target_build_options()` and :meth:`mola_configure_app()`.

**Arguments**

* ``TARGET`` - Name of the executable target.  
* ``SOURCES`` - List of source files for the executable.  
* ``PUBLIC_LINK_LIBRARIES`` - Libraries linked publicly to the target.  
* ``PRIVATE_LINK_LIBRARIES`` - Libraries linked privately (internal use only).  
* ``CMAKE_DEPENDENCIES`` - CMake packages required to configure this executable.

**Example**

.. code-block:: cmake

   mola_add_executable(
       TARGET mola_viewer
       SOURCES src/main_viewer.cpp
       PUBLIC_LINK_LIBRARIES mola_common
       PRIVATE_LINK_LIBRARIES Qt5::Widgets
       CMAKE_DEPENDENCIES Qt5
   )


``mola_add_test()``
-------------------

.. code-block:: cmake

   mola_add_test(
       TARGET <name>
       SOURCES <src1> [<src2> ...]
       [LINK_LIBRARIES <lib1> <lib2> ...]
   )

Defines a test executable and registers it with CTest under MOLA’s testing framework.

This function wraps `add_executable()` and `add_test()` to automatically
configure testing conventions, build options, and linking for unit and integration tests.

**Arguments**

* ``TARGET`` - Name of the test target.  
* ``SOURCES`` - List of test source files.  
* ``LINK_LIBRARIES`` - Libraries to link against for the test executable.

After creation, the function calls `add_test(NAME <target> COMMAND <target>)`
to register the test automatically.

**Example**

.. code-block:: cmake

   mola_add_test(
       TARGET test_geometry
       SOURCES tests/test_geometry.cpp
       LINK_LIBRARIES mola_geometry mola_common
   )

This defines a test executable `test_geometry` and adds it to the build’s
CTest configuration. Running `ctest` will automatically include it in
the suite of MOLA tests.


``find_mola_package()``
-----------------------

.. code-block:: cmake

   find_mola_package(package_name)

Attempts to locate another MOLA package and makes its exported targets available.
If the package’s targets are already known (i.e., defined in the current build),
this function does nothing. Otherwise, it falls back to a standard CMake `find_package()` call.

This is the preferred method for inter-package discovery within the MOLA build system,
as it ensures uniform handling for both local and system-installed MOLA components.


``mola_version_to_hexadecimal()``
---------------------------------

.. code-block:: cmake

   mola_version_to_hexadecimal(<OUT_VAR> <IN_VERSION>)

Converts a version string (in the form `MAJOR.MINOR.PATCH`) into a hexadecimal
version code suitable for compile-time comparisons or embedding into generated headers.

This macro extracts the three numeric components of a version string,
computes a hexadecimal integer representing them, and stores the result
in the specified output variable.

**Arguments**

* ``OUT_VAR`` - Name of the variable that will receive the hexadecimal result.  
* ``IN_VERSION`` - Input version string, e.g. `"1.2.3"`.

**Behavior**

The macro parses the version string into three numeric parts:

.. code-block::

   MAJOR = 1
   MINOR = 2
   PATCH = 3

It then performs the following conversion:

.. code-block::

   HEX_VALUE = (MAJOR << 16) + (MINOR << 8) + PATCH

and stores the result in hexadecimal format as a string beginning with `0x`.

**Example**

.. code-block:: cmake

   mola_version_to_hexadecimal(MY_VERSION_HEX "1.2.3")
   message(STATUS "Version in hex: ${MY_VERSION_HEX}")

This will output:

.. code-block::

   -- Version in hex: 0x10203

**Typical Use Case**

This macro is commonly used in MOLA’s CMake configuration process
to produce encoded version constants that can be passed to C++ code
to compile conditionally depending on some other library version,
following the philosophy of "one source for all versions".


Installation and Usage
======================

To make use of these functions in your own project:

.. code-block:: cmake

   find_package(mola_common REQUIRED)
   include(mola_cmake_functions)

Then invoke the desired utilities in your project’s `CMakeLists.txt`.

Example:

.. code-block:: cmake

   add_library(my_module src/main.cpp)
   mola_common_setup_target(my_module)
   mola_common_export_dependencies(my_module DEPENDS mola_common)
   find_mola_package(mola_geometry)

This ensures your module is correctly built, linked, and installed within
the larger MOLA ecosystem.


Summary
=======

The :file:`mola_cmake_functions.cmake` file is a foundational part of the
MOLA CMake infrastructure.  
It guarantees consistent compiler settings, output layout, dependency management,
and naming conventions across all packages using `mola_common`, simplifying both
local development and ROS 2 colcon integration.
