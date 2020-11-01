.. _mola_cmake_commands:

=====================
MOLA CMake functions
=====================

The following CMake functions are defined by `mola-common`.


`find_mola_package()`
-----------------------------------

.. code-block:: cmake

    # find_mola_package(package_name)
    #
    # Does nothing if the target is known at build time, or issues the corresponding
    # standard CMake find_package().


`mola_add_executable()`
-----------------------------------

.. code-block:: cmake

    # mola_add_executable(
    #	TARGET name
    #	SOURCES ${SRC_FILES}
    #	[LINK_LIBRARIES lib1 lib2]
    #	)
    #
    # Defines a MOLA executable



`mola_add_library()`
-----------------------------------

.. code-block:: cmake

    # mola_add_library(
    #	TARGET name
    #	SOURCES ${SRC_FILES}
    #	[PUBLIC_LINK_LIBRARIES lib1 lib2 ...]
    #	[PRIVATE_LINK_LIBRARIES lib3 lib4 ...]
    # [CMAKE_DEPENDENCIES dep1 dep2...]
    #	)
    #
    # Defines a MOLA library. CMAKE_DEPENDENCIES enumerates those cmake packages
    # that must be find_package()'d in this library's xxx-config.cmake script.


`mola_find_package_or_return()`
-----------------------------------

.. code-block:: cmake

    # mola_find_package_or_return(package_name)
    #
    # Calls find_package(package_name QUIET), and if it is not found, prints a
    # descriptive message and call "return()" to exit the current cmake script.
