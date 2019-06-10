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



`mola_add_library()`
-----------------------------------

.. code-block:: cmake

    # mola_add_library(
    #	TARGET name
    #	SOURCES ${SRC_FILES}
    #	[PUBLIC_LINK_LIBRARIES lib1 lib2]
    #	[PRIVATE_LINK_LIBRARIES lib3 lib4]
    #	)
    #
    # Defines a MOLA library


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
