.. _building:

=============
Building
=============

Get the sources
-------------------

Clone the main project git superproject with:

.. code-block:: bash

    git clone --recurse-submodules https://github.com/MOLAorg/mola.git

Build dependencies
-----------------------
- A C++17 compiler. Either gcc-7, clang-4 or MSVC 2017 are good.
- CMake >=3.4
- Mandatory libraries:
   - MRPT >= 1.9.9
   - GTSAM >= 4.0.0
   - yaml-cpp (will use an embedded version if not found in the system)
- Optional libraries:
   - opencv
   - ros1

Ubuntu
========
Install all dependencies in Ubuntu systems with:

.. code-block:: bash

    # MRPT >1.9.9, for now from this PPA (or build from sources if preferred):
    sudo add-apt-repository ppa:joseluisblancoc/mrpt
    sudo apt update
    sudo apt install libmrpt-dev mrpt-apps

    # GTSAM >=4.0.0, from this PPA, or build it from sources:
    sudo add-apt-repository ppa:joseluisblancoc/gtsam-develop
    sudo apt update
    sudo apt install libgtsam-dev

    # Boost, yaml-cpp, etc:
    sudo apt install libboost-serialization-dev libboost-system-dev \
      libboost-filesystem-dev libboost-thread-dev libboost-program-options-dev \
      libboost-date-time-dev libboost-timer-dev libboost-chrono-dev \
      libboost-regex-dev
    sudo apt install libyaml-cpp-dev


Compile
---------------------
Classic cmake stuff:

.. code-block:: bash

    cmake -H. -Bbuild
    cd build
    cmake --build .

Run tests
----------------------
To make sure the system works, you can run unit tests with:

.. code-block:: bash

    make test
