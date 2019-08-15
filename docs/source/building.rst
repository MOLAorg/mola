.. _building:

======================
Building
======================

1. First-time build
====================

Get the sources
-------------------

Clone the main project git superproject with:

.. code-block:: bash

    git clone --recurse-submodules https://github.com/MOLAorg/mola.git

Build dependencies
-----------------------
- A C++17 compiler. Either gcc-7, clang-4 or MSVC 2017 are good.
- CMake >=3.9
- Mandatory libraries:
   - MRPT >= 1.9.9
   - GTSAM >= 4.0.0
   - yaml-cpp (will use an embedded version if not found in the system)
- Optional libraries:
   - opencv
   - ros1: roscpp, sensor-msgs

-------
Ubuntu
-------

Install all dependencies in Ubuntu systems with:

.. code-block:: bash

    # **ONLY** if you use Ubuntu 16.04: 
    # 1) Upgrade to gcc-7: Follow instructions in
    # https://gist.github.com/jlblancoc/99521194aba975286c80f93e47966dc5
    # 2) Upgrade to CMake 2.9 or newer. Download and build CMake from sources.


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

    # ROS1 (Optional)
    # To install from the official Ubuntu repository (under /usr):
    sudo apt install libroscpp-dev libsensor-msgs-dev
    # Alternatively, install a particular ROS distribution (under /opt) and
    # source the corresponding setup.bash file before invoking cmake:


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

2. Add MOLA to PATH
======================

Add the `build/bin` build subdirectory to the environment variable `PATH` to
ease the invocation of MOLA commands and demos.
For example, if `MOLA_BINARY_DIR` is `$HOME/code/mola`, run the next command to
automatically add its build directory to PATH when opening a console:

.. code-block:: bash

    echo 'set $PATH=$PATH:$HOME/code/mola/build/bin' >> ~/.bashrc


3. Updating sources for rebuilding
====================================

Since MOLA is under heavy development, it is expected that cloned repositories
quickly get out of date.

To get the latest version of all modules, and clone recently added modules, run:

.. code-block:: bash

    cd MOLA_SOURCE_ROOT_DIR
    git pull
    git submodule update --init

then rebuild as usual with `make`, `cmake --build .`, etc.
