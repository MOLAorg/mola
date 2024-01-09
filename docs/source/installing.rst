.. _installing:

======================
Installing
======================

.. contents:: :local:


From ROS 2 repositories
------------------------

**Recommended**: This is the easiest way to install MOLA.

In Debian/Ubuntu systems, activate your ROS environment (``setup.bash``) if not done automatically 
in your ``~./bashrc`` file, then just run:

.. code-block:: bash

    sudo apt install ros-$ROS_DISTRO-mola

Check the `build status table <https://github.com/MOLAorg/mola#build-matrix-status>`_ to find out
what MOLA version is available for your ROS distribution.

Build from sources
----------------------

Building tools
~~~~~~~~~~~~~~~~~
MOLA uses ``colcon`` so you need to `install it first <https://colcon.readthedocs.io/en/released/user/installation.html>`_.


Get the sources
~~~~~~~~~~~~~~~~~

Clone the git repositories, including the submodules:

.. code-block:: bash

 mkdir ~/ros2_mola_ws/src/ 
 cd ~/ros2_mola_ws/src/
 # Get MRPT2 (until we get the latest version on rosdistro)
 git clone https://github.com/MRPT/mrpt.git mrpt2 --recursive

 # Main MOLA modules:
 git clone https://github.com/MOLAorg/mola_common.git
 git clone https://github.com/MOLAorg/mp2p_icp.git --recursive
 git clone https://github.com/MOLAorg/mola.git --recursive

 # MOLA lidar odometry package:
 # not published yet!
 #git clone https://github.com/MOLAorg/mola_lidar_odometry.git --recursive

Dependencies
~~~~~~~~~~~~~~~~~

Make sure you have all dependencies installed (make sure of having `rosdep already installed <https://wiki.ros.org/rosdep>`_):

.. code-block:: bash

 cd ~/ros2_mola_ws/src/
 rosdep install --from-paths src --ignore-src -r -y


Build and test
~~~~~~~~~~~~~~~~~

Now, compile as usual with colcon:

.. code-block:: bash

 cd ~/ros2_mola_ws/src/
 colcon  build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release


Next, activate the new environment and check if all new modules are visible:

.. code-block:: bash

 cd ~/ros2_mola_ws/src/
 . install/setup.bash

 # For example, let's launch the mm map viewer:
 mm-viewer

