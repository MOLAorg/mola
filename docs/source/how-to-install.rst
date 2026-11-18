.. _installing:

=====================
Install
=====================

How to install all MOLA modules:

.. dropdown:: From ROS 2 repositories
    :open:
    :icon: download

    **Recommended**: This is the easiest way to install MOLA.

    In Debian/Ubuntu systems, activate your ROS environment (``setup.bash``) if not done automatically
    in your ``~/.bashrc`` file, then just run:

    .. code-block:: bash

        # Install core MOLA modules and 3D LiDAR odometry
        # (includes the readers for ROS 1/2 bags, rawlogs, KITTI, MulRan, ...):
        sudo apt install \
         ros-$ROS_DISTRO-mola \
         ros-$ROS_DISTRO-mola-state-estimation \
         ros-$ROS_DISTRO-mola-lidar-odometry

    .. code-block:: bash

        # (OPTIONAL) All public dataset readers (e.g. EuRoC) and the KITTI metrics tool:
        sudo apt install ros-$ROS_DISTRO-mola-academic-datasets

    .. code-block:: bash

        # (OPTIONAL) Install example small datasets to run demos/unit tests:
        sudo apt install ros-$ROS_DISTRO-mola-test-datasets

    Check if all new nodes and apps are visible:

    .. code-block:: bash

        # For example, let's launch the mm map viewer.
        # If a GUI app is opened, it means installation was successful.
        mm-viewer

    .. code-block:: bash

        # You can also test the mola LO cli interface:
        mola-lidar-odometry-cli --help


    These are the **versions available** from ROS build farms for each main MOLA component:

    +----------------------+--------------------+----------------+----------------+----------------+
    | Repository           | ROS 2 Humble       | ROS 2 Jazzy    | ROS 2 Lyrical  | ROS 2 Rolling  |
    |                      | (u22.04)           | (u24.04)       | (u26.04)       | (u26.04)       |
    +======================+====================+================+================+================+
    | MOLA                 | |badgeHrel|        | |badgeJrel|    | |badgeLrel|    | |badgeRrel|    |
    +----------------------+--------------------+----------------+----------------+----------------+
    | mola_lidar_odometry  | |badgeHrel_LO|     | |badgeJrel_LO| | |badgeLrel_LO| | |badgeRrel_LO| |
    +----------------------+--------------------+----------------+----------------+----------------+
    | mola_state_estimation| |badgeHrel_SE|     | |badgeJrel_SE| | |badgeLrel_SE| | |badgeRrel_SE| |
    +----------------------+--------------------+----------------+----------------+----------------+
    | mp2p_icp             | |badgeHrel_MP|     | |badgeJrel_MP| | |badgeLrel_MP| | |badgeRrel_MP| |
    +----------------------+--------------------+----------------+----------------+----------------+


.. dropdown:: Build from sources
    :icon: code-square

    **Building tools**
    MOLA uses ``colcon`` so you need to `install it first <https://colcon.readthedocs.io/en/released/user/installation.html>`_.

    Note that despite ROS 2 integration, a full ROS 2 installation is actually not **required** for MOLA, only ``colcon`` and ``ament``.


    **Get the sources**

    Clone the git repositories, including the submodules:

    .. code-block:: bash

        mkdir -p ~/ros2_mola_ws/src/
        cd ~/ros2_mola_ws/src/

        # Main MOLA modules:
        git clone https://github.com/MOLAorg/mola_common.git
        git clone https://github.com/MOLAorg/mp2p_icp.git --recursive
        git clone https://github.com/MOLAorg/mola.git --recursive
        git clone https://github.com/MOLAorg/mola_state_estimation.git
        git clone https://github.com/MOLAorg/mola_test_datasets.git
        git clone https://github.com/MOLAorg/mola_imu_preintegration.git
        git clone https://github.com/MOLAorg/mola_sm_loop_closure.git

        # MOLA lidar odometry package:
        git clone https://github.com/MOLAorg/mola_lidar_odometry.git --recursive

    **Dependencies**

    Make sure you have all dependencies installed (make sure of having `rosdep already installed <https://wiki.ros.org/rosdep>`_):

    .. code-block:: bash

        cd ~/ros2_mola_ws/
        rosdep install --from-paths src --ignore-src -r -y


    **Build and test**

    Now, compile as usual with colcon:

    .. code-block:: bash

        cd ~/ros2_mola_ws/
        colcon  build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo


    Next, activate the new environment and check if all new modules are visible:

    .. code-block:: bash

        cd ~/ros2_mola_ws/
        . install/setup.bash

        # For example, let's launch the mm map viewer:
        mm-viewer

.. humble badges ------

.. |badgeHrel| image:: https://img.shields.io/ros/v/humble/mola
   :align: middle
   :target: https://index.ros.org/?search_packages=true&pkgs=mola

.. |badgeHrel_LO| image:: https://img.shields.io/ros/v/humble/mola_lidar_odometry
   :align: middle
   :target: https://index.ros.org/?search_packages=true&pkgs=mola_lidar_odometry

.. |badgeHrel_MP| image:: https://img.shields.io/ros/v/humble/mp2p_icp
   :align: middle
   :target: https://index.ros.org/?search_packages=true&pkgs=mp2p_icp

.. |badgeHrel_SE| image:: https://img.shields.io/ros/v/humble/mola_state_estimation
   :align: middle
   :target: https://index.ros.org/?search_packages=true&pkgs=mola_state_estimation

.. jazzy badges ------

.. |badgeJrel| image:: https://img.shields.io/ros/v/jazzy/mola
   :align: middle
   :target: https://index.ros.org/?search_packages=true&pkgs=mola

.. |badgeJrel_LO| image:: https://img.shields.io/ros/v/jazzy/mola_lidar_odometry
   :align: middle
   :target: https://index.ros.org/?search_packages=true&pkgs=mola_lidar_odometry

.. |badgeJrel_MP| image:: https://img.shields.io/ros/v/jazzy/mp2p_icp
   :align: middle
   :target: https://index.ros.org/?search_packages=true&pkgs=mp2p_icp

.. |badgeJrel_SE| image:: https://img.shields.io/ros/v/jazzy/mola_state_estimation
   :align: middle
   :target: https://index.ros.org/?search_packages=true&pkgs=mola_state_estimation

.. lyrical badges ------

.. |badgeLrel| image:: https://img.shields.io/ros/v/lyrical/mola
   :align: middle
   :target: https://index.ros.org/?search_packages=true&pkgs=mola

.. |badgeLrel_LO| image:: https://img.shields.io/ros/v/lyrical/mola_lidar_odometry
   :align: middle
   :target: https://index.ros.org/?search_packages=true&pkgs=mola_lidar_odometry

.. |badgeLrel_MP| image:: https://img.shields.io/ros/v/lyrical/mp2p_icp
   :align: middle
   :target: https://index.ros.org/?search_packages=true&pkgs=mp2p_icp

.. |badgeLrel_SE| image:: https://img.shields.io/ros/v/lyrical/mola_state_estimation
   :align: middle
   :target: https://index.ros.org/?search_packages=true&pkgs=mola_state_estimation

.. rolling badges ------

.. |badgeRrel| image:: https://img.shields.io/ros/v/rolling/mola
   :align: middle
   :target: https://index.ros.org/?search_packages=true&pkgs=mola

.. |badgeRrel_LO| image:: https://img.shields.io/ros/v/rolling/mola_lidar_odometry
   :align: middle
   :target: https://index.ros.org/?search_packages=true&pkgs=mola_lidar_odometry

.. |badgeRrel_MP| image:: https://img.shields.io/ros/v/rolling/mp2p_icp
   :align: middle
   :target: https://index.ros.org/?search_packages=true&pkgs=mp2p_icp

.. |badgeRrel_SE| image:: https://img.shields.io/ros/v/rolling/mola_state_estimation
   :align: middle
   :target: https://index.ros.org/?search_packages=true&pkgs=mola_state_estimation
