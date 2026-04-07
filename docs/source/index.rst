.. MOLA documentation master file, created by
   sphinx-quickstart on Sat May  4 17:03:47 2019.

.. _index:

.. toctree::
  :maxdepth: 2
  :hidden:
  :caption: Get started

  Home <index.html#http://>
  why-mola
  building-maps
  localization
  geo-referencing
  ros2api
  map-tools

.. toctree::
  :maxdepth: 2
  :hidden:
  :caption: Solutions & pricing

  solutions
  pricing

.. toctree::
  :maxdepth: 2
  :hidden:
  :caption: 3D LiDAR

  mola_lidar_odometry
  mola_lo_apps
  mola_lo_pipelines
  wrappers_3rd_party

.. toctree::
  :maxdepth: 2
  :hidden:
  :caption: State estimation

  mola_state_estimators

.. toctree::
  :maxdepth: 2
  :hidden:
  :caption: mp2p_icp

  module-mp2p-icp
  mp2p_icp_basics
  mp2p_icp_filters
  mp2p_icp_optimal-transformations
  mp2p_icp_applications
  sm2mm_pipelines


.. toctree::
  :maxdepth: 2
  :hidden:
  :caption: Learn more

  tutorials
  mola_architecture
  dataset-conversions
  supported-sensors
  modules
  changelog
  doxygen-index
  bibliography


.. raw:: html

   <div style="text-align: center; margin: 1.5em 0 1em 0;">
     <p style="font-size: 1.3em; color: #333; margin-bottom: 0.5em;">
       <strong>Production-grade LiDAR SLAM for robotics and surveying</strong>
     </p>
     <p style="font-size: 1.05em; color: #555;">
       Open Core - free for research, commercially licensable for products
     </p>
   </div>

:octicon:`mark-github` `MOLA`_ is a Modular system for Localization and Mapping,
providing LiDAR Odometry (LO), LiDAR-inertial Odometry (LIO), SLAM, localization-only modes,
and geo-referencing.

.. raw:: html

   <div style="width: 100%; overflow: hidden;">
     <video controls autoplay loop muted style="width: 512px;">
       <source src="https://mrpt.github.io/videos/mola-lo-mulran-dcc02-demo-decaying-clouds.mp4" type="video/mp4">
     </video>
   </div>

.. image:: https://mrpt.github.io/imgs/mola-slam-kitti-demo.gif

|

.. grid:: 3

    .. grid-item-card:: Get started
        :link: building-maps
        :link-type: ref
        :class-card: sd-border-1

        Build your first map in minutes with the step-by-step tutorial.

    .. grid-item-card:: Why MOLA?
        :link: why-mola
        :link-type: ref
        :class-card: sd-border-1

        See what makes MOLA different from other SLAM frameworks.

    .. grid-item-card:: Plans & Pricing
        :link: pricing
        :link-type: ref
        :class-card: sd-border-1

        Compare Community (free) vs. Pro (commercial license).

|

Quick links
=============
- :ref:`Video tutorial: Ouster LIO mapping <tutorial-ouster-lio>`
- :ref:`Building your first map <building-maps>`
- :ref:`Mapping and localization <tutorial-mola-lo-map-and-localize>`
- :ref:`LiDAR odometry documentation <mola_lidar_odometry>` and :ref:`pipelines <mola_lo_pipelines>`
- :ref:`Solutions and use cases <solutions>`
- :ref:`How to cite MOLA <citing_mola>`


.. _MOLA: https://github.com/MOLAorg/mola
.. _videos: https://www.youtube.com/playlist?list=PLOJ3GF0x2_eVaujK78PoVOvxJGrl_Z7fV

|

.. humble badges ------

.. |badgeHrel| image:: https://img.shields.io/ros/v/humble/mola
   :scale: 100%
   :align: middle
   :target: https://index.ros.org/?search_packages=true&pkgs=mola

.. |badgeHrel_LO| image:: https://img.shields.io/ros/v/humble/mola_lidar_odometry
   :scale: 100%
   :align: middle
   :target: https://index.ros.org/?search_packages=true&pkgs=mola_lidar_odometry

.. |badgeHrel_MP| image:: https://img.shields.io/ros/v/humble/mp2p_icp
   :scale: 100%
   :align: middle
   :target: https://index.ros.org/?search_packages=true&pkgs=mp2p_icp

.. |badgeHrel_SE| image:: https://img.shields.io/ros/v/humble/mola_state_estimation
   :scale: 100%
   :align: middle
   :target: https://index.ros.org/?search_packages=true&pkgs=mola_state_estimation

.. iron badges ------

.. |badgeIrel| image:: https://img.shields.io/ros/v/iron/mola
   :scale: 100%
   :align: middle
   :target: https://index.ros.org/?search_packages=true&pkgs=mola

.. |badgeIrel_LO| image:: https://img.shields.io/ros/v/iron/mola_lidar_odometry
   :scale: 100%
   :align: middle
   :target: https://index.ros.org/?search_packages=true&pkgs=mola_lidar_odometry

.. |badgeIrel_MP| image:: https://img.shields.io/ros/v/iron/mp2p_icp
   :scale: 100%
   :align: middle
   :target: https://index.ros.org/?search_packages=true&pkgs=mp2p_icp

.. jazzy badges ------

.. |badgeJrel| image:: https://img.shields.io/ros/v/jazzy/mola
   :scale: 100%
   :align: middle
   :target: https://index.ros.org/?search_packages=true&pkgs=mola

.. |badgeJrel_LO| image:: https://img.shields.io/ros/v/jazzy/mola_lidar_odometry
   :scale: 100%
   :align: middle
   :target: https://index.ros.org/?search_packages=true&pkgs=mola_lidar_odometry

.. |badgeJrel_MP| image:: https://img.shields.io/ros/v/jazzy/mp2p_icp
   :scale: 100%
   :align: middle
   :target: https://index.ros.org/?search_packages=true&pkgs=mp2p_icp

.. |badgeJrel_SE| image:: https://img.shields.io/ros/v/jazzy/mola_state_estimation
   :scale: 100%
   :align: middle
   :target: https://index.ros.org/?search_packages=true&pkgs=mola_state_estimation

.. rolling badges ------

.. |badgeRrel| image:: https://img.shields.io/ros/v/rolling/mola
   :scale: 100%
   :align: middle
   :target: https://index.ros.org/?search_packages=true&pkgs=mola

.. |badgeRrel_LO| image:: https://img.shields.io/ros/v/rolling/mola_lidar_odometry
   :scale: 100%
   :align: middle
   :target: https://index.ros.org/?search_packages=true&pkgs=mola_lidar_odometry

.. |badgeRrel_MP| image:: https://img.shields.io/ros/v/rolling/mp2p_icp
   :scale: 100%
   :align: middle
   :target: https://index.ros.org/?search_packages=true&pkgs=mp2p_icp

.. |badgeRrel_SE| image:: https://img.shields.io/ros/v/rolling/mola_state_estimation
   :scale: 100%
   :align: middle
   :target: https://index.ros.org/?search_packages=true&pkgs=mola_state_estimation


.. _installing:

Installing
======================

How to install all MOLA modules:

.. dropdown:: From ROS 2 repositories
    :open:
    :icon: download

    **Recommended**: This is the easiest way to install MOLA.

    In Debian/Ubuntu systems, activate your ROS environment (``setup.bash``) if not done automatically
    in your ``~./bashrc`` file, then just run:

    .. code-block:: bash

        # Install core MOLA modules and 3D LiDAR odometry:
        sudo apt install \
         ros-$ROS_DISTRO-mola \
         ros-$ROS_DISTRO-mola-state-estimation \
         ros-$ROS_DISTRO-mola-lidar-odometry

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

    +----------------------+--------------------+----------------+----------------+
    | Repository           | ROS 2 Humble       | ROS 2 Jazzy    | ROS 2 Rolling  |
    |                      |  (u22.04)          |   (u24.04)     |    (u24.04)    |
    +======================+====================+================+================+
    | MOLA                 | |badgeHrel|        | |badgeJrel|    | |badgeRrel|    |
    +----------------------+--------------------+----------------+----------------+
    | mola_lidar_odometry  | |badgeHrel_LO|     | |badgeJrel_LO| | |badgeRrel_LO| |
    +----------------------+--------------------+----------------+----------------+
    | mola_state_estimation| |badgeHrel_SE|     | |badgeJrel_SE| | |badgeRrel_SE| |
    +----------------------+--------------------+----------------+----------------+
    | mp2p_icp             | |badgeHrel_MP|     | |badgeJrel_MP| | |badgeRrel_MP| |
    +----------------------+--------------------+----------------+----------------+

    EOL ROS distribution:

     - ROS 2 Iron (u22.04): MOLA |badgeIrel|, mola_lidar_odometry |badgeIrel_LO|, mp2p_icp |badgeIrel_MP|


.. dropdown:: Build from sources
    :icon: code-square

    Building tools
    ~~~~~~~~~~~~~~~~~
    MOLA uses ``colcon`` so you need to `install it first <https://colcon.readthedocs.io/en/released/user/installation.html>`_.

    Note that despite ROS 2 integration, a full ROS 2 installation is actually not **required** for MOLA, only ``colcon`` and ``ament``.


    Get the sources
    ~~~~~~~~~~~~~~~~~

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

    Dependencies
    ~~~~~~~~~~~~~~~~~

    Make sure you have all dependencies installed (make sure of having `rosdep already installed <https://wiki.ros.org/rosdep>`_):

    .. code-block:: bash

        cd ~/ros2_mola_ws/
        rosdep install --from-paths src --ignore-src -r -y


    Build and test
    ~~~~~~~~~~~~~~~~~

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

|

.. _citing_mola:

How to cite MOLA
==================

The ``mola_lidar_odometry`` system was presented in :cite:`blanco2025mola_lo` ([ArXiV](https://arxiv.org/abs/2407.20465)):

  Blanco-Claraco JL. A flexible framework for accurate LiDAR odometry, map manipulation, and localization.
  The International Journal of Robotics Research. 2025;0(0).
  doi:10.1177/02783649251316881

.. _A flexible framework for accurate LiDAR odometry, map manipulation, and localization: https://arxiv.org/abs/2407.20465

The basics of the MOLA framework were introduced in :cite:`blanco2019modular`.

  J.L. Blanco,
  `A Modular Optimization Framework for Localization and Mapping`_, in
  *Robotics: Science and Systems (RSS)*, 2019.

.. _A Modular Optimization Framework for Localization and Mapping: https://ingmec.ual.es/~jlblanco/papers/blanco2019mola_rss2019.pdf
