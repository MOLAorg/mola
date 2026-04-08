.. _why-mola:

===========
Why MOLA
===========

MOLA is a **production-grade LiDAR SLAM framework** for robotics, autonomous vehicles,
and 3D surveying. It follows an :ref:`Open Core <pricing>` model: the full framework is
open-source and free for research, while commercial users can upgrade to **MOLA Pro**
for closed-source deployment, pre-built binaries, and priority support.

.. contents:: On this page
   :depth: 1
   :local:
   :backlinks: none

|

What makes MOLA different
===========================

Fully configurable - no recompilation
---------------------------------------
MOLA pipelines are defined entirely in **YAML**. Swap ICP matchers, change map layers,
adjust filter chains, or switch between LO and LIO - all without rebuilding a single
line of C++. No other SLAM framework offers this level of runtime configurability.
This means faster iteration during development and safer updates in production.

Flexible accuracy: real-time navigation to survey-grade
--------------------------------------------------------
The same framework serves both **fast, real-time AMR navigation** and
**survey-grade mapping with sub-centimeter accuracy**. Tune the pipeline for your
needs - there is no need to switch between different SLAM stacks for different
accuracy requirements.

Map-less RTK-quality outdoor georeferencing
---------------------------------------------
Using the :ref:`smoother state estimator <mola_sta_est_index>` with a
**low-cost GNSS receiver + LiDAR + IMU**, MOLA delivers geodetic-quality
pose estimation in UTM/ENU coordinates - **without a prebuilt map and without
an RTK base station**. This is ideal for outdoor robots, agricultural vehicles,
and autonomous driving applications where absolute positioning matters.

Rich metric map ecosystem
-----------------------------
MOLA's ``.mm`` metric map format comes with a full set of CLI tools:

- :ref:`mm-viewer <app_mm-viewer>`: Interactive 3D map viewer
- :ref:`mm2las <app_mm2las>`, :ref:`mm2ply <app_mm2ply>`, :ref:`mm2txt <app_mm2txt>`: Export to LAS, PLY, TXT for GIS/surveying workflows
- :ref:`mm-filter <app_mm-filter>`: Filter and transform maps
- :ref:`mm-georef <app_mm-georef>`: Georeference maps
- :ref:`mm-info <app_mm-info>`: Inspect map metadata

This makes MOLA maps **interoperable with industry-standard tools** like CloudCompare,
QGIS, and surveying software.

ROS 2 native AND standalone C++
---------------------------------
Deploy with full **ROS 2 integration** (Humble, Jazzy, Kilted, Rolling) or as a
**standalone C++ application** without any ROS dependency. No other SLAM SDK
offers both cleanly. This gives you maximum flexibility for edge deployment,
Docker containers, or integration into proprietary frameworks.

Multi-sensor, multi-environment
---------------------------------
- **LiDAR-only odometry (LO)** - no IMU required
- **LiDAR-inertial odometry (LIO)** - with IMU fusion
- **GNSS fusion** - consumer-grade GPS for georeferencing
- **Kinematics fusion** - wheel encoders, vehicle odometry

Validated across **urban driving** (KITTI), **agricultural environments** (GreenBot),
**indoor** (warehouses, buildings), **outdoor** (forests, campuses), and **aerial** (drones).

Academic rigor, production ready
----------------------------------
MOLA is backed by peer-reviewed publications in top venues:

- **IJRR 2025**: `A flexible framework for accurate LiDAR odometry, map manipulation, and localization <https://doi.org/10.1177/02783649251316881>`_
- **RSS 2019**: `A Modular Optimization Framework for Localization and Mapping <https://ingmec.ual.es/~jlblanco/papers/blanco2019mola_rss2019.pdf>`_

Benchmarked on KITTI (0.6% translation error), MulRan, HILTI, Kaist, and custom
datasets. This is not a paper prototype - it is actively deployed in real-world robots.

Pre-built for amd64 and arm64
--------------------------------
Binary packages are available from the **ROS build farms** for both amd64 and arm64
(Jetson, Raspberry Pi). MOLA Pro subscribers additionally get access to a private
apt repository and Docker images for streamlined deployment.


|

.. _use-cases:

Use cases
=============
MOLA is used across a wide range of industries and environments:

.. grid:: 2

    .. grid-item-card:: Autonomous Mobile Robots
        :link: solutions
        :link-type: ref

        Warehouse logistics, cleaning robots, inspection platforms.

    .. grid-item-card:: Agriculture & Greenhouses
        :link: solutions
        :link-type: ref

        Autonomous tractors, greenhouse navigation, crop monitoring.

    .. grid-item-card:: Automotive & ADAS
        :link: solutions
        :link-type: ref

        Urban autonomous driving, mapping for HD maps.

    .. grid-item-card:: Surveying & 3D Scanning
        :link: solutions
        :link-type: ref

        Backpack mapping, drone surveying, forest inventory.

|

Ready to get started?
======================

.. grid:: 3

    .. grid-item-card:: Get started
        :link: building-maps
        :link-type: ref

        Build your first map in minutes.

    .. grid-item-card:: See plans & pricing
        :link: pricing
        :link-type: ref

        Compare Community vs. Pro.

    .. grid-item-card:: Solutions
        :link: solutions
        :link-type: ref

        Explore use cases and capabilities.
