.. _solutions:

=========================
Solutions
=========================

MOLA provides a modular set of solutions for LiDAR-based localization, mapping,
and SLAM. All solutions are fully configurable via YAML - no recompilation needed.

See :ref:`Plans and pricing <pricing>` for the comparison between Community (open-source)
and Pro (commercial) editions.

.. contents:: On this page
   :depth: 1
   :local:
   :backlinks: none

|

.. _sol-lo-lio:

1. LiDAR odometry and localization (LO / LIO)
================================================

:ref:`LiDAR odometry <mola_lidar_odometry>` is one of the most advanced and flexible
LiDAR odometry modules available. It supports:

- **LiDAR-only odometry (LO)** - no IMU required
- **LiDAR-inertial odometry (LIO)** - fusing LiDAR with IMU for improved robustness

Both modes work in **mapping** (build a map from scratch) and **localization**
(localize within a prebuilt map) configurations.

**Key strengths:** Sub-centimeter accuracy when tuned for survey-grade applications,
or lightweight real-time performance for navigation. Configurable pipeline via YAML.

**Get started:** :ref:`Build your first map <building-maps>` |
:ref:`Mapping and localization tutorial <tutorial-mola-lo-map-and-localize>` |
:ref:`Ouster LIO tutorial <tutorial-ouster-lio>`

.. image:: https://mrpt.github.io/imgs/mola-slam-kitti-demo.gif

|

.. _sol-georef-localization:

2. Fused localization: LO/LIO + GNSS + kinematics
====================================================

Based on the :ref:`smoother state estimator <mola_sta_est_index>`, this solution
improves localization in a prebuilt map by fusing:

- LiDAR odometry output (LO or LIO)
- **GNSS** (consumer-grade GPS receivers)
- **Kinematics** (wheel encoders, vehicle odometry)
- **IMU** data

This multi-sensor fusion provides robust, drift-corrected localization suitable for
autonomous navigation in both indoor and outdoor environments. The smoother handles
sensor outages gracefully - if GNSS signal is lost indoors, LiDAR odometry continues
seamlessly.

**Best for:** Autonomous mobile robots (AMR), outdoor vehicles transitioning between
GPS-available and GPS-denied areas, mixed indoor-outdoor scenarios.

|

.. _sol-mapless-georef:

3. Map-less georeferenced localization
========================================

Based on the :ref:`smoother state estimator <mola_sta_est_index>`, this solution
provides **RTK-quality georeferenced pose estimation without a prebuilt map**.

By fusing **low-cost GNSS + LiDAR + IMU + kinematics**, the smoother estimates
the vehicle pose in geodetic (latitude/longitude) or UTM coordinates in real time.
No RTK base station is required - a standard consumer-grade GNSS receiver is sufficient.

This enables:

- Outdoor robot navigation with absolute positioning from the first second
- Georeferenced trajectory logging for fleet management
- Autonomous driving in open environments without prior mapping

**Best for:** Agricultural robots, autonomous tractors, outdoor inspection platforms,
delivery robots, and any application requiring absolute outdoor positioning without
the cost and complexity of RTK infrastructure.

|

.. _sol-full-slam:

4. Full 3D SLAM (georeferencing + loop closure)
=================================================

Build **globally consistent, georeferenced 3D maps**, even in large-scale environments
mixing indoor and outdoor areas. This is the most complete SLAM solution in MOLA,
combining:

- **Georeferencing** metric maps with consumer-grade GNSS sensors
  (see :ref:`geo-referencing`)
- **Offline loop closure** for globally consistent maps - corrects accumulated
  drift over long trajectories
- **Simple-map → metric-map pipelines** for flexible post-processing
  (see :ref:`sm2mm_pipelines`)

.. image:: https://mrpt.github.io/imgs/kaist01_georef_sample.png

**Best for:** Large-scale surveying, building 3D maps of campuses/cities,
creating reference maps for localization.

**Coming soon:**

- ``mola_3d_mapper``: Full live/offline 3D SLAM with online loop closure *(in development)*
- ``mola_2d_mapper``: 2D SLAM via pose graph optimization for 2D LiDARs *(in development)*

|

.. _sol-industry:

Industry applications
========================

.. grid:: 2

    .. grid-item-card:: Autonomous Mobile Robots (AMR)
        :class-card: sd-border-1

        Warehouse logistics, cleaning robots, inspection platforms.
        MOLA provides real-time LO/LIO for navigation and pre-built map
        localization for autonomous operation.

        **Relevant solutions:** :ref:`LO/LIO <sol-lo-lio>`,
        :ref:`Fused localization <sol-georef-localization>`

    .. grid-item-card:: Agriculture & Greenhouses
        :class-card: sd-border-1

        Autonomous tractors, greenhouse navigation, crop monitoring, and
        precision agriculture. Validated on the GreenBot dataset in
        Mediterranean greenhouse environments.

        **Relevant solutions:** :ref:`Map-less georef <sol-mapless-georef>`,
        :ref:`LO/LIO <sol-lo-lio>`

    .. grid-item-card:: Automotive & ADAS
        :class-card: sd-border-1

        Urban autonomous driving and HD map generation. Benchmarked on
        KITTI with 0.4-2.0% translation error. Compatible with standard
        automotive sensor suites (LiDAR + IMU + GNSS).

        **Relevant solutions:** :ref:`Full SLAM <sol-full-slam>`,
        :ref:`Fused localization <sol-georef-localization>`

    .. grid-item-card:: Surveying & 3D Scanning
        :class-card: sd-border-1

        Backpack mapping, drone-based surveying, forest inventory.
        Export to LAS/PLY for GIS workflows. Sub-centimeter accuracy
        with survey-grade pipeline tuning.

        **Relevant solutions:** :ref:`Full SLAM <sol-full-slam>`,
        :ref:`LO/LIO <sol-lo-lio>`

    .. grid-item-card:: Inspection
        :class-card: sd-border-1

        Industrial facility inspection, infrastructure monitoring,
        underground/mining environments. Works with handheld, backpack,
        and drone-mounted LiDARs.

        **Relevant solutions:** :ref:`LO/LIO <sol-lo-lio>`,
        :ref:`Full SLAM <sol-full-slam>`

    .. grid-item-card:: Indoor Mapping
        :class-card: sd-border-1

        Building interiors, warehouses, offices. Create detailed 3D
        maps for facility management, renovation planning, or robot
        navigation.

        **Relevant solutions:** :ref:`LO/LIO <sol-lo-lio>`,
        :ref:`Full SLAM <sol-full-slam>`

|

Demos and videos
===================

.. raw:: html

    <div style="margin-top:10px;">
      <iframe width="560" height="315" src="https://www.youtube.com/embed/sbakEOnsL6Y?si=xV8-RGNiEFKR-dAI" title="Forest inventory 3D mapping" frameborder="0" allow="accelerometer; autoplay; clipboard-write; encrypted-media; gyroscope; picture-in-picture; web-share" referrerpolicy="strict-origin-when-cross-origin" allowfullscreen></iframe>
    </div>

|

.. raw:: html

    <div style="margin-top:10px;">
      <iframe width="560" height="315" src="https://www.youtube.com/embed/XNvf8OMXZoY?si=QqiMlni2lmcojph_" title="Backpack 3D mapping indoors" frameborder="0" allow="accelerometer; autoplay; clipboard-write; encrypted-media; gyroscope; picture-in-picture; web-share" referrerpolicy="strict-origin-when-cross-origin" allowfullscreen></iframe>
    </div>

|

.. raw:: html

    <div style="margin-top:10px;">
      <iframe width="560" height="315" src="https://www.youtube.com/embed/1h2aayHvhVU?si=xWMJZ7bDfaWKlOfY" title="Drone mapping (HILTI 2021)" frameborder="0" allow="accelerometer; autoplay; clipboard-write; encrypted-media; gyroscope; picture-in-picture; web-share" referrerpolicy="strict-origin-when-cross-origin" allowfullscreen></iframe>
    </div>

|

.. raw:: html

    <div style="margin-top:10px;">
      <iframe width="560" height="315" src="https://www.youtube.com/embed/tdXzYeG51Bc?si=IgjYINt1t7qoLb7R" title="Greenhouse mapping" frameborder="0" allow="accelerometer; autoplay; clipboard-write; encrypted-media; gyroscope; picture-in-picture; web-share" referrerpolicy="strict-origin-when-cross-origin" allowfullscreen></iframe>
    </div>

|

See also the `MOLA YouTube playlist <https://www.youtube.com/playlist?list=PLOJ3GF0x2_eVaujK78PoVOvxJGrl_Z7fV>`_
for more demos.
