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

.. raw:: html

   <style>
   .mola-ind-grid {
     display: grid;
     grid-template-columns: repeat(auto-fit, minmax(240px, 1fr));
     gap: 14px;
     margin: 1.5rem 0 2rem;
   }
   .mola-ind-card {
     border: 1px solid #d0d7de;
     border-radius: 8px;
     padding: 18px 20px 16px;
     background: #fff;
   }
   .mola-ind-icon {
     width: 32px; height: 32px;
     border-radius: 6px;
     background: #eef5fc;
     display: flex; align-items: center; justify-content: center;
     margin-bottom: 12px;
   }
   .mola-ind-icon svg { width: 18px; height: 18px; stroke: #185FA5; fill: none; stroke-width: 1.8; stroke-linecap: round; stroke-linejoin: round; }
   .mola-ind-title {
     font-size: 14px; font-weight: 600;
     color: #1f2328; margin: 0 0 6px;
   }
   .mola-ind-body {
     font-size: 13px; color: #57606a;
     line-height: 1.55; margin: 0 0 12px;
   }
   .mola-ind-links {
     font-size: 12px; color: #0550ae;
     border-top: 1px solid #eaeef2;
     padding-top: 10px; margin-top: auto;
   }
   .mola-ind-links span { color: #57606a; font-weight: 500; }
   </style>

   <div class="mola-ind-grid">

     <div class="mola-ind-card">
       <div class="mola-ind-icon">
         <svg viewBox="0 0 24 24"><rect x="2" y="7" width="20" height="14" rx="2"/><path d="M16 7V5a2 2 0 0 0-2-2h-4a2 2 0 0 0-2 2v2"/><line x1="12" y1="12" x2="12" y2="16"/><line x1="10" y1="14" x2="14" y2="14"/></svg>
       </div>
       <p class="mola-ind-title">Autonomous Mobile Robots (AMR)</p>
       <p class="mola-ind-body">Warehouse logistics, cleaning robots, inspection platforms. Real-time LO/LIO for navigation and prebuilt-map localization for autonomous operation.</p>
       <div class="mola-ind-links"><span>Solutions:</span> LO/LIO &middot; Fused localization</div>
     </div>

     <div class="mola-ind-card">
       <div class="mola-ind-icon">
         <svg viewBox="0 0 24 24"><path d="M12 2a9 9 0 0 1 9 9c0 4.17-5.33 9.5-8.08 12.07a1.3 1.3 0 0 1-1.84 0C8.33 20.5 3 15.17 3 11a9 9 0 0 1 9-9z"/><circle cx="12" cy="11" r="3"/></svg>
       </div>
       <p class="mola-ind-title">Agriculture &amp; Greenhouses</p>
       <p class="mola-ind-body">Autonomous tractors, greenhouse navigation, crop monitoring, and precision agriculture. Validated on the GreenBot dataset in Mediterranean greenhouse environments.</p>
       <div class="mola-ind-links"><span>Solutions:</span> Map-less georef &middot; LO/LIO</div>
     </div>

     <div class="mola-ind-card">
       <div class="mola-ind-icon">
         <svg viewBox="0 0 24 24"><path d="M5 17H3a2 2 0 0 1-2-2V5a2 2 0 0 1 2-2h11a2 2 0 0 1 2 2v3"/><rect x="9" y="11" width="14" height="10" rx="2"/><line x1="12" y1="11" x2="12" y2="21"/><line x1="9" y1="16" x2="23" y2="16"/></svg>
       </div>
       <p class="mola-ind-title">Automotive &amp; ADAS</p>
       <p class="mola-ind-body">Urban autonomous driving and HD map generation. Benchmarked on KITTI with 0.6% translation error. Compatible with standard automotive sensor suites (LiDAR + IMU + GNSS).</p>
       <div class="mola-ind-links"><span>Solutions:</span> Full SLAM &middot; Fused localization</div>
     </div>

     <div class="mola-ind-card">
       <div class="mola-ind-icon">
         <svg viewBox="0 0 24 24"><polygon points="3 11 22 2 13 21 11 13 3 11"/></svg>
       </div>
       <p class="mola-ind-title">Surveying &amp; 3D Scanning</p>
       <p class="mola-ind-body">Backpack mapping, drone-based surveying, forest inventory. Export to LAS/PLY for GIS workflows. Sub-centimeter accuracy with survey-grade pipeline tuning.</p>
       <div class="mola-ind-links"><span>Solutions:</span> Full SLAM &middot; LO/LIO</div>
     </div>

     <div class="mola-ind-card">
       <div class="mola-ind-icon">
         <svg viewBox="0 0 24 24"><circle cx="11" cy="11" r="8"/><path d="M21 21l-4.35-4.35"/><line x1="11" y1="8" x2="11" y2="14"/><line x1="8" y1="11" x2="14" y2="11"/></svg>
       </div>
       <p class="mola-ind-title">Inspection</p>
       <p class="mola-ind-body">Industrial facility inspection, infrastructure monitoring, underground and mining environments. Works with handheld, backpack, and drone-mounted LiDARs.</p>
       <div class="mola-ind-links"><span>Solutions:</span> LO/LIO &middot; Full SLAM</div>
     </div>

     <div class="mola-ind-card">
       <div class="mola-ind-icon">
         <svg viewBox="0 0 24 24"><path d="M3 9l9-7 9 7v11a2 2 0 0 1-2 2H5a2 2 0 0 1-2-2z"/><polyline points="9 22 9 12 15 12 15 22"/></svg>
       </div>
       <p class="mola-ind-title">Indoor Mapping</p>
       <p class="mola-ind-body">Building interiors, warehouses, offices. Create detailed 3D maps for facility management, renovation planning, or robot navigation.</p>
       <div class="mola-ind-links"><span>Solutions:</span> LO/LIO &middot; Full SLAM</div>
     </div>

   </div>

|

Demos and videos
===================

.. raw:: html

   <style>
   .mola-videos-grid {
     display: grid;
     grid-template-columns: repeat(auto-fit, minmax(280px, 1fr));
     gap: 16px;
     margin: 1.5rem 0 2rem;
   }
   .mola-video-card {
     border: 1px solid #d0d7de;
     border-radius: 8px;
     overflow: hidden;
     background: #fff;
   }
   .mola-video-card iframe {
     display: block;
     width: 100%;
     aspect-ratio: 16 / 9;
     border: none;
   }
   .mola-video-label {
     padding: 10px 14px;
     font-size: 13px;
     font-weight: 500;
     color: #1f2328;
     border-top: 1px solid #eaeef2;
   }
   .mola-videos-more {
     font-size: 13px;
     color: #57606a;
     margin-top: 0.5rem;
   }
   .mola-videos-more a { color: #0550ae; }
   </style>

   <div class="mola-videos-grid">

     <div class="mola-video-card">
       <iframe src="https://www.youtube.com/embed/sbakEOnsL6Y?si=xV8-RGNiEFKR-dAI"
         title="Forest inventory 3D mapping"
         allow="accelerometer; autoplay; clipboard-write; encrypted-media; gyroscope; picture-in-picture; web-share"
         referrerpolicy="strict-origin-when-cross-origin" allowfullscreen></iframe>
       <div class="mola-video-label">Forest inventory 3D mapping</div>
     </div>

     <div class="mola-video-card">
       <iframe src="https://www.youtube.com/embed/XNvf8OMXZoY?si=QqiMlni2lmcojph_"
         title="Backpack 3D mapping indoors"
         allow="accelerometer; autoplay; clipboard-write; encrypted-media; gyroscope; picture-in-picture; web-share"
         referrerpolicy="strict-origin-when-cross-origin" allowfullscreen></iframe>
       <div class="mola-video-label">Backpack 3D mapping indoors</div>
     </div>

     <div class="mola-video-card">
       <iframe src="https://www.youtube.com/embed/1h2aayHvhVU?si=xWMJZ7bDfaWKlOfY"
         title="Drone mapping (HILTI 2021)"
         allow="accelerometer; autoplay; clipboard-write; encrypted-media; gyroscope; picture-in-picture; web-share"
         referrerpolicy="strict-origin-when-cross-origin" allowfullscreen></iframe>
       <div class="mola-video-label">Drone mapping - HILTI 2021</div>
     </div>

     <div class="mola-video-card">
       <iframe src="https://www.youtube.com/embed/tdXzYeG51Bc?si=IgjYINt1t7qoLb7R"
         title="Greenhouse mapping"
         allow="accelerometer; autoplay; clipboard-write; encrypted-media; gyroscope; picture-in-picture; web-share"
         referrerpolicy="strict-origin-when-cross-origin" allowfullscreen></iframe>
       <div class="mola-video-label">Greenhouse mapping</div>
     </div>

   </div>

   <p class="mola-videos-more">More demos on the <a href="https://www.youtube.com/playlist?list=PLOJ3GF0x2_eVaujK78PoVOvxJGrl_Z7fV" target="_blank">MOLA YouTube playlist &rarr;</a></p>