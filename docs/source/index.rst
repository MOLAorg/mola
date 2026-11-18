.. MOLA documentation master file, created by
   sphinx-quickstart on Sat May  4 17:03:47 2019.

.. _index:

.. toctree::
  :maxdepth: 2
  :hidden:
  :caption: Get started

  Home <index.html#http://>
  how-to-install
  quickstart
  your-own-rosbag
  gui-vs-cli
  troubleshooting

.. toctree::
  :maxdepth: 2
  :hidden:
  :caption: Guides

  building-maps
  localization
  geo-referencing
  mola_ros2_configurations
  datasets/index

.. toctree::
  :maxdepth: 2
  :hidden:
  :caption: Tutorials

  tutorials

.. toctree::
  :maxdepth: 2
  :hidden:
  :caption: LiDAR odometry

  mola_lidar_odometry
  mola_lo_apps
  mola_lo_pipelines
  mola_lo_diagnostics
  wrappers_3rd_party

.. toctree::
  :maxdepth: 2
  :hidden:
  :caption: SLAM and state estimation

  mola_mapper
  mola_state_estimators

.. toctree::
  :maxdepth: 2
  :hidden:
  :caption: Maps and mp2p_icp

  map-tools
  module-mp2p-icp
  mp2p_icp_basics
  mp2p_icp_filters
  mp2p_icp_optimal-transformations
  sm2mm_pipelines

.. toctree::
  :maxdepth: 2
  :hidden:
  :caption: Reference

  ros2api
  supported-sensors
  dataset-conversions
  concept-slam-configuration-file
  module-mola-launcher
  mola_architecture
  tutorial-mola-cmake-commands
  modules
  changelog
  doxygen-index
  bibliography

.. toctree::
  :maxdepth: 2
  :hidden:
  :caption: Product

  why-mola
  solutions
  pricing


=====================================================
MOLA: LiDAR Odometry, SLAM and Localization
=====================================================

.. raw:: html

   <section class="mola-hero" aria-label="MOLA">
     <video class="mola-hero-video" autoplay muted loop playsinline preload="auto"
            poster="https://mrpt.github.io/videos/mola_hero_v1_poster.jpg">
       <source src="https://mrpt.github.io/videos/mola_hero_v1_720p.mp4" type="video/mp4" media="(max-width: 900px)">
       <source src="https://mrpt.github.io/videos/mola_hero_v1.mp4" type="video/mp4">
     </video>
     <div class="mola-hero-shade"></div>
     <nav class="mola-hero-nav">
       <a class="mola-hero-brand" href="index.html">MOLA</a>
       <span class="mola-hero-links">
         <a href="quickstart.html">Quickstart</a>
         <a href="tutorials.html">Tutorials</a>
         <a href="modules.html">Modules</a>
         <a href="solutions.html">Solutions</a>
         <a href="pricing.html">Pricing</a>
         <a href="https://github.com/MOLAorg/mola">GitHub</a>
       </span>
     </nav>
     <div class="mola-hero-content">
       <p class="mola-hero-eyebrow">Open-source LiDAR SLAM &middot; C++ &middot; ROS 2</p>
       <p class="mola-hero-title">Production-grade LiDAR SLAM<br/>for robots and surveying</p>
       <p class="mola-hero-sub">LiDAR and LiDAR-inertial odometry, live SLAM with loop closure,
         localization and geo-referencing, from handheld scanners to legged robots and cars.
         Free for research, commercially licensable for products.</p>
       <div class="mola-hero-cta">
         <a class="mola-btn mola-btn-primary" href="quickstart.html">Get started</a>
         <a class="mola-btn mola-btn-secondary" href="why-mola.html">Why MOLA</a>
         <a class="mola-btn mola-btn-ghost" href="pricing.html">Plans &amp; pricing</a>
       </div>
     </div>
     <p class="mola-hero-caption">Real data: GrandTour (ANYmal), Husky robot at the University of Almer&iacute;a,
       TorWIC warehouse, Oxford Spires, VBR Rome.</p>
   </section>

:octicon:`mark-github` `MOLA`_ is a Modular system for Localization and Mapping,
providing LiDAR Odometry (LO), LiDAR-inertial Odometry (LIO), SLAM, localization-only modes,
and geo-referencing.

.. raw:: html

   <div class="mola-cards">

     <a class="mola-card" href="tutorial-ouster-lio.html">
       <div class="mola-card-icon">
         <svg viewBox="0 0 24 24"><path d="M12 20h9"/><path d="M16.5 3.5a2.121 2.121 0 0 1 3 3L7 19l-4 1 1-4L16.5 3.5z"/></svg>
       </div>
       <p class="mola-card-title">Get started</p>
       <p class="mola-card-body">Build your first map in minutes with the step-by-step tutorial.</p>
       <span class="mola-card-cta">
         Start tutorial
         <svg viewBox="0 0 24 24"><line x1="5" y1="12" x2="19" y2="12"/><polyline points="12 5 19 12 12 19"/></svg>
       </span>
     </a>

     <a class="mola-card" href="why-mola.html">
       <div class="mola-card-icon">
         <svg viewBox="0 0 24 24"><circle cx="12" cy="12" r="10"/><line x1="12" y1="8" x2="12" y2="12"/><line x1="12" y1="16" x2="12.01" y2="16"/></svg>
       </div>
       <p class="mola-card-title">Why MOLA?</p>
       <p class="mola-card-body">See what makes MOLA different from other SLAM frameworks.</p>
       <span class="mola-card-cta">
         Learn more
         <svg viewBox="0 0 24 24"><line x1="5" y1="12" x2="19" y2="12"/><polyline points="12 5 19 12 12 19"/></svg>
       </span>
     </a>

     <a class="mola-card" href="pricing.html">
       <div class="mola-card-icon">
         <svg viewBox="0 0 24 24"><rect x="2" y="7" width="20" height="14" rx="2"/><path d="M16 7V5a2 2 0 0 0-2-2h-4a2 2 0 0 0-2 2v2"/><line x1="12" y1="12" x2="12" y2="16"/><line x1="10" y1="14" x2="14" y2="14"/></svg>
       </div>
       <p class="mola-card-title">Plans &amp; pricing</p>
       <p class="mola-card-body">Compare Community (free) vs. Pro (commercial license).</p>
       <span class="mola-card-cta">
         See plans
         <svg viewBox="0 0 24 24"><line x1="5" y1="12" x2="19" y2="12"/><polyline points="12 5 19 12 12 19"/></svg>
       </span>
     </a>

   </div>

|

Quick links
=============
- :ref:`Install MOLA <installing>`
- :ref:`Video tutorial: Ouster LIO mapping <tutorial-ouster-lio>`
- :ref:`Building your first map <building-maps>`
- :ref:`Mapping and localization <tutorial-mola-lo-map-and-localize>`
- :ref:`LiDAR odometry documentation <mola_lidar_odometry>` and :ref:`pipelines <mola_lo_pipelines>`
- :ref:`Solutions and use cases <solutions>`
- :ref:`How to cite MOLA <citing_mola>`


.. _MOLA: https://github.com/MOLAorg/mola
.. _videos: https://www.youtube.com/playlist?list=PLOJ3GF0x2_eVaujK78PoVOvxJGrl_Z7fV

|

.. _citing_mola:

How to cite MOLA
==================

The ``mola_lidar_odometry`` system was presented in :cite:`blanco2025mola_lo` (`ArXiV <https://arxiv.org/abs/2407.20465>`_):

  Blanco-Claraco JL. A flexible framework for accurate LiDAR odometry, map manipulation, and localization.
  The International Journal of Robotics Research. 2025;0(0).
  doi:10.1177/02783649251316881

.. _A flexible framework for accurate LiDAR odometry, map manipulation, and localization: https://arxiv.org/abs/2407.20465

The basics of the MOLA framework were introduced in :cite:`blanco2019modular`.

  J.L. Blanco,
  `A Modular Optimization Framework for Localization and Mapping`_, in
  *Robotics: Science and Systems (RSS)*, 2019.

.. _A Modular Optimization Framework for Localization and Mapping: https://ingmec.ual.es/~jlblanco/papers/blanco2019mola_rss2019.pdf
