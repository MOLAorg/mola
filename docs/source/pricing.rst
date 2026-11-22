.. _pricing:

=======================
Plans and pricing
=======================

MOLA follows an **Open Core** model: the complete framework is open-source
(BSD-3 and GPL-3 licensed), free for research, education, and personal use.
Companies needing to deploy MOLA in commercial, closed-source products can
upgrade to **MOLA Pro**.

|

.. _plan-comparison:

Community vs. Pro
===================


.. raw:: html


   <div class="mola-pricing-wrap">
   <table class="mola-pricing-table">
     <colgroup>
       <col class="col-feat">
       <col class="col-comm">
       <col class="col-pro">
     </colgroup>
     <thead>
       <tr>
         <th class="col-feat-h">Feature</th>
         <th class="comm-col">Community</th>
         <th class="pro-col">Pro</th>
       </tr>
     </thead>
     <tbody>

       <!-- ── Core SLAM ── -->
       <tr class="section-row">
         <td>Core SLAM capabilities</td>
         <td class="comm-col"></td>
         <td class="pro-col"></td>
       </tr>
       <tr class="data-row">
         <td>LiDAR odometry (LO / LIO)</td>
         <td class="comm-col"><span class="check-icon">&#10004;</span></td>
         <td class="pro-col"><span class="check-icon">&#10004;</span></td>
       </tr>
       <tr class="data-row">
         <td>Mapping and localization</td>
         <td class="comm-col"><span class="check-icon">&#10004;</span></td>
         <td class="pro-col"><span class="check-icon">&#10004;</span></td>
       </tr>
       <tr class="data-row">
         <td>
           Georeferencing (GNSS fusion)
           <span class="info-wrap">
             <button class="info-btn" aria-label="About georeferencing">i</button>
             <span class="info-tooltip">Georeferenced maps, trajectories or real-time poses.</span>
           </span>
         </td>
         <td class="comm-col"><span class="check-icon">&#10004;</span></td>
         <td class="pro-col"><span class="check-icon">&#10004;</span></td>
       </tr>
       <tr class="data-row">
         <td>
           Loop closure / global SLAM
           <span class="info-wrap">
             <button class="info-btn" aria-label="About loop closure">i</button>
             <span class="info-tooltip">Online (mola_mapper, live SLAM) or offline (mola_sm_loop_closure), configurable level of global accuracy.</span>
           </span>
         </td>
         <td class="comm-col"><span class="check-icon">&#10004;</span></td>
         <td class="pro-col"><span class="check-icon">&#10004;</span></td>
       </tr>
       <tr class="data-row">
         <td>State estimation (smoother, IMU, GNSS, kinematics)</td>
         <td class="comm-col"><span class="check-icon">&#10004;</span></td>
         <td class="pro-col"><span class="check-icon">&#10004;</span></td>
       </tr>
       <tr class="data-row">
         <td>CLI map tools (mm-viewer, mm2las, mm2ply, mm-filter&nbsp;&hellip;)</td>
         <td class="comm-col"><span class="check-icon">&#10004;</span></td>
         <td class="pro-col"><span class="check-icon">&#10004;</span></td>
       </tr>
       <tr class="data-row">
         <td>ROS 2 integration</td>
         <td class="comm-col"><span class="check-icon">&#10004;</span></td>
         <td class="pro-col"><span class="check-icon">&#10004;</span></td>
       </tr>

       <!-- ── Deployment & licensing ── -->
       <tr class="section-row">
         <td>Deployment &amp; licensing</td>
         <td class="comm-col"></td>
         <td class="pro-col"></td>
       </tr>
       <tr class="data-row">
         <td>
           License
           <span class="feat-note">Copyleft: derivatives must be open-source</span>
         </td>
         <td class="comm-col">GPL-3</td>
         <td class="pro-col">Commercial</td>
       </tr>
       <tr class="data-row">
         <td>Redistribution / OEM</td>
         <td class="comm-col">GPL-3 only</td>
         <td class="pro-col">OEM agreement available</td>
       </tr>
       <tr class="data-row">
         <td>Developers covered</td>
         <td class="comm-col">Unlimited</td>
         <td class="pro-col">Unlimited (one org)</td>
       </tr>
       <tr class="data-row">
         <td>Pre-built binaries</td>
         <td class="comm-col">Public ROS apt</td>
         <td class="pro-col">&#10004; Private apt, faster updates</td>
       </tr>
       <tr class="data-row">
         <td>Docker images (amd64 / arm64)</td>
         <td class="comm-col"><span class="dash-icon">&mdash;</span></td>
         <td class="pro-col"><span class="check-icon">&#10004;</span></td>
       </tr>

       <!-- ── Pro-exclusive ── -->
       <tr class="section-row">
         <td>Pro-exclusive tools &amp; services</td>
         <td class="comm-col"></td>
         <td class="pro-col"></td>
       </tr>
       <tr class="data-row">
         <td>
           Sensor extrinsic calibration (LiDAR&#8209;camera)
           <span class="feat-note">Pro: source code, to run on your own machines. Anyone: online in MOLA cloud, with tokens</span>
         </td>
         <td class="comm-col"><a href="https://app.mola-slam.org">Online (MOLA cloud)</a></td>
         <td class="pro-col"><span class="check-icon">&#10004;</span> Local + online</td>
       </tr>
       <tr class="data-row">
         <td>Map post-processing service</td>
         <td class="comm-col"><span class="dash-icon">&mdash;</span></td>
         <td class="pro-col"><span class="check-icon">&#10004;</span></td>
       </tr>
       <tr class="data-row">
         <td>Visual odometry <span class="badge-wip">in development</span></td>
         <td class="comm-col"><span class="dash-icon">&mdash;</span></td>
         <td class="pro-col">Early access</td>
       </tr>

       <!-- ── Support ── -->
       <tr class="section-row">
         <td>Support</td>
         <td class="comm-col"></td>
         <td class="pro-col"></td>
       </tr>
       <tr class="data-row">
         <td>Community support (GitHub Issues)</td>
         <td class="comm-col"><span class="check-icon">&#10004;</span></td>
         <td class="pro-col"><span class="check-icon">&#10004;</span></td>
       </tr>
       <tr class="data-row">
         <td>Priority engineering support (email / video call)</td>
         <td class="comm-col"><span class="dash-icon">&mdash;</span></td>
         <td class="pro-col"><span class="check-icon">&#10004;</span></td>
       </tr>
       <tr class="data-row">
         <td>
           Dedicated support channel
           <span class="feat-note">Private Slack channel (or Teams / email alias)</span>
         </td>
         <td class="comm-col"><span class="dash-icon">&mdash;</span></td>
         <td class="pro-col"><span class="check-icon">&#10004;</span></td>
       </tr>
       <tr class="data-row">
         <td>Custom pipeline development (consulting)</td>
         <td class="comm-col"><span class="dash-icon">&mdash;</span></td>
         <td class="pro-col">Available</td>
       </tr>

       <!-- ── Price ── -->
       <tr class="price-row">
         <td class="price-head">Price</td>
         <td class="comm-col">
           <div class="price-label">Free</div>
           <div class="price-sub">Open-source, forever</div>
         </td>
         <td class="pro-col">
           <div class="price-pro">Annual license</div>
           <div class="price-pro-sub"><a href="#consulting-and-custom-projects">Contact us for pricing &rarr;</a></div>
         </td>
       </tr>

     </tbody>
   </table>
   </div>


.. |yes| unicode:: 0x2714 .. check mark
.. |wip| replace:: *(in development)*
.. |roadmap| replace:: *(roadmap)*

.. |pro_price| replace:: **Annual license** - :ref:`contact us <mola_contact>`

|

.. _buy-pro:

Get MOLA Pro
==================

The Pro license covers your entire organization - unlimited developers, unlimited
internal machines. No per-seat fees.

To request a Pro license, please :ref:`contact us <mola_contact>` using the form below.
After purchase, you will receive:

1. **Credentials** for the private apt repository and Docker registry
2. A **welcome email** with onboarding instructions and support channel access

.. note::

   The Pro license covers **internal development and deployment** (your own robots,
   your own fleet, your own R&D). If you plan to **redistribute** MOLA as part of a
   product sold to third parties (OEM), please :ref:`contact us <mola_contact>` for
   an OEM agreement with terms tailored to your deployment volume.

|

How licensing works
=======================

The MOLA framework is composed of several repositories with different open-source licenses:

.. list-table::
   :header-rows: 1
   :widths: 40 20 40

   * - Repository
     - License
     - Description

   * - `MRPT <https://github.com/MRPT/mrpt>`_
     - BSD-3
     - Core C++ data structures, algorithms, serialization

   * - `mp2p_icp <https://github.com/MOLAorg/mp2p_icp/>`_
     - BSD-3
     - Generic ICP algorithm, metric map pipelines

   * - `mrpt_navigation <https://github.com/mrpt-ros-pkg/mrpt_navigation/>`_
     - BSD-3
     - ROS 2 nodes: map server, localization, point cloud pipeline

   * - `MOLA <https://github.com/MOLAorg/mola>`_
     - GPL-3 (BSD-3: ``mola_bridge_ros2``, ``mola_yaml``, ``mola_traj_tools``, ``mola_demos``)
     - MOLA kernel, launcher, dataset inputs, metric maps, visualization, relocalization

   * - `mola_lidar_odometry <https://github.com/MOLAorg/mola_lidar_odometry/>`_
     - GPL-3
     - LiDAR odometry for mapping and localization

   * - `mola_state_estimation <https://github.com/MOLAorg/mola_state_estimation/>`_
     - GPL-3
     - State estimators: smoother, IMU/GNSS/kinematics fusion; map and trajectory georeferencing tools

   * - `mola_mapper <https://github.com/MOLAorg/mola_mapper/>`_
     - GPL-3
     - Live SLAM: global keyframe map, online loop closure, global pose graph, georeferencing

   * - `mola_sm_loop_closure <https://github.com/MOLAorg/mola_sm_loop_closure/>`_
     - GPL-3
     - Offline loop closure of simple-maps, for consistent large maps

**BSD-3 modules** (MRPT, mp2p_icp, mrpt_navigation, and the BSD-3 packages of the MOLA
repository listed above) can be freely used in any project, including closed-source
commercial products.

**GPL-3 modules** (MOLA core, lidar odometry, state estimation, mapper, loop closure) require
that derivative works also be released under GPL-3. If your product is closed-source,
you need a **MOLA Pro commercial license**.

|

Frequently asked questions
=============================

**Do I need Pro for research or academic use?**
  No. The Community (open-source) version is fully functional for research,
  teaching, and personal projects under the GPL-3 license.

**Can I evaluate MOLA before purchasing?**
  Yes. The open-source version has the same core algorithms. Pro adds deployment
  convenience (commercial license, binaries, Docker) and exclusive tools/support.

**What if I only use the BSD-3 modules (MRPT, mp2p_icp)?**
  You do not need a Pro license. BSD-3 modules can be used freely in any project.

**What is the difference between Pro and OEM?**
  Pro covers internal use - your team develops and deploys MOLA on your own robots
  and infrastructure. OEM covers redistribution - you ship a product with MOLA
  embedded to your end customers. OEM agreements are negotiated individually based
  on deployment volume. :ref:`Contact us <mola_contact>` for OEM terms.

**How many developers / machines does Pro cover?**
  Unlimited, within one organization. No per-seat or per-machine fees.

|

.. _mola_contact:

Consulting and custom projects
==================================

Beyond the Pro license, we offer:

- **Custom sensor driver development** (Livox, Hesai, RoboSense, and others)
- **Custom pipeline development** for your specific sensors and environment
- **Fleet deployment consulting** (multi-robot map sharing, configuration management)
- **On-site or remote training** for your engineering team
- **Map post-processing service** - send us your raw sensor data and receive
  optimized, georeferenced metric maps, or build them yourself online: upload a
  ROS bag to `MOLA cloud <https://app.mola-slam.org>`_

For consulting inquiries or demo requests, use the form below:

.. raw:: html

    <div style="margin-top:10px;">
      <iframe src="https://docs.google.com/forms/d/e/1FAIpQLSdgFfPclN7MuB4uKIbENxUDgC-pmimcu_PGcq5-vAALjUAOrg/viewform?embedded=true" width="700" height="1500" frameborder="0" marginheight="0" marginwidth="0">Loading...</iframe>
    </div>
