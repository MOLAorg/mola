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

   <style>
   .mola-pricing-wrap { margin: 2rem 0; font-size: 14px; font-family: inherit; }

   .mola-pricing-table {
     width: 100%;
     border-collapse: collapse;
     border: 1px solid #d0d7de;
     border-radius: 8px;
     overflow: hidden;
   }

   /* Column widths */
   .mola-pricing-table col.col-feat   { width: 52%; }
   .mola-pricing-table col.col-comm   { width: 24%; }
   .mola-pricing-table col.col-pro    { width: 24%; }

   /* Pro column tint */
   .mola-pricing-table td.pro-col,
   .mola-pricing-table th.pro-col { background: #eef5fc; }

   /* Header row */
   .mola-pricing-table thead tr th {
     padding: 12px 16px;
     text-align: center;
     font-size: 15px;
     font-weight: 600;
     border-bottom: 1px solid #d0d7de;
     color: #1f2328;
   }
   .mola-pricing-table thead tr th.col-feat-h {
     text-align: left;
     font-weight: 400;
     font-size: 13px;
     color: #57606a;
   }
   .mola-pricing-table thead tr th.pro-col {
     color: #0550ae;
     background: #dbeafe;
     border-bottom-color: #bfdbfe;
   }

   /* Section header rows */
   .mola-pricing-table tr.section-row td {
     padding: 8px 16px 5px;
     font-size: 11px;
     font-weight: 600;
     letter-spacing: 0.07em;
     text-transform: uppercase;
     color: #57606a;
     background: #f6f8fa;
     border-top: 1px solid #d0d7de;
     border-bottom: 1px solid #d0d7de;
   }
   .mola-pricing-table tr.section-row td.pro-col {
     background: #dbeafe;
   }

   /* Data rows */
   .mola-pricing-table tr.data-row td {
     padding: 9px 16px;
     border-bottom: 1px solid #eaeef2;
     vertical-align: middle;
     color: #1f2328;
     line-height: 1.45;
   }
   .mola-pricing-table tr.data-row:last-child td { border-bottom: none; }

   .mola-pricing-table tr.data-row td.comm-col,
   .mola-pricing-table tr.data-row td.pro-col {
     text-align: center;
     font-size: 13px;
     color: #0550ae;
     font-weight: 500;
   }
   .mola-pricing-table tr.data-row td.comm-col {
     color: #57606a;
   }

   /* Price row */
   .mola-pricing-table tr.price-row td {
     padding: 16px 16px;
     border-top: 2px solid #d0d7de;
     vertical-align: middle;
   }
   .mola-pricing-table tr.price-row td.comm-col { text-align: center; }
   .mola-pricing-table tr.price-row td.pro-col  { text-align: center; background: #dbeafe; }

   .price-label    { font-size: 22px; font-weight: 700; color: #1f2328; }
   .price-sub      { font-size: 12px; color: #57606a; margin-top: 3px; }
   .price-pro      { font-size: 14px; font-weight: 600; color: #0550ae; }
   .price-pro-sub  { font-size: 12px; color: #3b82b8; margin-top: 3px; }

   /* Icons */
   .check-icon { color: #1a7f37; font-size: 15px; }
   .dash-icon  { color: #bbb; font-size: 18px; line-height: 1; }

   /* Info tooltip */
   .info-wrap { display: inline-block; position: relative; margin-left: 5px; vertical-align: middle; }
   .info-btn {
     display: inline-flex; align-items: center; justify-content: center;
     width: 15px; height: 15px; border-radius: 50%;
     background: #d0d7de; color: #57606a;
     font-size: 10px; font-weight: 700; font-style: normal;
     cursor: pointer; border: none; padding: 0;
     font-family: inherit; line-height: 1;
     transition: background 0.15s;
   }
   .info-btn:hover { background: #0550ae; color: #fff; }
   .info-tooltip {
     display: none;
     position: absolute;
     left: 50%; bottom: calc(100% + 6px);
     transform: translateX(-50%);
     width: 230px;
     background: #1f2328; color: #fff;
     font-size: 12px; line-height: 1.5;
     padding: 8px 10px; border-radius: 6px;
     z-index: 99; font-weight: 400; text-align: left;
     pointer-events: none;
   }
   .info-tooltip::after {
     content: '';
     position: absolute; top: 100%; left: 50%;
     transform: translateX(-50%);
     border: 5px solid transparent;
     border-top-color: #1f2328;
   }
   .info-wrap:hover .info-tooltip,
   .info-btn:focus + .info-tooltip { display: block; }

   /* Badges */
   .badge-wip {
     display: inline-block; font-size: 10px; padding: 1px 6px;
     background: #fff8c5; color: #7d4e00;
     border: 1px solid #e8c84a; border-radius: 4px;
     margin-left: 6px; vertical-align: middle; font-weight: 500;
     white-space: nowrap;
   }
   .badge-road {
     display: inline-block; font-size: 10px; padding: 1px 6px;
     background: #ddf4ff; color: #0550ae;
     border: 1px solid #80caff; border-radius: 4px;
     margin-left: 6px; vertical-align: middle; font-weight: 500;
     white-space: nowrap;
   }
   .feat-note {
     display: block; font-size: 11px; color: #57606a;
     margin-top: 2px; font-weight: 400;
   }
   </style>

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
             <span class="info-tooltip">Online or offline versions, configurable level of global accuracy.</span>
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
         <td>Sensor extrinsic calibration (LiDAR&#8209;camera)</td>
         <td class="comm-col"><span class="dash-icon">&mdash;</span></td>
         <td class="pro-col"><span class="check-icon">&#10004;</span></td>
       </tr>
       <tr class="data-row">
         <td>Map post-processing service</td>
         <td class="comm-col"><span class="dash-icon">&mdash;</span></td>
         <td class="pro-col"><span class="check-icon">&#10004;</span></td>
       </tr>
       <tr class="data-row">
         <td>2D mapper <span class="badge-wip">in development</span></td>
         <td class="comm-col"><span class="dash-icon">&mdash;</span></td>
         <td class="pro-col">Early access</td>
       </tr>
       <tr class="data-row">
         <td>3D mapper <span class="badge-wip">in development</span></td>
         <td class="comm-col"><span class="dash-icon">&mdash;</span></td>
         <td class="pro-col">Early access</td>
       </tr>
       <tr class="data-row">
         <td>Visual odometry <span class="badge-road">roadmap</span></td>
         <td class="comm-col"><span class="dash-icon">&mdash;</span></td>
         <td class="pro-col">Early access when available</td>
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
         <td style="color:#57606a; font-size:13px;">Price</td>
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
     - GPL-3
     - MOLA kernel, visualization, relocalization

   * - `mola_lidar_odometry <https://github.com/MOLAorg/mola_lidar_odometry/>`_
     - GPL-3
     - LiDAR odometry for mapping and localization

   * - `mola_state_estimation <https://github.com/MOLAorg/mola_state_estimation/>`_
     - GPL-3
     - State estimators: smoother, IMU/GNSS/kinematics fusion

   * - `mola_sm_loop_closure <https://github.com/MOLAorg/mola_sm_loop_closure/>`_
     - GPL-3
     - Map georeferencing, loop closure for consistent large maps

**BSD-3 modules** (MRPT, mp2p_icp, mrpt_navigation) can be freely used in any project,
including closed-source commercial products.

**GPL-3 modules** (MOLA core, lidar odometry, state estimation, loop closure) require
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
  optimized, georeferenced metric maps

For consulting inquiries or demo requests, use the form below:

.. raw:: html

    <div style="margin-top:10px;">
      <iframe src="https://docs.google.com/forms/d/e/1FAIpQLSdgFfPclN7MuB4uKIbENxUDgC-pmimcu_PGcq5-vAALjUAOrg/viewform?embedded=true" width="700" height="1500" frameborder="0" marginheight="0" marginwidth="0">Loading...</iframe>
    </div>
