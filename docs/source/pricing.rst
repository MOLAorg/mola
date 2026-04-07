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

.. list-table::
   :header-rows: 1
   :widths: 50 25 25

   * - Feature
     - Community
     - Pro

   * - **Core SLAM capabilities**
     -
     -

   * - LiDAR odometry (LO / LIO)
     - |yes|
     - |yes|

   * - Mapping and localization
     - |yes|
     - |yes|

   * - Georeferencing (GNSS fusion)
     - |yes|
     - |yes|

   * - Loop closure / global SLAM
     - |yes|
     - |yes|

   * - State estimation (smoother, IMU, GNSS, kinematics)
     - |yes|
     - |yes|

   * - CLI map tools (mm-viewer, mm2las, mm2ply, mm-filter, ...)
     - |yes|
     - |yes|

   * - ROS 2 integration
     - |yes|
     - |yes|

   * -
     -
     -

   * - **Deployment & licensing**
     -
     -

   * - License
     - GPL-3 (copyleft - derivatives must be open-source)
     - Commercial (closed-source internal use)

   * - Redistribution / OEM (ship MOLA inside your product)
     - GPL-3 (derivative must be open-source)
     - Requires OEM agreement - :ref:`contact us <mola_contact>`

   * - Developers covered
     - Unlimited
     - Unlimited within one organization

   * - Pre-built binaries (private apt repository)
     - Public ROS apt only
     - |yes| Private apt with faster updates

   * - Docker images (amd64 / arm64)
     - ---
     - |yes|

   * -
     -
     -

   * - **Pro-exclusive tools & services**
     -
     -

   * - Sensor extrinsic calibration (LiDAR-camera)
     - ---
     - |yes|

   * - Map post-processing service (send raw data, receive optimized maps)
     - ---
     - |yes|

   * - 2D mapper |wip|
     - ---
     - Early access

   * - 3D mapper |wip|
     - ---
     - Early access

   * - Visual odometry |roadmap|
     - ---
     - Early access when available

   * -
     -
     -

   * - **Support**
     -
     -

   * - Community support (GitHub Issues)
     - |yes|
     - |yes|

   * - Priority engineering support (email / video call)
     - ---
     - |yes|

   * - Dedicated support channel
     - ---
     - |yes|

   * - Custom pipeline development (consulting)
     - ---
     - Available

   * -
     -
     -

   * - **Price**
     - **Free**
     - |pro_price|


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

**Can I get a trial of Pro?**
  Contact us for a time-limited evaluation license.


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

For consulting inquiries, demo requests, or Enterprise pricing, use the form below:

.. raw:: html

    <div style="margin-top:10px;">
      <iframe src="https://docs.google.com/forms/d/e/1FAIpQLSdgFfPclN7MuB4uKIbENxUDgC-pmimcu_PGcq5-vAALjUAOrg/viewform?embedded=true" width="700" height="1500" frameborder="0" marginheight="0" marginwidth="0">Loading...</iframe>
    </div>
