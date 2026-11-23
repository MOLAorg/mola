:og:description: Find the extrinsic calibration between a LiDAR and a camera online. Record a ROS bag of a board with four holes and four ArUco markers, upload it, and get the camera pose with its residuals and images of the LiDAR points drawn over the camera view.

.. meta::
   :description: LiDAR-camera extrinsic calibration online: record a ROS bag of a target board with four holes and four ArUco markers, upload it, and get the camera pose and overlay images.

.. _lidar-camera-calibration-online:

================================
LiDAR-camera calibration online
================================

.. raw:: html

   <div class="mc-hero">
     <div>
       <p class="mc-eyebrow">MOLA cloud &middot; online tool</p>
       <p class="mc-lead">Record a calibration board in front of your LiDAR and camera, upload the
         ROS bag, and get the camera's position and orientation relative to the LiDAR: the
         extrinsic calibration. Images of the LiDAR points drawn over the camera view show that
         it is right.</p>
       <p class="mc-actions">
         <a class="mola-btn mola-btn-primary" href="https://app.mola-slam.org">Calibrate LiDAR and camera online</a>
         <a class="mola-btn mc-btn-outline" href="#the-calibration-target">About the target</a>
       </p>
       <p class="mc-note">Sign in with GitHub or an email address.</p>
     </div>
     <figure class="mc-hero-art">
       <img src="_static/cloud/lidar-camera-overlay.svg" alt="LiDAR points colored by range, projected onto a camera image of the calibration target" width="640" height="400">
       <figcaption>Schematic of an overlay image: LiDAR points drawn over the camera view. The
         rings that pass through the holes hit the wall behind the board.</figcaption>
     </figure>
   </div>

   <ul class="mc-facts">
     <li><strong>Input</strong>A ROS bag with the point cloud, the camera images and <code>/tf</code></li>
     <li><strong>Target</strong>A rigid board with four circular holes and four ArUco markers</li>
     <li><strong>Output</strong>Camera pose as YAML, residuals, and overlay images</li>
     <li><strong>Intrinsics</strong>From the bag's <code>CameraInfo</code>, a YAML file, or a camera calibration job</li>
   </ul>


How it works
=============

.. raw:: html

   <ol class="mc-steps">
     <li><strong>Get a target</strong>Buy the ready-made board, or build one: its geometry is
       below.</li>
     <li><strong>Record a ROS bag</strong>Hold the board still in front of both sensors for a few
       seconds, then move it to another place and angle. Repeat for at least three positions.</li>
     <li><strong>Upload and describe the board</strong>Pick a board preset or type its dimensions,
       and check the LiDAR and camera topics the app proposes.</li>
     <li><strong>Download the result</strong>The camera pose in the reference frame you choose,
       with its residuals and the overlay images.</li>
   </ol>

The frames where the board is still and visible to both sensors are chosen automatically, so
there is no frame-by-frame review to do.


.. _the-calibration-target:

The calibration target
========================

.. raw:: html

   <figure class="mc-figure mc-figure-narrow">
     <img src="_static/cloud/lidar-camera-target.svg" alt="LiDAR-camera calibration target with four circular holes and four ArUco markers in the corners" width="480" height="360" loading="lazy">
     <figcaption>The target, with the dimensions of the ready-made board: holes of 9 cm radius
       whose centers are 36 &times; 35 cm apart, and 15 cm ArUco markers (6&times;6 dictionary,
       IDs 1 to 4) in the corners. The outline of the board is schematic.</figcaption>
   </figure>

The LiDAR finds the four **holes** in the point cloud, and the camera finds the four **ArUco
markers** in the image. Since the board geometry relates both, every position of the board ties
the two sensors together.

The board form in the app has three presets, and you can type your own dimensions (hole radius,
hole spacing, marker size and position, ArUco dictionary and IDs):

- The ready-made board sold by `BYMIG <https://bymig.com/patron-calibracion-lidar-camara/>`_
  (Spain).
- The board used by the FAST-Calib project.
- MOLA's own design.

To build your own, cut the holes precisely (laser or CNC) in a rigid, flat sheet, print the
markers at their exact size, and glue them flat in place.


Recording tips
================

- **Leave space behind the board.** The LiDAR must see through the holes to a surface farther
  away, such as a wall, so that the edges of the holes stand out.
- **Fit the whole board in the camera image**, with several LiDAR rings crossing every hole.
  With a LiDAR of few beams, bring the board closer or tilt it.
- **Keep the board still** for a few seconds at each position.
- **Vary the positions**: left and right, near and far, turned and tilted. Three is the minimum;
  more give a better-conditioned result.
- Record the point cloud, the camera images (and their ``CameraInfo`` if the camera publishes
  it), ``/tf`` and ``/tf_static``.


What you get
==============

- A **YAML file with the camera pose** relative to the LiDAR, or to another frame of your
  ``/tf`` tree, ready to publish as a static transform.
- The **residuals** of the fit.
- **Overlay images**: the LiDAR points of each selected frame drawn over the camera image. When
  the calibration is right, the points line up with the edges of the holes and of the board.

The camera intrinsics come from the ``CameraInfo`` messages in the bag, from a YAML file you
upload, or from a :ref:`camera calibration <camera-calibration-online>` you ran before.


Running it on your own machines
=================================

To run the calibration on your own machines, the source code of the tool is included in
:ref:`MOLA Pro <pricing>`.

**Coming soon, for Pro accounts:** calibrating every LiDAR, camera, IMU and GNSS antenna of a
vehicle at once, without a target, from a drive in circles and figure-eights.


Frequently asked questions
============================

Which LiDARs can I calibrate?
  Any LiDAR recorded as ``sensor_msgs/PointCloud2``. The holes are easier to find with more
  rings across the board, so with a 16-beam LiDAR place the board closer than with a 128-beam
  one.

Do I need the camera intrinsics first?
  Yes. If your camera does not publish a calibrated ``CameraInfo``, run a
  :ref:`camera calibration online <camera-calibration-online>` first and use its result.

Where can I get a target board?
  Buy the ready-made board from `BYMIG <https://bymig.com/patron-calibracion-lidar-camara/>`_,
  or build one with the dimensions above and enter them in the board form.

Where is my data processed?
  On our server in Spain (EU). Each job runs in an isolated container with no network access.

How do I use the result in ROS 2?
  Publish the pose as a static transform between the LiDAR frame (or your vehicle frame) and the
  camera frame, in your URDF or with ``static_transform_publisher``.


.. raw:: html

   <div class="mc-cta">
     <div>
       <p class="mc-cta-title">Calibrate your LiDAR and camera</p>
       <p class="mc-cta-sub">Upload a ROS bag of the target board and get the camera pose.</p>
     </div>
     <a class="mola-btn mc-btn-inverse" href="https://app.mola-slam.org">Open MOLA cloud</a>
   </div>

   <div class="mc-related">
     <a href="camera-calibration-online.html"><strong>Camera calibration</strong><span>Get the camera intrinsics from a video of a pattern, or with no pattern at all.</span></a>
     <a href="rosbag-to-point-cloud-map.html"><strong>ROS bag to 3D map</strong><span>Upload a LiDAR recording and get a point cloud map with LiDAR SLAM.</span></a>
   </div>
