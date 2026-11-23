:og:description: Calibrate a camera online from a phone video, a folder of images or a ROS 2 bag. Checkerboard, ChArUco, circle grids, AprilGrid, or no pattern at all. Pinhole and fisheye lenses. Results as ROS camera_info, OpenCV and MRPT YAML files.
:og:image: _static/social/camera-calibration-online.png
:og:image:alt: Camera calibration online, from a video, with MOLA cloud

.. meta::
   :description: Calibrate a camera online from a video, a folder of images or a ROS 2 bag. Checkerboard, ChArUco, circle grid, AprilGrid or no pattern. Pinhole and fisheye lenses.

.. _camera-calibration-online:

=========================================
Camera calibration online, from a video
=========================================

.. raw:: html

   <div class="mc-hero">
     <div>
       <p class="mc-eyebrow">MOLA cloud &middot; online tool</p>
       <p class="mc-lead">Record a short video of a calibration pattern with the camera you want to
         calibrate, upload it, and download the camera's intrinsic parameters: focal length,
         principal point and lens distortion. There is nothing to install.</p>
       <p class="mc-actions">
         <a class="mola-btn mola-btn-primary" href="https://app.mola-slam.org">Calibrate a camera online</a>
         <a class="mola-btn mc-btn-outline" href="#supported-calibration-patterns">See the patterns</a>
       </p>
       <p class="mc-note">Sign in with GitHub or an email address.</p>
     </div>
     <figure class="mc-hero-art">
       <img src="_static/cloud/pattern-aprilgrid.svg" alt="AprilGrid camera calibration target with AprilTag 36h11 markers" width="436" height="353">
     </figure>
   </div>

   <ul class="mc-facts">
     <li><strong>Input</strong>Video (MP4, MOV, MKV, WebM), a ZIP of images, or a ROS 2 bag</li>
     <li><strong>Patterns</strong>Checkerboard, ChArUco, circle grids, AprilGrid, or none</li>
     <li><strong>Lenses</strong>Pinhole with radial-tangential distortion, and fisheye</li>
     <li><strong>Output</strong>ROS <code>camera_info</code>, OpenCV and MRPT YAML files</li>
   </ul>


How it works
=============

.. raw:: html

   <ol class="mc-steps">
     <li><strong>Print a pattern and record</strong>Film the pattern with the camera to calibrate,
       moving slowly so it shows up all over the image, near and far, and tilted. One or two
       minutes of video is usually enough.</li>
     <li><strong>Upload the recording</strong>Uploads resume if the connection drops. You can also
       import a file from S3, Zenodo, Hugging Face or Google Cloud Storage by URL.</li>
     <li><strong>Check the detected pattern</strong>The app finds the pattern in a few frames and
       fills in its type, rows and columns. Check them, choose the lens model, and launch the job.</li>
     <li><strong>Download the calibration</strong>Get the parameter files, the RMS reprojection
       error and a plot of which parts of the image the pattern covered.</li>
   </ol>

The job looks for the pattern in the whole recording and keeps the 60 most diverse views, so a
long video with many similar frames does no harm. Views with an error far above the rest are left
out, with a warning. If the error stays high, the job fails and tells you which pattern size the
recording seems to show: a wrong number of rows or columns is the most common mistake.


.. _supported-calibration-patterns:

Supported calibration patterns
================================

.. raw:: html

   <div class="mc-grid">
     <figure class="mc-item">
       <div class="mc-item-art"><img src="_static/cloud/pattern-checkerboard.svg" alt="Checkerboard calibration pattern" width="372" height="300" loading="lazy"></div>
       <figcaption><strong>Checkerboard</strong>The classic target. Count the inner corners, where
         four squares meet, not the squares: this one has 8 &times; 6.</figcaption>
     </figure>
     <figure class="mc-item">
       <div class="mc-item-art"><img src="_static/cloud/pattern-charuco.svg" alt="ChArUco calibration board" width="370" height="278" loading="lazy"></div>
       <figcaption><strong>ChArUco</strong>A checkerboard with ArUco markers in its white squares.
         It still works when part of the board is out of view, which helps to cover the image
         corners.</figcaption>
     </figure>
     <figure class="mc-item">
       <div class="mc-item-art"><img src="_static/cloud/pattern-aprilgrid.svg" alt="AprilGrid calibration target" width="436" height="353" loading="lazy"></div>
       <figcaption><strong>AprilGrid</strong>The Kalibr target: AprilTag 36h11 tags with small black
         squares between them. Every tag is identified, so partial views count too.</figcaption>
     </figure>
     <figure class="mc-item">
       <div class="mc-item-art"><img src="_static/cloud/pattern-circles.svg" alt="Symmetric circle grid calibration pattern" width="336" height="244" loading="lazy"></div>
       <figcaption><strong>Circle grid</strong>Rows and columns of black circles on white.</figcaption>
     </figure>
     <figure class="mc-item">
       <div class="mc-item-art"><img src="_static/cloud/pattern-asymmetric-circles.svg" alt="Asymmetric circle grid calibration pattern" width="360" height="270" loading="lazy"></div>
       <figcaption><strong>Asymmetric circle grid</strong>Offset rows of circles, like OpenCV's
         standard 4 &times; 11 pattern. It had the lowest error of all patterns in our synthetic
         tests.</figcaption>
     </figure>
     <figure class="mc-item">
       <div class="mc-item-art mc-item-art-text">
         <div>
           <svg viewBox="0 0 24 24" aria-hidden="true"><path d="M23 19a2 2 0 0 1-2 2H3a2 2 0 0 1-2-2V8a2 2 0 0 1 2-2h4l2-3h6l2 3h4a2 2 0 0 1 2 2z"/><circle cx="12" cy="13" r="4"/></svg>
           <div>Any static, textured scene</div>
         </div>
       </div>
       <figcaption><strong>No pattern <span class="mc-badge">Experimental</span></strong>Film a
         static scene while turning the camera in every direction. The focal length and distortion
         are estimated from tracked image features.</figcaption>
     </figure>
   </div>

The app has a `guide to the calibration patterns <https://app.mola-slam.org/calibration-patterns>`_
with what to count on each one, how to print it and how to record it.


Lens models and output files
==============================

Two lens models are available: **pinhole** with radial-tangential distortion (five coefficients,
``plumb_bob`` in ROS), for most cameras, and **fisheye** (equidistant, also known as
Kannala-Brandt), for wide-angle lenses.

.. list-table::
   :header-rows: 1
   :widths: 30 70

   * - File
     - What it is for
   * - ``camera_info.yaml``
     - ROS ``camera_info`` format. Pass it to your camera driver with
       ``camera_info_url: file:///path/to/camera_info.yaml``.
   * - OpenCV YAML
     - Camera matrix and distortion coefficients for OpenCV (``cv::undistort``,
       ``cv::fisheye``).
   * - ``mrpt_camera.yaml``
     - MRPT camera model, loaded by ``mrpt::img::TCamera::FromYAML()``: the format MOLA and the
       :ref:`LiDAR-camera calibration <lidar-camera-calibration-online>` use.
   * - Report
     - RMS reprojection error in pixels, number of views used, and a coverage plot.

Without a pattern, the results page also shows the reconstructed scene in 3D, with one camera
frustum per view and the points in their real colors, so you can see what the calibration is
based on.


How accurate is it?
=====================

We checked the results against the official calibrations of the
`EuRoC MAV dataset <https://projects.asl.ethz.ch/datasets/euroc-mav/>`_, and against synthetic
images rendered with known parameters:

.. list-table::
   :header-rows: 1
   :widths: 45 55

   * - Test
     - Result
   * - EuRoC checkerboard sequence (752 × 480)
     - Focal length about 0.5 % from the official values, in 69 s
   * - EuRoC AprilGrid sequence
     - Focal length about 0.5 % from the official values, principal point within 1 px,
       RMS error 0.17 px
   * - Synthetic images, every pattern, pinhole and fisheye
     - Focal length within 0.4 %
   * - No pattern: EuRoC sequence as a 60 s MP4 video
     - Focal length within 0.3 %, in 7 minutes
   * - No pattern: other real recordings
     - 0.35 % on fisheye lenses; 0.7 % to 3.7 % on pinhole cameras, like COLMAP on the same data

With a pattern and a good video, the RMS error is usually well below one pixel. Without one, the accuracy depends on how much the
camera rotates in the video. When the motion cannot determine the focal length, as with a camera
on a robot that only drives on flat ground, the job says so instead of returning a wrong
calibration.


Tips for a good calibration video
===================================

- **Lock the focus and the zoom.** A camera that refocuses changes its focal length in the middle
  of the video. On most phones, a long press on the screen locks focus and exposure.
- **Turn off video stabilization**, unless your application uses stabilized video too: it crops
  and warps every frame.
- **Record at the resolution you will use.** The parameters are in pixels of that resolution.
- **Cover the whole image, corners included.** Lens distortion is strongest near the borders.
- **Tilt the pattern**, up to about 45 degrees, and change the distance. Views that all face the
  camera straight on leave the focal length poorly determined.
- **Move slowly in good light.** Blurred frames are rejected.
- **Keep the pattern flat**: glue the print to a rigid board, or show it on a flat monitor.
- **Without a pattern**, turn the camera about all three axes (pan, tilt and roll) in a static
  scene with texture at several depths. Walking straight ahead is not enough.



Frequently asked questions
============================

Do I need to install anything?
  No. You only need a web browser and an account.

What do I count on a checkerboard?
  The inner corners, where four squares meet. A board of 9 × 7 squares has 8 × 6 inner corners.
  The app detects the size and fills it in; check it before you launch the job.

Does the size of the squares matter?
  Enter it as printed. The focal length, principal point and distortion are in pixels and do not
  depend on it, so a small error in the measured size does not change them.

Can I calibrate a fisheye or wide-angle lens?
  Yes. Choose the fisheye model, which handles fields of view that the pinhole model cannot.

Can I use a ROS bag instead of a video?
  Yes, a ROS 2 bag (``.mcap``, or a zipped ``.db3`` folder) with an image topic. ROS 1 bags are
  not supported yet for this job.

Where is my data processed?
  On our server in Spain (EU). Each job runs in an isolated container with no network access.

How do I use the result with my LiDAR?
  Run a :ref:`LiDAR-camera calibration <lidar-camera-calibration-online>` next: it takes the
  result of this job as the camera's intrinsics.


.. raw:: html

   <div class="mc-cta">
     <div>
       <p class="mc-cta-title">Calibrate your camera now</p>
       <p class="mc-cta-sub">Upload a video from your phone or your robot. It takes a few minutes.</p>
     </div>
     <a class="mola-btn mc-btn-inverse" href="https://app.mola-slam.org">Open MOLA cloud</a>
   </div>

   <div class="mc-related">
     <a href="lidar-camera-calibration-online.html"><strong>LiDAR-camera calibration</strong><span>Find where the camera sits relative to the LiDAR, from a ROS bag of a target board.</span></a>
     <a href="rosbag-to-point-cloud-map.html"><strong>ROS bag to 3D map</strong><span>Upload a LiDAR recording and get a point cloud map with LiDAR SLAM.</span></a>
   </div>
