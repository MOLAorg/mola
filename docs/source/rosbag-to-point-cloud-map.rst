:og:description: Upload a ROS 1 or ROS 2 bag recorded with a 3D LiDAR and get a 3D point cloud map built with LiDAR SLAM and loop closure: LAS and PLY files, the trajectory, geo-referenced outputs when there is GNSS, and a 3D viewer in the browser.

.. meta::
   :description: Turn a ROS bag into a 3D point cloud map online, with LiDAR SLAM and loop closure. LAS and PLY files, trajectory, GNSS geo-referencing and a 3D viewer. No install.

.. _rosbag-to-point-cloud-map:

======================================
ROS bag to 3D point cloud map, online
======================================

.. raw:: html

   <div class="mc-hero">
     <div>
       <p class="mc-eyebrow">MOLA cloud &middot; online tool</p>
       <p class="mc-lead">Upload a recording from a 3D LiDAR, with an IMU and a GNSS receiver if
         you have them, and get back a consistent 3D map: LAS and PLY point clouds, the trajectory,
         and a 3D viewer in your browser. It runs the same LiDAR SLAM that MOLA runs on robots, with
         nothing to install.</p>
       <p class="mc-actions">
         <a class="mola-btn mola-btn-primary" href="https://app.mola-slam.org">Build a map online</a>
         <a class="mola-btn mc-btn-outline" href="#what-you-get">What you get</a>
       </p>
       <p class="mc-note">Sign in with GitHub or an email address.</p>
     </div>
     <figure class="mc-hero-art">
       <img src="https://mrpt.github.io/imgs/mola-lio-slam-on-greenbot-dataset.jpg" alt="3D point cloud map of a greenhouse built with MOLA LiDAR-inertial SLAM" width="1920" height="1080">
       <figcaption>A greenhouse mapped with MOLA LiDAR-inertial SLAM (GreenBot dataset).</figcaption>
     </figure>
   </div>

   <ul class="mc-facts">
     <li><strong>Input</strong>ROS 2 bag (<code>.mcap</code> or a zipped <code>.db3</code>) or ROS 1 <code>.bag</code></li>
     <li><strong>Sensors</strong>A 3D LiDAR; optionally an IMU, GNSS and wheel odometry</li>
     <li><strong>Method</strong>LiDAR-inertial SLAM with loop closure, or LiDAR odometry</li>
     <li><strong>Output</strong>LAS, PLY, trajectory, MOLA maps; KMZ and WGS-84 with GNSS</li>
   </ul>


How it works
=============

.. raw:: html

   <ol class="mc-steps">
     <li><strong>Upload the bag</strong>From the browser, with uploads that resume if the
       connection drops, or by URL from S3 (public or presigned links), Zenodo, Hugging Face or
       Google Cloud Storage.</li>
     <li><strong>Check the topics</strong>The app reads the bag and proposes the LiDAR, IMU and
       GNSS topics, and the vehicle frame from <code>/tf</code>. Change them if needed.</li>
     <li><strong>Choose the method</strong>SLAM with loop closure for a globally consistent map,
       or plain odometry. Recordings without an IMU use LiDAR-only odometry.</li>
     <li><strong>Download the map</strong>Follow the progress and the log live. You get an email
       when the map is ready.</li>
   </ol>

As an example, a 961 m route around a university campus, recorded for 14 minutes, gave a map of
9.8 million points with 18 loop closures, in 15 minutes for the whole job.


.. _what-you-get:

What you get
==============

.. list-table::
   :header-rows: 1
   :widths: 28 72

   * - Result
     - Details
   * - Point cloud map
     - LAS and PLY files, for CloudCompare, QGIS and surveying or GIS software.
   * - Trajectory
     - TUM format, corrected by the loop closures. The odometry-only trajectory is included too.
   * - MOLA maps
     - The metric map (``.mm``) and the keyframe map (``.simplemap``), to reuse with the
       :ref:`MOLA map tools <map-tools>` or for :ref:`localization <localization-only>`.
   * - 3D viewer
     - In the browser, also on phones: color by height or intensity, trajectory overlay, full
       screen.
   * - Report
     - Map and trajectory previews, and statistics: duration, distance, loop closures and number
       of points. The log lists the exact MOLA version used.


Geo-referenced maps with GNSS
===============================

When the bag has a ``sensor_msgs/NavSatFix`` topic, the map is geo-referenced. The app lists the
GNSS receivers in the recording, best first, and marks RTK receivers. With the SLAM method, the
GNSS fixes go into the SLAM graph, with settings tuned for RTK precision when the receiver
reports it.

You also get:

- A LAS point cloud in WGS-84 coordinates.
- A KMZ file to open the map and the trajectory in Google Earth.
- The trajectory in WGS-84, as CSV, and the geo-reference itself (``georef.yaml``).
- An **aerial view** in the 3D viewer: the map over orthophotos where free national imagery
  exists (Spain, Switzerland, the Netherlands, Austria, France, the United States and Japan),
  and over OpenStreetMap elsewhere.

.. raw:: html

   <figure class="mc-figure mc-figure-narrow">
     <img src="https://mrpt.github.io/imgs/kaist01_georef_sample.png" alt="Geo-referenced 3D LiDAR map over satellite imagery of a university campus" width="773" height="684" loading="lazy">
     <figcaption>A MOLA map geo-referenced with GNSS, over satellite imagery (KAIST campus,
       MulRan dataset).</figcaption>
   </figure>

See :ref:`geo-referencing` for how MOLA fuses GNSS with LiDAR odometry.


What your recording needs
===========================

- A **3D LiDAR** recorded as ``sensor_msgs/PointCloud2``, ideally with per-point timestamps so
  that each scan can be corrected for the motion during the sweep.
- ``/tf`` and ``/tf_static`` with the pose of each sensor on the vehicle. Without a vehicle frame,
  the map is built in the LiDAR frame and the IMU is not used.
- Optionally, an **IMU** (``sensor_msgs/Imu``), a **GNSS receiver** (``sensor_msgs/NavSatFix``)
  and **wheel odometry** (``nav_msgs/Odometry``).

:ref:`Run MOLA on your own ROS 2 bag <your_own_rosbag>` explains how to check a bag, and the same
advice applies here.



Frequently asked questions
============================

Which LiDARs are supported?
  Any 3D LiDAR recorded as ``sensor_msgs/PointCloud2``, such as Ouster, Velodyne or RoboSense
  sensors. See :ref:`supported_sensors`.

Does it work without an IMU?
  Yes. Choose LiDAR odometry, which uses the LiDAR alone. An IMU makes the result more robust in
  fast motion and in places with little structure.

Can I use the map to localize a robot?
  Yes. Download the ``.mm`` metric map and use it with MOLA's
  :ref:`localization mode <localization-only>`.

Where is my data processed?
  On our server in Spain (EU). Each job runs in an isolated container with no network access.

Can I build the same map on my own computer?
  Yes. MOLA is open source: follow :ref:`Build a map <building-maps>`. MOLA cloud runs the same
  software, without the installation and with the settings already chosen for you.


.. raw:: html

   <div class="mc-cta">
     <div>
       <p class="mc-cta-title">Turn your ROS bag into a map</p>
       <p class="mc-cta-sub">Upload a LiDAR recording and download LAS and PLY files.</p>
     </div>
     <a class="mola-btn mc-btn-inverse" href="https://app.mola-slam.org">Open MOLA cloud</a>
   </div>

   <div class="mc-related">
     <a href="camera-calibration-online.html"><strong>Camera calibration</strong><span>Get the camera intrinsics from a video of a pattern, or with no pattern at all.</span></a>
     <a href="lidar-camera-calibration-online.html"><strong>LiDAR-camera calibration</strong><span>Find where the camera sits relative to the LiDAR, from a ROS bag of a target board.</span></a>
   </div>
