.. _tutorial-ouster-mapping-lc:

=================================================
MOLA-LIO + Loop Closure: Globally Consistent Map
=================================================

This tutorial shows you how to build a **globally consistent, centimeter-accurate 3D map**
from an Ouster LiDAR dataset using MOLA-LIO followed by offline loop closure (LC) post-processing.

.. contents::
   :depth: 1
   :local:
   :backlinks: none

|

This video summarizes the results of the complete pipeline:

.. raw:: html

    <div style="margin-top:10px;">
      <iframe width="560" height="315" src="https://www.youtube.com/embed/pVFmyqJiMws" title="YouTube video player" frameborder="0" allow="accelerometer; autoplay; clipboard-write; encrypted-media; gyroscope; picture-in-picture; web-share" referrerpolicy="strict-origin-when-cross-origin" allowfullscreen></iframe>
    </div>

|

.. note::

    This tutorial is part of the :ref:`Full 3D SLAM (georeferencing + loop closure) <solutions>`
    solution in MOLA. If you only need basic LIO mapping without loop closure, see the
    simpler :ref:`MOLA-LIO tutorial <tutorial-ouster-lio>` first.

|

Overview
--------

The pipeline consists of four steps:

1. **LIO mapping** — Run MOLA-LIO on a rosbag to produce a raw ``*.simplemap`` keyframe map.
2. **Loop closure** — Run ``mola-sm-lc-cli`` to detect and close loops, producing a corrected ``*.simplemap``.
3. **Metric map generation** — Run ``sm2mm`` to convert the loop-closed simplemap into a dense ``*.mm`` metric map.
4. **Georeferencing + visualization** — Optionally georeference the map (IMU gravity alignment) and inspect with ``mm-viewer``.

|

Prerequisites
-------------

You must have the following MOLA packages installed:

- ``mola_lidar_odometry`` (provides ``mola-lo-gui-rosbag2``, ``mm-viewer``)
- ``mola_sm_loop_closure`` (provides ``mola-sm-lc-cli``)
- ``mola_state_estimation`` (provides ``mola-sm-georeferencing``)
- ``mp2p_icp`` (provides ``sm2mm``)

Following the default :ref:`installation instructions <installing>` is enough for most of these.
For ``mola_sm_loop_closure``, as of today (April 2026), it must be installed from source code
(``apt install`` to be available in the short term).

Also make sure you have the ``mola_sm_loop_closure`` pipeline files available in your workspace:

.. code-block:: bash

    ls ~/ros2_ws/src/mola_sm_loop_closure/pipelines/

You should see files like ``loop-closure-f2f-lidar3d-gicp.yaml`` listed there.

|

Step 1: Run MOLA-LIO to generate the raw simplemap
----------------------------------------------------

Use ``mola-lo-gui-rosbag2`` to run the LIO pipeline on your rosbag.
The environment variables below configure the pipeline specifically for Ouster sensors.

.. code-block:: bash

    MOLA_LO_INITIAL_LOCALIZATION_METHOD=InitLocalization::PitchAndRollFromIMU \
    MOLA_SIMPLEMAP_GENERATE_LAZY_LOAD=1 \
    MOLA_GENERATE_SIMPLEMAP=true \
    MOLA_SIMPLEMAP_ALSO_NON_KEYFRAMES=true \
    MOLA_SIMPLEMAP_MIN_XYZ=0.15 \
    MOLA_LOCAL_MAP_MAX_SIZE=30.0 \
    MOLA_DESKEW_METHOD=MotionCompensationMethod::IMU \
    MOLA_TF_BASE_LINK=os_sensor \
    MOLA_LIDAR_TOPIC=/ouster/points \
    MOLA_IMU_TOPIC=/ouster/imu \
    mola-lo-gui-rosbag2   YOUR_DATASET.mcap

.. note::

    Replace ``YOUR_DATASET`` with the actual path to your ``.db3``
    or ``.mcap`` rosbag file.

**Key environment variables explained:**

.. TODO: Expand this table into a more detailed explanation with links to the pipeline
.. configuration documentation at https://docs.mola-slam.org/latest/mola_lo_pipelines.html
.. for each relevant variable.

- ``MOLA_LO_INITIAL_LOCALIZATION_METHOD=InitLocalization::PitchAndRollFromIMU``:
  Uses the Ouster IMU to determine initial pitch and roll (gravity alignment). This avoids
  needing a manual initial pose estimate and gives a physically consistent starting orientation.
- ``MOLA_SIMPLEMAP_GENERATE_LAZY_LOAD=1``: Stores raw sensor observations on disk
  (as external files) rather than embedding them in the simplemap. This keeps memory
  usage low for large datasets and posterior simplemap manipulation extremely fast.
- ``MOLA_GENERATE_SIMPLEMAP=true``: Instructs MOLA-LIO to save a keyframe map in
  ``.simplemap`` format alongside the metric map.
- ``MOLA_SIMPLEMAP_ALSO_NON_KEYFRAMES=true``: Also stores non-keyframe poses, which
  is useful only if one wants to export the full trajectory from the simplemap.
- ``MOLA_SIMPLEMAP_MIN_XYZ=0.15``: Minimum distance (in meters) between consecutive
  keyframes. Lower values produce denser maps.
- ``MOLA_LOCAL_MAP_MAX_SIZE=30.0``: Maximum radius of the local map kept in memory
  during odometry.
- ``MOLA_DESKEW_METHOD=MotionCompensationMethod::IMU``: Use IMU data for per-point
  motion compensation (deskewing), greatly improving accuracy for fast movements.
- ``MOLA_TF_BASE_LINK=os_sensor``: The TF frame name for the sensor base. Use
  ``os_sensor`` for Ouster sensors.
- ``MOLA_LIDAR_TOPIC`` / ``MOLA_IMU_TOPIC``: ROS 2 topic names for LiDAR and IMU data.
  Adjust if your Ouster driver uses different topic names.

.. dropdown:: What output files does this step produce?
   :icon: file-directory

   After the GUI process completes (or when you close the window):

   - ``final_map.simplemap``: The keyframe map with all poses and sensor observations.
   - ``final_map_Images/``: Directory containing the externally stored observation files
     (referenced from the simplemap via lazy-load).

|

Step 2: Loop closure post-processing
--------------------------------------

This step is the core of global consistency. The tool ``mola-sm-lc-cli`` reads the raw
simplemap, detects revisited places, aligns them via ICP, and runs a pose-graph optimization
to correct the accumulated drift.

.. code-block:: bash

    SAVE_3D_SCENES=true \
    SAVE_3D_SCENES_PER_ITER=true \
    MOLA_LC_SENSOR_FILTER_MIN_RANGE=0.5 \
    MOLA_LC_SENSOR_FILTER_MAX_RANGE=50.0 \
    LC_SELECTION_VERBOSE=true \
    MAX_LC_CANDIDATES=50 \
    MIN_FRAMES_BETWEEN_LC=30 \
    MAX_LC_OPTIMIZATION_ROUNDS=30 \
    LC_OPTIMIZE_EVERY_N=0 \
    MIN_ICP_QUALITY=0.75 \
    LC_ICP_INITIAL_SIGMA=1.0 \
    LC_ICP_FINAL_SIGMA=0.10 \
    LC_SELECTION_VERBOSE=1 \
    MIN_LC_DISTANCE=0.25 \
    MAX_LC_DISTANCE=2.5 \
    INPUT_ODOMETRY_NOISE_XYZ_PER_M=0.04 \
    INPUT_ODOMETRY_NOISE_ANG_DEG_PER_M=0.04 \
    MOLA_DESKEW_IGNORE_NO_TIMESTAMPS=false \
    MOLA_DESKEW_METHOD=MotionCompensationMethod::IMU \
    mola-sm-lc-cli \
        -a mola::FrameToFrameLoopClosure \
        -p ~/ros2_ws/src/mola_sm_loop_closure/pipelines/loop-closure-f2f-lidar3d-gicp.yaml \
        -i final_map.simplemap \
        -o final_map_lc.simplemap \
        --externals-dir final_map_Images/

.. note::

    The settings above are tuned for **centimeter-level accuracy**. They run up to
    ``MAX_LC_OPTIMIZATION_ROUNDS=30`` pose-graph optimization rounds, which can take
    significant time on large datasets. To speed things up at the cost of some accuracy,
    reduce ``MAX_LC_OPTIMIZATION_ROUNDS`` (e.g. to 5-10).

**Key parameters explained:**

- ``MOLA_LC_SENSOR_FILTER_MIN_RANGE`` / ``MAX_RANGE``: Points outside this range
  are excluded from loop closure ICP alignment. For Ouster sensors, 0.5-50 m is a
  good starting range for indoor/outdoor mixed environments.
- ``MAX_LC_CANDIDATES``: Maximum number of loop closure candidates to evaluate per keyframe.
- ``MIN_FRAMES_BETWEEN_LC``: Minimum number of keyframes that must separate two frames
  for them to be considered a loop closure candidate (avoids trivial "loops" between
  adjacent keyframes).
- ``MAX_LC_OPTIMIZATION_ROUNDS``: Number of pose-graph optimization rounds. More rounds
  generally improve accuracy. Set to 30 for highest quality; reduce to 5-10 for speed.
- ``LC_OPTIMIZE_EVERY_N=0``: Set to 0 to run optimization only at the end (faster).
  Set to a positive integer to optimize every N loop closures (more incremental but slower).
- ``MIN_ICP_QUALITY``: Loop closure candidates whose ICP alignment quality falls below
  this threshold are rejected. Higher values = fewer but more reliable loop closures.
- ``LC_ICP_INITIAL_SIGMA`` / ``LC_ICP_FINAL_SIGMA``: ICP uncertainty schedule (meters).
  The ICP starts with a coarser alignment (1.0 m sigma) and refines to 0.10 m.
- ``MIN_LC_DISTANCE`` / ``MAX_LC_DISTANCE``: Spatial distance range (meters) within which
  two poses are considered loop closure candidates.
- ``INPUT_ODOMETRY_NOISE_XYZ_PER_M`` / ``INPUT_ODOMETRY_NOISE_ANG_DEG_PER_M``: Noise
  model for the input odometry, used in the pose-graph. Tune these to match your
  LIO accuracy (0.04 is a conservative estimate for MOLA-LIO).
- ``SAVE_3D_SCENES=true``: Saves ``.3Dscene`` files for visualization of loop closure
  edges and the optimized path. These are used in the visualization step below.
- ``-a mola::FrameToFrameLoopClosure``: Specifies the loop closure algorithm class.
- ``-p``: Path to the ICP pipeline YAML file used for frame-to-frame alignment.
- ``--externals-dir``: Must point to the same externals directory used during LIO mapping.


|

Step 3: Generate the final metric map
---------------------------------------

Convert the loop-closed simplemap into a dense metric map (``*.mm``) using the ``sm2mm`` tool.
See the :ref:`sm2mm pipelines documentation <sm2mm_pipelines>` for full details on available pipelines.

.. code-block:: bash

    sm2mm \
        -i final_map_lc.simplemap \
        -o final_map_lc.mm \
        -p ~/ros2_ws/src/mp2p_icp/demos/sm2mm_no_decim_imu_mls.yaml \
        --externals-dir final_map_Images/

.. note::

    The pipeline file ``sm2mm_no_decim_imu_mls.yaml`` produces a **dense, MLS-filtered**
    point cloud with IMU-based motion compensation. It is available in the
    `mp2p_icp GitHub repository <https://github.com/MOLAorg/mp2p_icp/tree/develop/demos>`_
    under the ``demos/`` directory.

    See the :ref:`sm2mm pipelines page <sm2mm_pipelines>` for a full description of this
    pipeline and its alternatives (e.g. voxelized maps, static/dynamic separation).

.. dropdown:: Choosing a different sm2mm pipeline
   :icon: gear

   The choice of ``sm2mm`` pipeline determines the structure and density of your final map.
   Some useful alternatives from the ``mp2p_icp`` demos directory:

   - ``sm2mm_no_decim_imu_mls.yaml`` (**this tutorial**): Dense point cloud with MLS
     surface smoothing and IMU deskewing. Best for high-quality survey maps.
   - ``sm2mm_no_decim_imu_mls_keyframe_map.yaml``: Same but also creates a
     ``mola::KeyframePointCloudMap`` layer suitable for MOLA-LIO localization-only mode.
   - ``sm2mm_pointcloud_voxelize.yaml``: Voxelized (downsampled) point cloud.
     Much smaller file size, useful for navigation.


Step 4: Georeferencing
-----------------------

Even without a GNSS sensor, the ``mola-sm-georeferencing`` tool can use IMU data to align
the map against gravity, ensuring that the ``z`` axis is truly vertical. This improves
map consistency and is useful for any downstream application (visualization, localization, export).

.. code-block:: bash

    mola-sm-georeferencing \
        -i final_map_lc.simplemap \
        --write-into final_map_lc.mm

This command reads the IMU observations embedded in the simplemap, estimates the optimal
gravity-aligned orientation, and writes the georeferencing metadata directly into the
existing ``final_map_lc.mm`` file.

See the :ref:`Georeferencing documentation <geo-referencing>` for full details on how
georeferencing works and what additional outputs are available (e.g. KML trajectories,
geodetic coordinates for point cloud layers).

.. note::

    If you have a GNSS sensor (GPS), its data recorded in the simplemap will be
    automatically used for full geodetic georeferencing (latitude/longitude/altitude),
    not just gravity alignment. MOLA-LIO must be configured to capture GPS observations;
    see the :ref:`LIO pipeline sensor inputs <mola-lo-pipelines>` documentation.

|

Step 5: Visualize the result
-----------------------------

Use ``mm-viewer`` to inspect the final map. You can optionally overlay the loop closure
edge 3D scenes generated in step 2 to visually verify which loop closures were accepted:

.. code-block:: bash

    # Basic map view:
    mm-viewer final_map_lc.mm

    # With loop closure edges and trajectory overlay:
    mm-viewer final_map_lc.mm \
        -s final_map_lc_lc_edges.3Dscene \
        -s final_map_lc_path_edges.3Dscene

The ``-s`` flag loads additional ``.3Dscene`` files as overlays. The ``_lc_edges`` scene
shows the accepted loop closure constraints (edges between matched keyframe pairs), and
``_path_edges`` shows the optimized trajectory.

|

Troubleshooting
---------------

.. dropdown:: No loop closures found
   :icon: alert

   If the tool reports zero loop closure candidates:

   - Check that ``MIN_LC_DISTANCE`` is not too large or ``MAX_LC_DISTANCE`` too small:
     if the robot never returned close enough to a previous pose, no candidates will be found.
   - Check that ``MIN_FRAMES_BETWEEN_LC`` is not too large relative to your trajectory length.

.. dropdown:: ICP quality always too low (all candidates rejected)
   :icon: alert

   - Try reducing ``MIN_ICP_QUALITY`` to 0.5-0.6 as a first diagnostic step.
   - Verify that your LIO simplemap (step 1) is of reasonable quality before running LC.

|

Next steps
----------

After building your globally consistent map, you can:

- **Use it for localization**: Load ``final_map_lc.mm`` in MOLA-LIO localization-only mode.
  See :ref:`tutorial-mola-lo-map-and-localize` for step-by-step instructions.
- **Export to LAS/PLY**: Use ``mm2txt`` or ``mm2ply`` for GIS workflows or external processing.
- **Add geodetic coordinates**: Use ``mola-mm-add-geodetic`` to enrich the point cloud with
  latitude/longitude/altitude fields. See :ref:`geo-referencing`.

