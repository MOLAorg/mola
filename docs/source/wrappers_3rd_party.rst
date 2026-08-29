.. _wrappers_3rd_party:

======================
3rd-party wrappers
======================
For the sake of scientific comparison between different methods, we provide wrappers of
other LiDAR odometry methods that mimic the interface of :ref:`mola-lidar-odometry-cli <mola_lidar_odometry_cli>`,
so exactly the same input datasets can be processed by different methods.

____________________________________________

|

KISS-ICP
--------------------------------------
Wrapper for the work :cite:`vizzo2023kiss`.

Repository: https://github.com/MOLAorg/mola_kiss_icp_wrapper

.. dropdown:: Compile instructions
   :icon: code-square

   Clone in your ROS 2 workspace:

   .. code-block:: bash

      mkdir -p ~/ros2_mola_ws/src/
      cd ~/ros2_mola_ws/src/

      git clone https://github.com/MOLAorg/mola_kiss_icp_wrapper.git --recursive

   Install dependencies:

   .. code-block:: bash

      cd ~/ros2_mola_ws/
      rosdep install --from-paths src --ignore-src -r -y

   Compile:

   .. code-block:: bash

      cd ~/ros2_mola_ws/
      colcon  build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo

.. dropdown:: CLI reference
   :icon: checklist

   .. code-block:: bash

      USAGE:

         mola-lidar-odometry-cli-kiss  [--input-mulran-seq <KAIST01>]
                                       [--kitti-correction-angle-deg <0.205
                                       [degrees]>] [--input-kitti-seq <00>]
                                       [--lidar-sensor-label <>] [--input-rosbag2
                                       <dataset.mcap>] [--input-rawlog
                                       <dataset.rawlog>] [--only-first-n <Number
                                       of dataset entries to run>] [--no-deskew]
                                       [--output-tum-path
                                       <output-trajectory.txt>] [--max-range
                                       <max-range>] [--min-range <min-range>]
                                       [--] [--version] [-h]


      Where: 

         --input-mulran-seq <KAIST01>
         INPUT DATASET: Use Mulran dataset sequence KAIST01|KAIST01|...

         --kitti-correction-angle-deg <0.205 [degrees]>
         Correction vertical angle offset (see Deschaud,2018)

         --input-kitti-seq <00>
         INPUT DATASET: Use KITTI dataset sequence number 00|01|...

         --lidar-sensor-label <>
         If provided, this supersedes the values in the 'lidar_sensor_labels'
         entry of the odometry pipeline, defining the sensorLabel/topic name to
         read LIDAR data from. It can be a regular expression (std::regex)

         --input-rosbag2 <dataset.mcap>
         INPUT DATASET: rosbag2. Input dataset in rosbag2 format (*.mcap)

         --input-rawlog <dataset.rawlog>
         INPUT DATASET: rawlog. Input dataset in rawlog format (*.rawlog)

         --only-first-n <Number of dataset entries to run>
         Run for the first N steps only (0=default, not used)

         --no-deskew
         Skip scan de-skew

         --output-tum-path <output-trajectory.txt>
         Save the estimated path as a TXT file using the TUM file format (see
         evo docs)

         --max-range <max-range>
         max-range parameter

         --min-range <min-range>
         min-range parameter

         --,  --ignore_rest
         Ignores the rest of the labeled arguments following this flag.

         --version
         Displays version information and exits.

         -h,  --help
         Displays usage information and exits.


         mola-lidar-odometry-cli-kiss


|


SiMpLE
--------------------------------------
Wrapper for the work :cite:`bhandari2024minimal`.

Repository: https://github.com/MOLAorg/mola_simple_wrapper

.. dropdown:: Compile instructions
   :icon: code-square

   Clone in your ROS 2 workspace:

   .. code-block:: bash

      mkdir -p ~/ros2_mola_ws/src/
      cd ~/ros2_mola_ws/src/

      git clone https://github.com/MOLAorg/mola_simple_wrapper.git --recursive

   Install dependencies:

   .. code-block:: bash

      cd ~/ros2_mola_ws/
      rosdep install --from-paths src --ignore-src -r -y

   Compile:

   .. code-block:: bash

      cd ~/ros2_mola_ws/
      colcon  build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo

.. dropdown:: CLI reference
   :icon: checklist

   .. code-block:: bash

      USAGE: 

         mola-lidar-odometry-cli-simple  [--input-paris-luco] [--input-mulran-seq
                                       <KAIST01>] [--input-kitti360-seq <00>]
                                       [--kitti-correction-angle-deg <0.205
                                       [degrees]>] [--input-kitti-seq <00>]
                                       [--lidar-sensor-label <lidar1>]
                                       [--input-rosbag2 <dataset.mcap>]
                                       [--input-rawlog <dataset.rawlog>]
                                       [--only-first-n <Number of dataset
                                       entries to run>] [--output-tum-path
                                       <output-trajectory.txt>] -c
                                       <config.yaml> [--] [--version] [-h]


      Where: 

         --input-paris-luco
         INPUT DATASET: Use Paris Luco dataset (unique sequence=00)

         --input-mulran-seq <KAIST01>
         INPUT DATASET: Use Mulran dataset sequence KAIST01|KAIST01|...

         --input-kitti360-seq <00>
         INPUT DATASET: Use KITTI360 dataset sequence number 00|01|...

         --kitti-correction-angle-deg <0.205 [degrees]>
         Correction vertical angle offset (see Deschaud,2018)

         --input-kitti-seq <00>
         INPUT DATASET: Use KITTI dataset sequence number 00|01|...

         --lidar-sensor-label <lidar1>
         If provided, this supersedes the values in the 'lidar_sensor_labels'
         entry of the odometry pipeline, defining the sensorLabel/topic name to
         read LIDAR data from. It can be a regular expression (std::regex)

         --input-rosbag2 <dataset.mcap>
         INPUT DATASET: rosbag2. Input dataset in rosbag2 format (*.mcap)

         --input-rawlog <dataset.rawlog>
         INPUT DATASET: rawlog. Input dataset in rawlog format (*.rawlog)

         --only-first-n <Number of dataset entries to run>
         Run for the first N steps only (0=default, not used)

         --output-tum-path <output-trajectory.txt>
         Save the estimated path as a TXT file using the TUM file format (see
         evo docs)

         -c <config.yaml>,  --config-file <config.yaml>
         (required)  Simple config file

         --,  --ignore_rest
         Ignores the rest of the labeled arguments following this flag.

         --version
         Displays version information and exits.

         -h,  --help
         Displays usage information and exits.


         mola-lidar-odometry-cli-simple


DLIO
--------------------------------------
Wrapper for the work :cite:`chen2023dlio`.

Repository: https://github.com/MOLAorg/mola_dlio_wrapper

Unlike the filter-based methods above, DLIO uses no filter and no factor
graph: the pose comes from GICP scan-to-submap registration, and the IMU is
fused by a nonlinear geometric observer with constant gains. Every point is
deskewed against a continuous IMU-integrated trajectory rather than a
per-scan linear interpolation, and the local map is a submap selected from
keyframes by a hull plus kNN search rather than a voxel or octree structure.

It provides two entry points:

- ``mola::DlioOdometry``, an online module loadable from a ``mola-cli`` launch
  YAML (``type: mola::DlioOdometry``), which publishes localization and submap
  updates so the MOLA GUI draws the trajectory and the growing map live.
- ``mola-dlio-cli``, an offline tool that waits for each observation to finish
  before feeding the next, so no scan is ever dropped. This is the one to use
  for comparisons: see :ref:`gui_vs_cli`.

.. note::
   This wrapper currently targets Oxford Spires, read through the generic
   ``mola::Rosbag2Dataset`` input. It does not cover the full set of dataset
   sources that the KISS-ICP and SiMpLE wrappers accept.

.. dropdown:: Compile instructions
   :icon: code-square

   Clone in your ROS 2 workspace:

   .. code-block:: bash

      mkdir -p ~/ros2_mola_ws/src/
      cd ~/ros2_mola_ws/src/

      git clone https://github.com/MOLAorg/mola_dlio_wrapper.git --recursive

   Install dependencies and compile:

   .. code-block:: bash

      cd ~/ros2_mola_ws/
      rosdep install --from-paths src --ignore-src -r -y
      colcon build --symlink-install --packages-select mola_dlio_wrapper \
        --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo

|

GLIM
--------------------------------------
Wrapper for the work :cite:`koide2024glim`.

Repository: https://github.com/MOLAorg/mola_glim_wrapper

Upstream GLIM is a full SLAM system: an odometry front end, a sub-mapping
stage, and a global-mapping back end with loop closure. **This wrapper
instantiates the odometry stage only** (``glim::OdometryEstimationCPU``, a
fixed-lag factor-graph estimator over GICP/VGICP scan-to-model factors and
preintegrated IMU factors).

That is deliberate. The wrapper exists for cross-method *odometry*
comparison, and a loop-closed, globally optimized trajectory is not
comparable with the odometry-only results the other wrappers on this page
produce.

It provides:

- ``mola::GlimOdometry``, an online module loadable from a ``mola-cli`` launch
  YAML.
- ``mola-glim-cli``, an offline, loss-free CLI over the MOLA dataset sources:
  KITTI, ROS 1 bags, ROS 2 bags, MulRan and rawlog.
- Pipeline YAMLs for Oxford Spires, KITTI, BotanicGarden and Newer College.

.. dropdown:: Compile instructions
   :icon: code-square

   Everything GLIM needs is vendored in the repository, so the only
   prerequisites are system packages: GTSAM >= 4.2 with ``gtsam_unstable``
   (``ros-$ROS_DISTRO-gtsam`` on ROS distributions), Eigen 3, Boost (graph,
   filesystem, serialization), spdlog, OpenMP, plus MRPT and the MOLA core
   packages.

   .. code-block:: bash

      mkdir -p ~/ros2_mola_ws/src/
      cd ~/ros2_mola_ws/src/

      git clone https://github.com/MOLAorg/mola_glim_wrapper.git --recursive

   .. code-block:: bash

      cd ~/ros2_mola_ws/
      rosdep install --from-paths src --ignore-src -r -y
      colcon build --symlink-install --packages-select mola_glim_wrapper \
        --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo

.. dropdown:: Example: run on a KITTI sequence
   :icon: terminal

   .. code-block:: bash

      KITTI_BASE_DIR=/path/to/kitti mola-glim-cli \
        -c $(ros2 pkg prefix mola_glim_wrapper)/share/mola_glim_wrapper/pipelines/glim-kitti.yaml \
        --input-kitti-seq 04 \
        --output-tum-path /tmp/glim_kitti04.tum
