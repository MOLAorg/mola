/* -------------------------------------------------------------------------
 *   A Modular Optimization framework for Localization and mApping  (MOLA)
 * Copyright (C) 2018-2024 Jose Luis Blanco, University of Almeria
 * See LICENSE for license information.
 * ------------------------------------------------------------------------- */
/**
 * @file   Kitti360Dataset.h
 * @brief  RawDataSource from Kitti-360 dataset
 * @author Jose Luis Blanco Claraco
 * @date   Feb 23, 2024
 */
#pragma once

#include <mola_kernel/interfaces/Dataset_UI.h>
#include <mola_kernel/interfaces/OfflineDatasetSource.h>
#include <mola_kernel/interfaces/RawDataSourceBase.h>
#include <mrpt/core/Clock.h>
#include <mrpt/img/TCamera.h>
#include <mrpt/math/TPose3D.h>
#include <mrpt/obs/obs_frwds.h>

#include <array>

// fwrd decls:
namespace mrpt::obs
{
class CObservationPointCloud;
}

namespace mola
{
/** RawDataSource from [Kitti-360
 * datasets](https://www.cvlibs.net/datasets/kitti-360/).
 *
 * Each "sequence" directory contains these sensor streams:
 * - `image_0` & `image_1`: Perspective cameras
 * - `image_2` & `image_3`: Fish-eye cameras (TO-DO)
 * - `lidar`: Velodyne 3D LIDAR. Published as mrpt::obs::CObservationPointCloud
 *    with X,Y,Z,I,T channels. The timestamp (T) channel is estimated from
 *    azimuth of points (if `generate_lidar_timestamps` param is `true`), with
 *    generated timestamps between [-0.05,+0.05] seconds.
 * - Ground truth poses
 *
 * The sequence to load is determined by the `sequence` parameter (e.g. via
 * config yaml file), from these possible values:
 *
 * \code
 * sequence: 00|02|03|04|05|06|07|08|09|10|18|test_0|test_1|test_2|test_3
 * \endcode
 *
 *
 * Expected contents under `base_dir` directory (the result of using the
 * download scripts provided by the KITTI-360 web):
 *
 * \code
 * KITTI-360/
 * ├── calibration
 * │  ├── calib_cam_to_pose.txt
 * │  ├── calib_cam_to_velo.txt
 * │  ├── calib_sick_to_velo.txt
 * │  ├── image_02.yaml
 * │  ├── image_03.yaml
 * │  └── perspective.txt
 * │
 * ├── data_3d_raw
 * │   ├ 2013_05_28_drive_00xx_sync
 * │   │ ├── cam0_to_world.txt
 * │   │ ├── poses.txt
 * │   │ └── velodyne_points
 * │   │     ├── timestamps.txt
 * │   │     └── data
 * │   │         ├── 0000000000.bin
 * │   │         ├── 0000000001.bin
 * ... ...           ...
 * ├── data_2d_raw
 * │   ├── 2013_05_28_drive_00xx_sync
 * │   │   ├── image_00
 * │   │   │   ├── data_rect
 * │   │   │   │   ├ 0000000000.png
 * │   │   │   │   ├ 0000000001.png
 * │   │   │   │   ...
 * │   │   │   └── timestamps.txt
 * │   │   └── image_01
 * │   │       ├── data_rect
 * │   │       │   ├ 0000000000.png
 * │   │       │   ├ 0000000001.png
 * │   │       │   ...
 * │   │       └── timestamps.txt
 * ... ...
 * ├── data_poses/
 * │   ├── 2013_05_28_drive_00xx_sync
 * │   │   ├── cam0_to_world.txt
 * │   │   └── poses.txt
 * ... ...
 * └──  data_3d_test_slam/
 *     ├── test_0
 *     │   └── 2013_05_28_drive_0008_sync
 *     │       └── velodyne_points
 *     │           ├── data
 *     │           └── timestamps.txt
 *     ├── test_1
 *     │   └── 2013_05_28_drive_0008_sync
 *     │       └── velodyne_points
 *     │           ├── data
 *     │           └── timestamps.txt
 *     ├── test_2
 *     │   └── 2013_05_28_drive_0004_sync
 *     │       └── velodyne_points
 *     │           ├── data
 *     │           └── timestamps.txt
 *     └── test_3
 *         └── 2013_05_28_drive_0002_sync
 *             └── velodyne_points
 *                 ├── data
 *                 └── timestamps.txt
 * \endcode
 *
 * - Example `base_dir`: `/mnt/storage/KITTI-360/` (normally read from
 *   environment variable KITTI360_DATASET in mola-cli launch files).
 * - Sequences: `01`, `02`, etc.
 *
 * \ingroup mola_input_kitti360_dataset_grp
 */
class Kitti360Dataset : public RawDataSourceBase,
                        public OfflineDatasetSource,
                        public Dataset_UI
{
    DEFINE_MRPT_OBJECT(Kitti360Dataset, mola)

   public:
    Kitti360Dataset();
    ~Kitti360Dataset() override = default;

    // See docs in base class
    void spinOnce() override;
    bool hasGroundTruthTrajectory() const override
    {
        return !groundTruthTrajectory_.empty();
    }
    trajectory_t getGroundTruthTrajectory() const override
    {
        return groundTruthTrajectory_;
    }

    /** Direct programmatic access to dataset observations.
     * The type of the lidar observation can be either:
     * - mrpt::obs::CObservationPointCloud (`clouds_as_organized_points_`=false)
     * - mrpt::obs::CObservationRotatingScan
     *   (`clouds_as_organized_points_`=true)
     */
    std::shared_ptr<mrpt::obs::CObservation> getPointCloud(
        timestep_t step) const;
    std::shared_ptr<mrpt::obs::CObservationImage> getImage(
        const unsigned int cam_idx, timestep_t step) const;

    // See docs in base class:
    size_t datasetSize() const override;

    mrpt::obs::CSensoryFrame::Ptr datasetGetObservations(
        size_t timestep) const override;

    // Virtual interface of Dataset_UI (see docs in derived class)
    size_t datasetUI_size() const override { return datasetSize(); }
    size_t datasetUI_lastQueriedTimestep() const override
    {
        auto lck = mrpt::lockHelper(dataset_ui_mtx_);
        return last_used_tim_index_;
    }
    double datasetUI_playback_speed() const override
    {
        auto lck = mrpt::lockHelper(dataset_ui_mtx_);
        return time_warp_scale_;
    }
    void datasetUI_playback_speed(double speed) override
    {
        auto lck         = mrpt::lockHelper(dataset_ui_mtx_);
        time_warp_scale_ = speed;
    }
    bool datasetUI_paused() const override
    {
        auto lck = mrpt::lockHelper(dataset_ui_mtx_);
        return paused_;
    }
    void datasetUI_paused(bool paused) override
    {
        auto lck = mrpt::lockHelper(dataset_ui_mtx_);
        paused_  = paused;
    }
    void datasetUI_teleport(size_t timestep) override
    {
        auto lck       = mrpt::lockHelper(dataset_ui_mtx_);
        teleport_here_ = timestep;
    }

    /** See:
     *  "IMLS-SLAM: scan-to-model matching based on 3D data", JE Deschaud, 2018.
     */
    double VERTICAL_ANGLE_OFFSET = mrpt::DEG2RAD(0.205);

   protected:
    // See docs in base class
    void initialize_rds(const Yaml& cfg) override;

   private:
    bool                initialized_ = false;
    std::string         base_dir_;  //!< base dir for "sequences/*".
    std::string         sequence_;  //!< "00", "01", ...
    timestep_t          replay_next_tim_index_{0};
    bool                publish_lidar_{true};
    bool                generate_lidar_timestamps_{true};
    bool                publish_ground_truth_{true};
    std::array<bool, 4> publish_image_{{true, true, true, true}};
    std::array<mrpt::img::TCamera, 4>   cam_intrinsics_;
    std::array<mrpt::poses::CPose3D, 4> cam_poses_;  //!< wrt vehicle origin
    mrpt::poses::CPose3D                velodyne_pose_;  //!< wrt vehicle origin

    std::optional<mrpt::Clock::time_point> last_play_wallclock_time_;
    double                                 last_dataset_time_ = 0;

    std::array<std::vector<std::string>, 4> lst_image_;
    std::array<std::string, 4>              lst_image_basedir_;

    std::vector<std::string> lst_velodyne_;
    std::string              lst_velodyne_basedir_;

    trajectory_t groundTruthTrajectory_;
    mutable std::map<timestep_t, mrpt::obs::CObservation::Ptr>
        read_ahead_lidar_obs_;
    mutable std::map<timestep_t, std::array<mrpt::obs::CObservation::Ptr, 4>>
        read_ahead_image_obs_;

    std::vector<double> lstLidarTimestamps_;
    double              replay_time_{.0};

    mutable timestep_t    last_used_tim_index_ = 0;
    bool                  paused_              = false;
    double                time_warp_scale_     = 1.0;
    std::optional<size_t> teleport_here_;
    mutable std::mutex    dataset_ui_mtx_;

    void load_img(const unsigned int cam_idx, const timestep_t step) const;
    void load_lidar(timestep_t step) const;

    void autoUnloadOldEntries() const;
};

}  // namespace mola
