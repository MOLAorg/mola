/* -------------------------------------------------------------------------
 *   A Modular Optimization framework for Localization and mApping  (MOLA)
 * Copyright (C) 2018-2024 Jose Luis Blanco, University of Almeria
 * See LICENSE for license information.
 * ------------------------------------------------------------------------- */
/**
 * @file   KittiOdometryDataset.h
 * @brief  RawDataSource from Kitti odometry/SLAM datasets
 * @author Jose Luis Blanco Claraco
 * @date   Nov 29, 2018
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
/** RawDataSource from Kitti odometry/SLAM datasets.
 * Each "sequence" directory contains these sensor streams:
 * - `image_0` & `image_1`: A grayscale stereo camera pair
 * - `image_2` & `image_3`: An RGB stereo camera pair
 * - `lidar`: Velodyne 3D LIDAR
 * - Ground truth poses
 *
 * If the option `clouds_as_organized_points` is true, point cloud
 * are published as mrpt::obs::CObservationRotatingScan.
 * Otherwise (default), they are published as mrpt::obs::CObservationPointCloud
 * with X,Y,Z,I channels.
 *
 * \ingroup mola_input_kitti_dataset_grp */
class KittiOdometryDataset : public RawDataSourceBase,
                             public OfflineDatasetSource,
                             public Dataset_UI
{
    DEFINE_MRPT_OBJECT(KittiOdometryDataset, mola)

   public:
    KittiOdometryDataset();
    ~KittiOdometryDataset() override = default;

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
    bool                clouds_as_organized_points_ = false;
    unsigned int        range_matrix_column_count_  = 2000;
    unsigned int        range_matrix_row_count_     = 64;
    timestep_t          replay_next_tim_index_{0};
    bool                publish_lidar_{true};
    bool                publish_ground_truth_{true};
    std::array<bool, 4> publish_image_{{true, true, true, true}};
    std::array<mrpt::img::TCamera, 4>  cam_intrinsics_;
    std::array<mrpt::math::TPose3D, 4> cam_poses_;  //!< wrt vehicle origin

    std::optional<mrpt::Clock::time_point> last_play_wallclock_time_;
    double                                 last_dataset_time_ = 0;

    std::array<std::vector<std::string>, 4> lst_image_;
    std::vector<std::string>                lst_velodyne_;
    mrpt::math::CMatrixDouble               groundTruthPoses_;
    trajectory_t                            groundTruthTrajectory_;
    mutable std::map<timestep_t, mrpt::obs::CObservation::Ptr>
        read_ahead_lidar_obs_;
    mutable std::map<timestep_t, std::array<mrpt::obs::CObservation::Ptr, 4>>
        read_ahead_image_obs_;

    std::vector<double> lst_timestamps_;
    double              replay_time_{.0};
    std::string         seq_dir_;

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
