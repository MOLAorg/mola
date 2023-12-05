/* -------------------------------------------------------------------------
 *   A Modular Optimization framework for Localization and mApping  (MOLA)
 * Copyright (C) 2018-2023 Jose Luis Blanco, University of Almeria
 * See LICENSE for license information.
 * ------------------------------------------------------------------------- */
/**
 * @file   KittiOdometryDataset.h
 * @brief  RawDataSource from Kitti odometry/SLAM datasets
 * @author Jose Luis Blanco Claraco
 * @date   Nov 29, 2018
 */
#pragma once

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
 * If the option `clouds_as_organized_points` is true (default), point cloud
 * are published as mrpt::obs::CObservationRotatingScan.
 * Otherwise, they are published as mrpt::obs::CObservationPointCloud.
 *
 * \ingroup mola_input_kitti_dataset_grp */
class KittiOdometryDataset : public RawDataSourceBase,
                             public OfflineDatasetSource
{
    DEFINE_MRPT_OBJECT(KittiOdometryDataset, mola)

   public:
    KittiOdometryDataset();
    ~KittiOdometryDataset() override = default;

    // See docs in base class
    void initialize(const Yaml& cfg) override;
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

    /** See:
     *  "IMLS-SLAM: scan-to-model matching based on 3D data", JE Deschaud, 2018.
     */
    double VERTICAL_ANGLE_OFFSET = mrpt::DEG2RAD(0.205);

   private:
    bool                    initialized_ = false;
    std::string             base_dir_;  //!< base dir for "sequences/*".
    std::string             sequence_;  //!< "00", "01", ...
    bool                    clouds_as_organized_points_ = true;
    unsigned int            range_matrix_column_count_  = 2000;
    unsigned int            range_matrix_row_count_     = 64;
    mrpt::Clock::time_point replay_begin_time_{};
    timestep_t              replay_next_tim_index_{0};
    bool                    replay_started_{false};
    bool                    publish_lidar_{true};
    bool                    publish_ground_truth_{true};
    std::array<bool, 4>     publish_image_{{true, true, true, true}};
    double                  time_warp_scale_{1.0};
    std::array<mrpt::img::TCamera, 4>  cam_intrinsics_;
    std::array<mrpt::math::TPose3D, 4> cam_poses_;  //!< wrt vehicle origin

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

    void load_img(const unsigned int cam_idx, const timestep_t step) const;
    void load_lidar(timestep_t step) const;
};

}  // namespace mola
