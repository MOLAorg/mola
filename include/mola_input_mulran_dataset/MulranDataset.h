/* -------------------------------------------------------------------------
 *   A Modular Optimization framework for Localization and mApping  (MOLA)
 * Copyright (C) 2018-2023 Jose Luis Blanco, University of Almeria
 * See LICENSE for license information.
 * ------------------------------------------------------------------------- */
/**
 * @file   MulranDataset.h
 * @brief  RawDataSource from Mulran datasets
 * @author Jose Luis Blanco Claraco
 * @date   Dec 12, 2023
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
/** RawDataSource from Mulran datasets.
 * Each "sequence" directory contains these sensor streams:
 * - `lidar`: Ouster OS1-64 LIDAR
 * - `radar`: Navtech CIR204-H (Not implemented yet).
 * - `gps`  : (TBD) (Not implemented yet).
 * - `imu`  : (TBD) (Not implemented yet).
 * - Ground truth poses
 *
 * If the option `clouds_as_organized_points` is true (default), point cloud
 * are published as mrpt::obs::CObservationRotatingScan.
 * Otherwise, they are published as mrpt::obs::CObservationPointCloud.
 *
 * Expected contents under `base_dir` directory:
 *
 * Example `base_dir`: `/mnt/storage/MulRan/`
 * Sequences: `KAIST01`, `KAIST02`, etc.
 * \code
 * MulRan/
 * ├── KAIST01
 * │   ├── data_stamp.csv
 * │   ├── global_pose.csv
 * │   ├── gps.csv
 * │   ├── navtech_top_stamp.csv
 * │   ├── Ouster
 * │   │   ├── 1561000444390857630.bin
 * │   │   ├── 1561000444489636410.bin
 * │   │   ...
 * │   ├── ouster_front_stamp.csv
 * │   └── xsens_imu.csv
 * ...
 * \endcode
 *
 * \ingroup mola_input_mulran_dataset_grp
 */
class MulranDataset : public RawDataSourceBase, public OfflineDatasetSource
{
    DEFINE_MRPT_OBJECT(MulranDataset, mola)

   public:
    MulranDataset();
    ~MulranDataset() override = default;

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
    bool                    clouds_as_organized_points_ = false;
    unsigned int            range_matrix_column_count_  = 2000;
    unsigned int            range_matrix_row_count_     = 64;
    mrpt::Clock::time_point replay_begin_time_{};
    timestep_t              replay_next_tim_index_{0};
    bool                    replay_started_{false};
    bool                    publish_lidar_{true};
    bool                    publish_ground_truth_{true};
    double                  time_warp_scale_{1.0};

    std::vector<std::string> lstPointCloudFiles_;

    mrpt::math::CMatrixDouble groundTruthPoses_;
    trajectory_t              groundTruthTrajectory_;
    mutable std::map<timestep_t, mrpt::obs::CObservation::Ptr>
        read_ahead_lidar_obs_;

    mrpt::poses::CPose3D ousterPoseOnVehicle_;

    std::vector<double> lst_timestamps_;  // for each lidar
    double              replay_time_ = .0;
    std::string         seq_dir_;

    void load_lidar(timestep_t step) const;
};

}  // namespace mola
