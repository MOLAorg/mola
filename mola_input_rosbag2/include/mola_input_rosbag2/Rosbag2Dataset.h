/* -------------------------------------------------------------------------
 *   A Modular Optimization framework for Localization and mApping  (MOLA)
 * Copyright (C) 2018-2024 Jose Luis Blanco, University of Almeria
 * See LICENSE for license information.
 * ------------------------------------------------------------------------- */
/**
 * @file   Rosbag2Dataset.h
 * @brief  RawDataSource for datasets in rosbag2 format
 * @author Jose Luis Blanco Claraco
 * @date   Dec 23, 2023
 */
#pragma once

#include <mola_kernel/interfaces/OfflineDatasetSource.h>
#include <mola_kernel/interfaces/RawDataSourceBase.h>
#include <mrpt/io/CFileGZInputStream.h>
#include <mrpt/obs/CSensoryFrame.h>
#include <mrpt/serialization/CArchive.h>

#include <list>

// forward decls to isolate build dependencies downstream:
namespace tf2
{
class BufferCore;
}
namespace rosbag2_cpp::readers
{
class SequentialReader;
}
namespace rosbag2_storage
{
class SerializedBagMessage;
}

namespace mola
{
/** RawDataSource for datasets in rosbag2 format.
 *
 *  It reads a rosbag2 file, and exposes it as a dataset
 *  with N entries, N being the number of messages in the bag.
 *  Reading them via the offline API (OfflineDatasetSource)
 *  returns empty shared_ptr observations for those messages
 *  that do not have a direct mapping to mrpt::obs classes.
 *  The dataset can be also played in an online (real-time, or with a custom
 *  time wrapping) fashion via the RawDataSourceBase API.
 *
 *  See example configuration files to see how to define what topics
 *  to publish, and how to optionally override the sensor poses in the local
 *  robot frame.
 *
 * \ingroup mola_input_rosbag2_grp */
class Rosbag2Dataset : public RawDataSourceBase, public OfflineDatasetSource
{
    DEFINE_MRPT_OBJECT(Rosbag2Dataset, mola)

   public:
    Rosbag2Dataset();
    ~Rosbag2Dataset() override = default;

    // See docs in base class
    void initialize(const Yaml& cfg) override;
    void spinOnce() override;

    // See docs in base class:
    size_t datasetSize() const override;

    mrpt::obs::CSensoryFrame::Ptr datasetGetObservations(
        size_t timestep) const override;

   private:
    bool        initialized_ = false;
    std::string rosbag_filename_;
    std::string rosbag_storage_id_    = "mcap";
    std::string rosbag_serialization_ = "cdr";
    std::string base_link_frame_id_   = "base_footprint";

    mrpt::Clock::time_point                replay_begin_time_{};
    std::optional<mrpt::Clock::time_point> rawlog_begin_time_;
    bool                                   replay_started_    = false;
    double                                 time_warp_scale_   = 1.0;
    size_t                                 read_ahead_length_ = 15;

    std::shared_ptr<rosbag2_cpp::readers::SequentialReader> reader_;
    size_t bagMessageCount_ = 0;

    using SF = mrpt::obs::CSensoryFrame;

    SF::Ptr to_mrpt(const rosbag2_storage::SerializedBagMessage& rosmsg);

    void doReadAhead(
        const std::optional<size_t>& requestedIndex = std::nullopt);

    // timestep in this class is just the index of the message in the rosbag:
    struct DatasetEntry
    {
        SF::Ptr obs;

        /// empty if obs == nullptr
        std::optional<mrpt::Clock::time_point> timestamp;
    };

    /** At initialization
     *
     */
    std::vector<std::optional<DatasetEntry>> read_ahead_;
    size_t                                   rosbag_next_idx_       = 0;
    size_t                                   rosbag_next_idx_write_ = 0;
    std::set<std::string>                    already_pub_sensor_labels_;

    // Methods and variables for the ROS->MRPT conversion
    // -------------------------------------------------------
    using Obs = std::vector<mrpt::obs::CObservation::Ptr>;

    using CallbackFunction =
        std::function<Obs(const rosbag2_storage::SerializedBagMessage&)>;

    std::map<std::string, std::vector<CallbackFunction>> lookup_;
    std::set<std::string>                                unhandledTopics_;

    std::shared_ptr<tf2::BufferCore> tfBuffer_;

    template <bool isStatic>
    Obs toTf(const rosbag2_storage::SerializedBagMessage& rosmsg);

    Obs toPointCloud2(
        std::string_view                             label,
        const rosbag2_storage::SerializedBagMessage& rosmsg,
        const std::optional<mrpt::poses::CPose3D>&   fixedSensorPose);

    Obs toLidar2D(
        std::string_view                             msg,
        const rosbag2_storage::SerializedBagMessage& rosmsg,
        const std::optional<mrpt::poses::CPose3D>&   fixedSensorPose);

    Obs toRotatingScan(
        std::string_view                             msg,
        const rosbag2_storage::SerializedBagMessage& rosmsg,
        const std::optional<mrpt::poses::CPose3D>&   fixedSensorPose);

    Obs toIMU(
        std::string_view                             msg,
        const rosbag2_storage::SerializedBagMessage& rosmsg,
        const std::optional<mrpt::poses::CPose3D>&   fixedSensorPose);

    Obs toOdometry(
        std::string_view                             msg,
        const rosbag2_storage::SerializedBagMessage& rosmsg);

    Obs toImage(
        std::string_view                             msg,
        const rosbag2_storage::SerializedBagMessage& rosmsg,
        const std::optional<mrpt::poses::CPose3D>&   fixedSensorPose);

    bool findOutSensorPose(
        mrpt::poses::CPose3D& des, const std::string& target_frame,
        const std::string&                         source_frame,
        const std::optional<mrpt::poses::CPose3D>& fixedSensorPose);
};

}  // namespace mola
