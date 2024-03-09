/* -------------------------------------------------------------------------
 *   A Modular Optimization framework for Localization and mApping  (MOLA)
 * Copyright (C) 2018-2024 Jose Luis Blanco, University of Almeria
 * See LICENSE for license information.
 * ------------------------------------------------------------------------- */
/**
 * @file   MulranDataset.cpp
 * @brief  RawDataSource from Mulran datasets
 * @author Jose Luis Blanco Claraco
 * @date   Dec 12, 2023
 */

/** \defgroup mola_input_mulran_dataset_grp mola-input-mulran-dataset.
 * RawDataSource from Mulran datasets.
 *
 *
 */

#include <mola_input_mulran_dataset/MulranDataset.h>
#include <mola_yaml/yaml_helpers.h>
#include <mrpt/containers/yaml.h>
#include <mrpt/core/initializer.h>
#include <mrpt/maps/CPointsMapXYZI.h>
#include <mrpt/maps/CPointsMapXYZIRT.h>
#include <mrpt/obs/CObservationIMU.h>
#include <mrpt/obs/CObservationPointCloud.h>
#include <mrpt/obs/CObservationRobotPose.h>
#include <mrpt/obs/CObservationRotatingScan.h>
#include <mrpt/obs/gnss_messages_ascii_nmea.h>
#include <mrpt/system/CDirectoryExplorer.h>
#include <mrpt/system/filesystem.h>  //ASSERT_DIRECTORY_EXISTS_()

#include <Eigen/Dense>

using namespace mola;

// arguments: class_name, parent_class, class namespace
IMPLEMENTS_MRPT_OBJECT(MulranDataset, RawDataSourceBase, mola)

MRPT_INITIALIZER(do_register_MulranDataset)
{
    MOLA_REGISTER_MODULE(MulranDataset);
}

MulranDataset::MulranDataset() = default;

namespace
{
void build_list_files(
    const std::string& dir, const std::string& file_extension,
    std::vector<std::string>& out_lst)
{
    out_lst.clear();
    if (!mrpt::system::directoryExists(dir)) return;

    using direxpl = mrpt::system::CDirectoryExplorer;
    direxpl::TFileInfoList lstFiles;
    direxpl::explore(dir, FILE_ATTRIB_ARCHIVE, lstFiles);
    direxpl::sortByName(lstFiles);
    direxpl::filterByExtension(lstFiles, file_extension);
    out_lst.resize(lstFiles.size());
    std::transform(
        lstFiles.begin(), lstFiles.end(), out_lst.begin(),
        [](auto& fil) { return fil.name; });
}

}  // namespace

void MulranDataset::initialize_rds(const Yaml& c)
{
    using namespace std::string_literals;

    MRPT_START

    setLoggerName("MulranDataset");

    ProfilerEntry tle(profiler_, "initialize");

    MRPT_LOG_DEBUG_STREAM("Initializing with these params:\n" << c);

    // Mandatory parameters:
    ENSURE_YAML_ENTRY_EXISTS(c, "params");
    auto cfg = c["params"];

    YAML_LOAD_MEMBER_REQ(base_dir, std::string);
    YAML_LOAD_MEMBER_REQ(sequence, std::string);
    YAML_LOAD_MEMBER_OPT(lidar_to_ground_truth_1to1, bool);

    seq_dir_ = mrpt::system::pathJoin({base_dir_, sequence_});
    ASSERT_DIRECTORY_EXISTS_(seq_dir_);

    // Optional params with default values:
    YAML_LOAD_MEMBER_OPT(time_warp_scale, double);
    paused_ = cfg.getOrDefault<bool>("start_paused", paused_);

    YAML_LOAD_MEMBER_OPT(publish_lidar, bool);
    YAML_LOAD_MEMBER_OPT(publish_gps, bool);
    YAML_LOAD_MEMBER_OPT(publish_ground_truth, bool);

    // Make list of all existing files and preload everything we may need later
    // to quickly replay the dataset in realtime:
    MRPT_LOG_INFO_STREAM("Loading dataset from: " << seq_dir_);

    // make a list of files and sort them.
    build_list_files(
        mrpt::system::pathJoin({seq_dir_, "Ouster"}), "bin",
        lstPointCloudFiles_);
    ASSERT_(!lstPointCloudFiles_.empty());

    // Remove the last one, since it seems that the last scan is not cleanly
    // read and it's only half scan:
    lstPointCloudFiles_.resize(lstPointCloudFiles_.size() - 1);

    MRPT_LOG_INFO_STREAM("Ouster pointclouds: " << lstPointCloudFiles_.size());

    // Load sensors calibration:
    const double T_lidar_to_base_data[4 * 4] = {
        -9.9998295e-01, -5.8398386e-03, -5.2257060e-06, 1.7042000e00,  //
        5.8398386e-03,  -9.9998295e-01, 1.7758769e-06,  -2.1000000e-02,  //
        -5.2359878e-06, 1.7453292e-06,  1.0000000e00,   1.8047000e00,  //
        0.0000000e00,   0.0000000e00,   0.0000000e00,   1.0000000e00  //
    };

    ousterPoseOnVehicle_ =
        mrpt::poses::CPose3D() -
        mrpt::poses::CPose3D::FromHomogeneousMatrix(
            mrpt::math::CMatrixDouble44(T_lidar_to_base_data));

    MRPT_LOG_DEBUG_STREAM("ousterPoseOnVehicle = " << ousterPoseOnVehicle_);

    // Read GPS, if found:
    const auto gpsFile = mrpt::system::pathJoin({seq_dir_, "gps.csv"});
    if (mrpt::system::fileExists(gpsFile))
    {
        try
        {
            gpsCsvData_.loadFromTextFile(gpsFile);
            ASSERT_EQUAL_(gpsCsvData_.cols(), 13);
        }
        catch (const std::exception& e)
        {
            MRPT_LOG_ERROR_STREAM(
                "Error parsing GPS file: '" << gpsFile << "':\n"
                                            << e.what());
            throw;
        }

        MRPT_LOG_INFO_STREAM(
            "GPS found: " << gpsCsvData_.rows() << " entries.");

        // Parse into the unified container:
        for (int row = 0; row < gpsCsvData_.rows(); row++)
        {
            const double t     = 1e-9 * gpsCsvData_(row, 0);
            Entry        entry = {EntryType::GNNS};
            entry.gpsIdx       = row;
            datasetEntries_.emplace(t, entry);
        }
    }
    else
    {
        MRPT_LOG_INFO_FMT("No GPS file found, expected: '%s'", gpsFile.c_str());
    }

    // Load ground truth poses, if available:
    const auto gtFile = mrpt::system::pathJoin({seq_dir_, "global_pose.csv"});
    if (!mrpt::system::fileExists(gtFile))
    {
        MRPT_LOG_WARN_STREAM(
            "No ground truth file was found, expected it under: " << gtFile);
    }
    else
    {
        // Yes, we have GT:
        mrpt::math::CMatrixDouble gtMatrix;

        gtMatrix.loadFromTextFile(gtFile);
        ASSERT_EQUAL_(gtMatrix.cols(), 13U);
        mrpt::math::CMatrixDouble44 m = mrpt::math::CMatrixDouble44::Identity();

        // 1st) build a trajectory with the GT poses:
        trajectory_t gtPoses;

        for (int i = 0; i < gtMatrix.rows(); i++)
        {
            const double t = 1e-9 * gtMatrix(i, 0);

            for (int row = 0, ij_idx = 1; row < 3; row++)
                for (int col = 0; col < 4; col++, ij_idx++)
                    m(row, col) = gtMatrix(i, ij_idx);

            const auto gtPose = mrpt::poses::CPose3D::FromHomogeneousMatrix(m);

            gtPoses.insert(mrpt::Clock::fromDouble(t), gtPose);
        }

        if (lidar_to_ground_truth_1to1_)
        {
            // 2nd) Find matches between gt poses and the lidar scans, so we
            // have the GT for each scan:
            gtPoses.setInterpolationMethod(
                mrpt::poses::TInterpolatorMethod::imLinearSlerp);
            gtPoses.setMaxTimeInterpolation(std::chrono::seconds(1));

            std::vector<timestep_t> lidarIdxsToRemove;

            for (size_t i = 0; i < lstPointCloudFiles_.size(); i++)
            {
                const double t =
                    LidarFileNameToTimestamp(lstPointCloudFiles_[i]);
                const auto ts = mrpt::Clock::fromDouble(t);

                mrpt::poses::CPose3D p;
                bool                 interpOk = false;
                gtPoses.interpolate(ts, p, interpOk);

                if (!interpOk)
                {
                    lidarIdxsToRemove.push_back(i);
                    continue;
                }

                groundTruthTrajectory_.insert(ts, p);

                Entry entry = {EntryType::GroundTruth};
                entry.gtIdx = groundTruthTrajectory_.size() - 1;
                datasetEntries_.emplace(t, entry);
            }

            // remove in reverse order!
            for (auto it = lidarIdxsToRemove.rbegin();
                 it != lidarIdxsToRemove.rend(); ++it)
            {
                auto idx = *it;

                lstPointCloudFiles_.erase(
                    std::next(lstPointCloudFiles_.begin(), idx));
            }

            MRPT_LOG_INFO_FMT(
                "LIDAR timestamps: %zu, matched ground truth timestamps: %zu, "
                "from overall GT poses: %zu, removed %zu unmatched lidar "
                "scans.",
                lstPointCloudFiles_.size(), groundTruthTrajectory_.size(),
                gtPoses.size(), lidarIdxsToRemove.size());
        }
        else
        {
            // just keep lidars and GT vectors are they are originally.
        }
    }

    for (size_t i = 0; i < lstPointCloudFiles_.size(); i++)
    {
        // nanoseconds -> seconds
        const double t     = LidarFileNameToTimestamp(lstPointCloudFiles_[i]);
        const Entry  entry = {EntryType::Lidar, i};
        datasetEntries_.emplace(t, entry);
    }

    replay_next_it_ = datasetEntries_.begin();
    initialized_    = true;

    MRPT_END
}  // end initialize()

void MulranDataset::spinOnce()
{
    MRPT_START

    ASSERT_(initialized_);

    ProfilerEntry tleg(profiler_, "spinOnce");

    const auto tNow = mrpt::Clock::now();

    // Starting time:
    if (!last_play_wallclock_time_) last_play_wallclock_time_ = tNow;

    // get current replay time:
    auto         lckUIVars       = mrpt::lockHelper(dataset_ui_mtx_);
    const double time_warp_scale = time_warp_scale_;
    const bool   paused          = paused_;
    const auto   teleport_here   = teleport_here_;
    teleport_here_.reset();
    lckUIVars.unlock();

    double dt = mrpt::system::timeDifference(*last_play_wallclock_time_, tNow) *
                time_warp_scale;
    last_play_wallclock_time_ = tNow;

    const double t0 = datasetEntries_.begin()->first;

    // override by an special teleport order?
    if (teleport_here.has_value() && *teleport_here < datasetEntries_.size())
    {
        auto it = datasetEntries_.begin();
        std::advance(it, *teleport_here);

        replay_next_it_    = it;
        last_dataset_time_ = it->first - t0;
    }
    else
    {
        if (paused) return;
        // move forward replayed dataset time:
        last_dataset_time_ += dt;
    }

    if (replay_next_it_ == datasetEntries_.end())
    {
        onDatasetPlaybackEnds();  // notify base class

        MRPT_LOG_THROTTLE_INFO(
            10.0,
            "End of dataset reached! Nothing else to publish (CTRL+C to quit)");
        return;
    }
    else if (!datasetEntries_.empty())
    {
        const auto pos =
            std::distance(datasetEntries_.begin(), replay_next_it_);

        MRPT_LOG_THROTTLE_INFO_FMT(
            5.0, "Dataset replay progress: %lu / %lu  (%4.02f%%)",
            static_cast<unsigned long>(pos),
            static_cast<unsigned long>(datasetEntries_.size()),
            (100.0 * pos) / (datasetEntries_.size()));
    }

    std::optional<timestep_t> lastUsedLidarIdx;

    // We have to publish all observations until "t":
    while (replay_next_it_ != datasetEntries_.end() &&
           last_dataset_time_ >= (replay_next_it_->first - t0))
    {
        MRPT_LOG_DEBUG_STREAM(
            "Sending observations for replay time: "
            << mrpt::system::formatTimeInterval(last_dataset_time_));

        const auto& de = replay_next_it_->second;

        switch (de.type)
        {
            case EntryType::Lidar:
            {
                if (!publish_lidar_) break;

                lastUsedLidarIdx = de.lidarIdx;

                ProfilerEntry tle(profiler_, "spinOnce.publishLidar");
                load_lidar(de.lidarIdx);
                auto o = read_ahead_lidar_obs_.at(de.lidarIdx);
                this->sendObservationsToFrontEnds(o);

                // Free memory in read-ahead buffers:
                read_ahead_lidar_obs_.erase(de.lidarIdx);
            }
            break;

            case EntryType::GNNS:
            {
                if (!publish_gps_) break;

                auto o = get_gps_by_row_index(de.gpsIdx);
                this->sendObservationsToFrontEnds(o);
            }
            break;

            case EntryType::GroundTruth:
            {
                if (!publish_ground_truth_) break;

                // Get GT pose: it's already stored and correctly transformed
                // into groundTruthTrajectory_:
                auto it = groundTruthTrajectory_.begin();
                std::advance(it, de.gtIdx);

                // Publish as robot pose observation:
                auto o         = mrpt::obs::CObservationRobotPose::Create();
                o->sensorLabel = "ground_truth";
                o->pose.mean   = mrpt::poses::CPose3D(it->second);
                // o->pose.cov? don't use
                o->timestamp = it->first;

                this->sendObservationsToFrontEnds(o);
            }
            break;

            default:
                THROW_EXCEPTION("Unhandled dataset entry type (!?)");
        };

        // move on:
        replay_next_it_++;
    }

    {
        auto lck = mrpt::lockHelper(dataset_ui_mtx_);
        last_used_tim_index_ =
            std::distance(datasetEntries_.begin(), replay_next_it_);
    }

    // Read ahead to save delays in the next iteration:
    if (lastUsedLidarIdx)
    {
        const auto nextLidarIdx = *lastUsedLidarIdx + 1;

        if (nextLidarIdx < lstPointCloudFiles_.size())
        {
            ProfilerEntry tle(profiler_, "spinOnce.read_ahead");
            if (0 == read_ahead_lidar_obs_.count(nextLidarIdx))
                load_lidar(nextLidarIdx);
        }
    }

    MRPT_END
}

void MulranDataset::load_lidar(timestep_t step) const
{
    MRPT_START

    // unload() very old observations.
    autoUnloadOldEntries();

    // Already loaded?
    if (read_ahead_lidar_obs_[step]) return;

    ProfilerEntry tleg(profiler_, "load_lidar");

    // Load velodyne pointcloud:
    const auto f =
        mrpt::system::pathJoin({seq_dir_, "Ouster", lstPointCloudFiles_[step]});

    auto obs         = mrpt::obs::CObservationPointCloud::Create();
    obs->sensorLabel = "lidar";

    auto pts        = mrpt::maps::CPointsMapXYZIRT::Create();
    obs->pointcloud = pts;

    // Load XYZI from kitti-like file:
    mrpt::maps::CPointsMapXYZI kittiData;

    bool loadOk = kittiData.loadFromKittiVelodyneFile(f);
    ASSERTMSG_(
        loadOk, mrpt::format("Error loading kitti scan file: '%s'", f.c_str()));

    // Copy XYZI:
    *pts = kittiData;

    const size_t nPts = pts->size();
    ASSERT_EQUAL_(nPts, 1024 * 64);
    pts->resize_XYZIRT(nPts, true /*i*/, true /*R*/, true /*t*/);

    // Fixed to 10 Hz rotation in this dataset:
    const double sweepDuration = 0.1;  //  [s]
    const double At            = -0.5 * sweepDuration;

    for (size_t i = 0; i < nPts; i++)
    {
        const int row = i % 64;
        const int col = i / 64;
        pts->setPointTime(i, At + sweepDuration * col / 1024.0);
        pts->setPointRing(i, row);
    }

    // Pose:
    obs->sensorPose = ousterPoseOnVehicle_;
    obs->timestamp  = mrpt::Clock::fromDouble(
        LidarFileNameToTimestamp(lstPointCloudFiles_[step]));

#if 0  // Export clouds to txt for debugging externally (e.g. python, matlab)
    pts->saveXYZIRT_to_text_file(
        mrpt::format("mulran_%s_%06zu.txt", sequence_.c_str(), step));
#endif

#if 0
    {
        auto rs         = mrpt::obs::CObservationRotatingScan::Create();
        rs->sensorPose  = obs->sensorPose;
        rs->sensorLabel = obs->sensorLabel;
        rs->timestamp   = obs->timestamp;

        rs->sweepDuration = 0.10;  // [sec]
        rs->lidarModel    = "OS1-64";
        rs->minRange      = 0.1;
        rs->maxRange      = 120.0;

        rs->columnCount     = 1024;
        rs->rowCount        = 64;
        rs->rangeResolution = 1e-2;  // 1 cm

        rs->organizedPoints.resize(rs->rowCount, rs->columnCount);
        rs->intensityImage.resize(rs->rowCount, rs->columnCount);
        rs->rangeImage.resize(rs->rowCount, rs->columnCount);

        auto ptsXYZI = std::dynamic_pointer_cast<mrpt::maps::CPointsMapXYZI>(
            obs->pointcloud);
        ASSERT_(ptsXYZI);

        const auto& xs = ptsXYZI->getPointsBufferRef_x();
        const auto& ys = ptsXYZI->getPointsBufferRef_y();
        const auto& zs = ptsXYZI->getPointsBufferRef_z();

        const size_t nPts = xs.size();

        ASSERT_EQUAL_(nPts, 1024 * 64);

        for (size_t i = 0; i < nPts; i++)
        {
            const int row = i % 64;
            const int col = i / 64;

            // intensity comes normalized [0,1]
            const float ptInt = ptsXYZI->getPointIntensity(i);

            const auto pt = mrpt::math::TPoint3Df(xs[i], ys[i], zs[i]);

            rs->rangeImage(row, col)      = pt.norm() / rs->rangeResolution;
            rs->intensityImage(row, col)  = (ptInt / 2048.0) * 255;
            rs->organizedPoints(row, col) = pt;
        }

        // save:
        o = std::dynamic_pointer_cast<mrpt::obs::CObservation>(rs);
    }
#endif

    // Store in the output queue:
    read_ahead_lidar_obs_[step] = std::move(obs);

    MRPT_END
}

mrpt::obs::CObservationPointCloud::Ptr MulranDataset::getPointCloud(
    timestep_t step) const
{
    ASSERT_(initialized_);
    ASSERT_LT_(step, datasetEntries_.size());

    auto it = datasetEntries_.begin();
    std::advance(it, step);

    if (it->second.type != EntryType::Lidar) return {};

    auto lidarIdx = it->second.lidarIdx;

    load_lidar(lidarIdx);
    return read_ahead_lidar_obs_.at(lidarIdx);
}

mrpt::obs::CObservationGPS::Ptr MulranDataset::getGPS(timestep_t step) const
{
    ASSERT_(initialized_);
    ASSERT_LT_(step, datasetEntries_.size());

    auto it = datasetEntries_.begin();
    std::advance(it, step);

    if (it->second.type != EntryType::GNNS) return {};

    return get_gps_by_row_index(it->second.gpsIdx);
}

mrpt::obs::CObservationGPS::Ptr MulranDataset::get_gps_by_row_index(
    size_t row) const
{
    ASSERT_(initialized_);
    ASSERT_LT_(row, static_cast<size_t>(gpsCsvData_.rows()));

    auto obs         = mrpt::obs::CObservationGPS::Create();
    obs->sensorLabel = "gps";
    obs->timestamp   = mrpt::Clock::fromDouble(1e-9 * gpsCsvData_(row, 0));

    // clang-format off
    // column order:
    //   0       1         2         3        4 xx     5       6       7     8 yy     9       10     11      12 zz
    // &stamp,&latitude,&longitude,&altitude,&cov[0],&cov[1],&cov[2],&cov[3],&cov[4],&cov[5],&cov[6],&cov[7],&cov[8]
    // clang-format on

    auto gga = new mrpt::obs::gnss::Message_NMEA_GGA();
    auto msg = mrpt::obs::gnss::gnss_message_ptr(gga);

    mrpt::system::TTimeParts tp;
    mrpt::system::timestampToParts(obs->timestamp, tp);
    gga->fields.UTCTime.hour   = tp.hour;
    gga->fields.UTCTime.minute = tp.minute;
    gga->fields.UTCTime.sec    = tp.second;

    gga->fields.thereis_HDOP = true;
    gga->fields.HDOP = std::sqrt(gpsCsvData_(row, 4)) / HDOP_REFERENCE_METERS;
    gga->fields.altitude_meters   = gpsCsvData_(row, 3);
    gga->fields.fix_quality       = 1;  // regular GPS fix.
    gga->fields.latitude_degrees  = gpsCsvData_(row, 1);
    gga->fields.longitude_degrees = gpsCsvData_(row, 2);
    gga->fields.satellitesUsed    = 10;

    obs->messages[mrpt::obs::gnss::NMEA_GGA] = msg;

    // full 3x3 cov:
    auto& cov = obs->covariance_enu.emplace();
    for (int r = 0, i = 0; r < 3; r++)
        for (int c = 0; c < 3; c++)  //
            cov(r, c) = gpsCsvData_(row, 4 + (i++));

    return obs;
}

size_t MulranDataset::datasetSize() const
{
    ASSERT_(initialized_);
    return datasetEntries_.size();
}

mrpt::obs::CSensoryFrame::Ptr MulranDataset::datasetGetObservations(
    size_t timestep) const
{
    {
        auto lck             = mrpt::lockHelper(dataset_ui_mtx_);
        last_used_tim_index_ = timestep;
    }

    auto sf = mrpt::obs::CSensoryFrame::Create();

    if (publish_lidar_)
    {
        if (auto o = getPointCloud(timestep); o) sf->insert(o);
    }
    if (publish_gps_)
    {
        if (auto o = getGPS(timestep); o) sf->insert(o);
    }

    return sf;
}

constexpr size_t MAX_UNLOAD_LEN = 250;

void MulranDataset::autoUnloadOldEntries() const
{
    while (read_ahead_lidar_obs_.size() > MAX_UNLOAD_LEN)
        read_ahead_lidar_obs_.erase(read_ahead_lidar_obs_.begin());
}

double MulranDataset::LidarFileNameToTimestamp(const std::string& filename)
{
    return 1e-9 * std::stod(mrpt::system::extractFileName(filename));
}
