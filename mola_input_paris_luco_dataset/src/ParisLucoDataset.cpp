/* -------------------------------------------------------------------------
 *   A Modular Optimization framework for Localization and mApping  (MOLA)
 * Copyright (C) 2018-2024 Jose Luis Blanco, University of Almeria
 * See LICENSE for license information.
 * ------------------------------------------------------------------------- */
/**
 * @file   ParisLucoDataset.cpp
 * @brief  RawDataSource from Paris Luco Dataset
 * @author Jose Luis Blanco Claraco
 * @date   Feb 1, 2024
 */

/** \defgroup mola_input_paris_luco_dataset_grp mola-input-paris-luco-dataset.
 * RawDataSource from Paris Luco Dataset
 *
 */

#include <mola_input_paris_luco_dataset/ParisLucoDataset.h>
#include <mola_yaml/yaml_helpers.h>
#include <mrpt/containers/yaml.h>
#include <mrpt/core/initializer.h>
#include <mrpt/core/round.h>
#include <mrpt/maps/CPointsMapXYZIRT.h>
#include <mrpt/obs/CObservationImage.h>
#include <mrpt/obs/CObservationPointCloud.h>
#include <mrpt/obs/CObservationRobotPose.h>
#include <mrpt/obs/CObservationRotatingScan.h>
#include <mrpt/system/CDirectoryExplorer.h>
#include <mrpt/system/filesystem.h>  //ASSERT_DIRECTORY_EXISTS_()

#include <Eigen/Dense>

using namespace mola;

// arguments: class_name, parent_class, class namespace
IMPLEMENTS_MRPT_OBJECT(ParisLucoDataset, RawDataSourceBase, mola)

MRPT_INITIALIZER(do_register_ParisLucoDataset)
{
    MOLA_REGISTER_MODULE(ParisLucoDataset);
}

ParisLucoDataset::ParisLucoDataset() = default;

static void build_list_files(
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

void ParisLucoDataset::initialize_rds(const Yaml& c)
{
    using namespace std::string_literals;

    MRPT_START
    ProfilerEntry tle(profiler_, "initialize");

    MRPT_LOG_DEBUG_STREAM("Initializing with these params:\n" << c);

    // Mandatory parameters:
    ENSURE_YAML_ENTRY_EXISTS(c, "params");
    auto cfg = c["params"];

    YAML_LOAD_MEMBER_REQ(base_dir, std::string);
    YAML_LOAD_MEMBER_OPT(sequence, std::string);

    seq_dir_ = mrpt::system::pathJoin({base_dir_, sequence_});
    ASSERT_DIRECTORY_EXISTS_(seq_dir_);

    YAML_LOAD_MEMBER_OPT(time_warp_scale, double);
    paused_ = cfg.getOrDefault<bool>("start_paused", paused_);

    // Make list of all existing files and preload everything we may need later
    // to quickly replay the dataset in realtime:
    MRPT_LOG_INFO_STREAM("Loading ParisLuco dataset from: " << seq_dir_);

    build_list_files(
        mrpt::system::pathJoin({seq_dir_, "frames"}), "ply", lstLidarFiles_);

    MRPT_LOG_INFO_STREAM(
        "LIDAR scans: "
        << (!lstLidarFiles_.empty()
                ? "Found ("s + std::to_string(lstLidarFiles_.size()) + ")"s
                : "Not found"));

    // list of timestamps:
    lst_timestamps_.resize(lstLidarFiles_.size());
    {
        double t = 0;
        std::generate(
            lst_timestamps_.begin(), lst_timestamps_.end(),
            [&t, this] { return t += lidarPeriod_; });
    }

    // Load ground truth poses:
    const auto gtFile = mrpt::system::pathJoin({seq_dir_, "gt_traj_lidar.txt"});

    if (mrpt::system::fileExists(gtFile))
    {
        groundTruthTranslations_.loadFromTextFile(gtFile);

        ASSERT_EQUAL_(groundTruthTranslations_.cols(), 3);
        ASSERT_EQUAL_(
            static_cast<size_t>(groundTruthTranslations_.rows()),
            lstLidarFiles_.size());

        MRPT_LOG_INFO("Ground truth translations: Found");
    }
    else
    {
        MRPT_LOG_WARN_STREAM(
            "Ground truth translations: not found. Expected file: " << gtFile);
    }

    readAheadSome();

    initialized_ = true;

    MRPT_END
}  // end initialize()

void ParisLucoDataset::spinOnce()
{
    MRPT_START

    ASSERT_(initialized_);

    ProfilerEntry tleg(profiler_, "spinOnce");

    const auto tNow = mrpt::Clock::now();

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

    // override by an special teleport order?
    if (teleport_here.has_value() && *teleport_here < lst_timestamps_.size())
    {
        replay_next_tim_index_ = *teleport_here;
        last_dataset_time_     = lst_timestamps_.at(*teleport_here);
    }
    else
    {
        if (paused) return;
        // move forward replayed dataset time:
        last_dataset_time_ += dt;
    }

    if (replay_next_tim_index_ >= lst_timestamps_.size())
    {
        onDatasetPlaybackEnds();  // notify base class

        MRPT_LOG_THROTTLE_INFO(
            10.0,
            "End of dataset reached! Nothing else to publish (CTRL+C to quit)");
        return;
    }
    else if (!lst_timestamps_.empty())
    {
        MRPT_LOG_THROTTLE_INFO_FMT(
            5.0, "Dataset replay progress: %lu / %lu  (%4.02f%%)",
            static_cast<unsigned long>(replay_next_tim_index_),
            static_cast<unsigned long>(lst_timestamps_.size()),
            (100.0 * replay_next_tim_index_) / (lst_timestamps_.size()));
    }

    // We have to publish all observations until "t":
    while (replay_next_tim_index_ < lst_timestamps_.size() &&
           last_dataset_time_ >= lst_timestamps_[replay_next_tim_index_])
    {
        MRPT_LOG_DEBUG_STREAM(
            "Sending observations for replay time: "
            << mrpt::system::formatTimeInterval(last_dataset_time_));

        // Save one single timestamp for all observations, since they are in
        // theory shynchronized in the Kitti datasets:
        const auto obs_tim =
            mrpt::Clock::fromDouble(lst_timestamps_[replay_next_tim_index_]);

        {
            ProfilerEntry tle(profiler_, "spinOnce.publishLidar");
            load_lidar(replay_next_tim_index_);
            auto o = read_ahead_lidar_obs_[replay_next_tim_index_];
            // o->timestamp = obs_tim; // already done in load_lidar()
            this->sendObservationsToFrontEnds(o);
        }

        if (groundTruthTrajectory_.size() > replay_next_tim_index_)
        {
            // Get GT pose: it's already stored and correctly transformed
            // into groundTruthTrajectory_:
            auto it = groundTruthTrajectory_.begin();
            std::advance(it, replay_next_tim_index_);

            // Publish as robot pose observation:
            auto o         = mrpt::obs::CObservationRobotPose::Create();
            o->sensorLabel = "ground_truth";
            o->pose.mean   = mrpt::poses::CPose3D(it->second);
            // o->pose.cov? don't use
            o->timestamp = obs_tim;

            this->sendObservationsToFrontEnds(o);
        }

        // Free memory in read-ahead buffers:
        read_ahead_lidar_obs_.erase(replay_next_tim_index_);

        replay_next_tim_index_++;
    }

    {
        auto lck             = mrpt::lockHelper(dataset_ui_mtx_);
        last_used_tim_index_ = replay_next_tim_index_;
    }

    // Read ahead to save delays in the next iteration:
    readAheadSome();

    MRPT_END
}

void ParisLucoDataset::load_lidar(timestep_t step) const
{
    MRPT_START

    // unload() very old observations.
    autoUnloadOldEntries();

    // Already loaded?
    if (read_ahead_lidar_obs_[step]) return;

    ProfilerEntry tleg(profiler_, "load_lidar");

    // Load velodyne pointcloud:
    const auto f =
        mrpt::system::pathJoin({seq_dir_, "frames", lstLidarFiles_[step]});

    auto pts = mrpt::maps::CPointsMapXYZIRT::Create();

    bool ok = pts->loadFromPlyFile(f);
    if (!ok)
        THROW_EXCEPTION_FMT(
            "Error reading scan PLY file '%s': %s", f.c_str(),
            pts->getLoadPLYErrorString().c_str());

    auto obs         = mrpt::obs::CObservationPointCloud::Create();
    obs->sensorLabel = "lidar";
    obs->pointcloud  = pts;

    auto* Ts = pts->getPointsBufferRef_timestamp();
    ASSERT_(Ts);
    const float earliestTime = *std::min_element(Ts->cbegin(), Ts->cend());
    const float shiftTime    = -earliestTime - 0.5 * lidarPeriod_;

    std::transform(Ts->cbegin(), Ts->cend(), Ts->begin(), [=](double t) {
        return t + shiftTime;
    });

    // Fix missing RING_ID: ParisLuco does not have a RING_ID field,
    // but we can generate it from the timestamps + pitch angle:
    ASSERT_(pts->hasRingField());
    ASSERT_EQUAL_(
        pts->getPointsBufferRef_ring()->size(),
        pts->getPointsBufferRef_timestamp()->size());
    std::map<int /*ring*/, std::map<float /*time*/, size_t /*index*/>>
        histogram;

    // Equivalent matlab code:
    // depth = sqrt(D(:,1).^2 + D(:,2).^2);  % (x,y) only
    // pitch = asin((D(:,3)) ./ depth);
    // [nn,xx] =hist(pitch,128);

    const auto&  xs   = pts->getPointsBufferRef_x();
    const auto&  ys   = pts->getPointsBufferRef_y();
    const auto&  zs   = pts->getPointsBufferRef_z();
    const size_t nPts = xs.size();

    const float fov_down = mrpt::DEG2RAD(36.374f);
    const float fov      = mrpt::DEG2RAD(10.860f) + fov_down;

    for (size_t i = 0; i < nPts; i++)
    {
        const float depth = sqrt(mrpt::square(xs[i]) + mrpt::square(ys[i]));
        if (depth < 0.05) continue;
        const float pitch = asin(zs[i] / depth);

        int iP = mrpt::round(31 * (pitch + fov_down) / fov);
        mrpt::saturate(iP, 0, 31);

        auto& vec     = histogram[iP];
        vec[(*Ts)[i]] = i;
    }

    auto& Rs = *pts->getPointsBufferRef_ring();
    for (const auto& [ringId, vec] : histogram)
        for (const auto& [time, idx] : vec) Rs[idx] = ringId;

    // Lidar is at the origin of the vehicle frame:
    obs->sensorPose = mrpt::poses::CPose3D();
    obs->timestamp  = mrpt::Clock::fromDouble(lst_timestamps_.at(step));

#if 0  // Export clouds to txt for debugging externally (e.g. python, matlab)
    obs->pointcloud->save3D_to_text_file(
        mrpt::format("paris_%s_%06zu.txt", sequence_.c_str(), step));
#endif

    mrpt::obs::CObservation::Ptr o;
    o = std::dynamic_pointer_cast<mrpt::obs::CObservation>(obs);
    read_ahead_lidar_obs_[step] = std::move(o);

    MRPT_END
}

void ParisLucoDataset::readAheadSome()
{
    if (replay_next_tim_index_ >= lstLidarFiles_.size()) return;

    ProfilerEntry tle(profiler_, "spinOnce.read_ahead");
    if (0 == read_ahead_lidar_obs_.count(replay_next_tim_index_))
        load_lidar(replay_next_tim_index_);
}

size_t ParisLucoDataset::datasetSize() const
{
    ASSERT_(initialized_);
    return lst_timestamps_.size();
}

mrpt::obs::CSensoryFrame::Ptr ParisLucoDataset::datasetGetObservations(
    size_t timestep) const
{
    ASSERT_(initialized_);
    ASSERT_LT_(timestep, lst_timestamps_.size());

    {
        auto lck             = mrpt::lockHelper(dataset_ui_mtx_);
        last_used_tim_index_ = timestep;
    }

    load_lidar(timestep);
    auto o = read_ahead_lidar_obs_.at(timestep);

    auto sf = mrpt::obs::CSensoryFrame::Create();
    sf->insert(o);
    return sf;
}

constexpr size_t MAX_UNLOAD_LEN = 250;

void ParisLucoDataset::autoUnloadOldEntries() const
{
    while (read_ahead_lidar_obs_.size() > MAX_UNLOAD_LEN)
        read_ahead_lidar_obs_.erase(read_ahead_lidar_obs_.begin());
}
