/* -------------------------------------------------------------------------
 *   A Modular Optimization framework for Localization and mApping  (MOLA)
 * Copyright (C) 2018-2019 Jose Luis Blanco, University of Almeria
 * See LICENSE for license information.
 * ------------------------------------------------------------------------- */
/**
 * @file   KittiOdometryDataset.h
 * @brief  RawDataSource from Kitti odometry/SLAM datasets
 * @author Jose Luis Blanco Claraco
 * @date   Nov 29, 2018
 */

/** \defgroup mola_sensor_kitti_dataset_grp RawDataSource from Kitti
 * odometry/SLAM datasets
 */

#include <mola-sensor-kitti-dataset/KittiOdometryDataset.h>
#include <mrpt/maps/CPointsMapXYZI.h>
#include <mrpt/math/CMatrixTemplateNumeric.h>
#include <mrpt/obs/CObservationImage.h>
#include <mrpt/obs/CObservationVelodyneScan.h>
#include <mrpt/system/CDirectoryExplorer.h>
#include <mrpt/system/filesystem.h>  //ASSERT_DIRECTORY_EXISTS_()
#include <yaml-cpp/yaml.h>

using namespace mola;

MOLA_REGISTER_RAWDATASOURCE(KittiOdometryDataset)

KittiOdometryDataset::KittiOdometryDataset()
{
    this->setLoggerName("KittiOdometryDataset");
}

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

static void parse_calib_line(
    const std::string& line, Eigen::Matrix<double, 3, 4>& M)
{
    MRPT_TRY_START

    std::istringstream ss(line);
    for (Eigen::Index r = 0; r < M.rows(); r++)
        for (Eigen::Index c = 0; c < M.cols(); c++)
        {
            if (!(ss >> M(r, c)))
            {
                THROW_EXCEPTION_FMT(
                    "Error parsing calib line: `%s`", line.c_str());
            }
        }
    MRPT_TRY_END
}

void KittiOdometryDataset::initialize(const std::string& cfg_block)
{
    MRPT_START
    MRPT_LOG_DEBUG_STREAM("Initializing with these params:\n" << cfg_block);

    // Mandatory parameters:
    auto cfg = YAML::Load(cfg_block);

    ENSURE_YAML_ENTRY_EXISTS(cfg, "params");
    auto params = cfg["params"];

    ENSURE_YAML_ENTRY_EXISTS(params, "base_dir");
    kitti_basedir_ = params["base_dir"].as<std::string>();
    ASSERT_DIRECTORY_EXISTS_(kitti_basedir_);

    ENSURE_YAML_ENTRY_EXISTS(params, "sequence");
    replay_selected_seq_ = params["sequence"].as<std::string>();
    const std::string seq_dir =
        kitti_basedir_ + "/sequences/" + replay_selected_seq_;
    ASSERT_DIRECTORY_EXISTS_(seq_dir);

    // Optional params with default values:
    time_warp_scale_ = params["time_warp_scale"].as<double>(time_warp_scale_);
    publish_lidar_   = params["publish_lidar"].as<bool>(publish_lidar_);
    for (unsigned int i = 0; i < 4; i++)
        publish_image_[i] =
            params[mrpt::format("publish_image_%u", i)].as<bool>(
                publish_image_[i]);

    // Make list of all existing files and preload everything we may need later
    // to quickly replay the dataset in realtime:
    MRPT_LOG_INFO_STREAM("Loading kitti dataset from: " << seq_dir);
    // Load timestamps:
    {
        Eigen::VectorXd mtimes;
        // Use MRPT load function:
        mtimes.loadFromTextFile(seq_dir + std::string("/times.txt"));
        // Convert to std::vector:
        lst_timestamps_.resize(static_cast<std::size_t>(mtimes.size()));
        Eigen::VectorXd::Map(&lst_timestamps_[0], mtimes.size()) = mtimes;
    }
    const auto N = lst_timestamps_.size();
    MRPT_LOG_DEBUG_STREAM("Dataset timesteps: " << N);

    // Odometry datasets:
    // Images  : "000000.png"
    // Velodyne: "001092.bin"
    // Raw datasets:
    // Velodyne: "0000000000.bin"
    // Images  : "0000000000.png"
    // Non-uniform format. Solution: make a list of files and sort them.
    build_list_files(seq_dir + "/velodyne", "bin", lst_velodyne_);
    if (!lst_velodyne_.empty())
        ASSERTMSG_(lst_velodyne_.size() == N, "Velodyne: invalid file count");

    MRPT_LOG_INFO_STREAM(
        "Velodyne pointclouds: "
        << (!lst_velodyne_.empty() ? "Found" : "Not found"));

    for (unsigned int i = 0; i < 4; i++)
    {
        build_list_files(
            seq_dir + "/image_" + std::to_string(i), "png", lst_image_[i]);
        if (!lst_image_[i].empty())
            ASSERTMSG_(
                lst_image_[i].size() == N,
                mrpt::format("image_%u: invalid file count", i));

        MRPT_LOG_INFO_STREAM(
            "Camera channel `image_"
            << i << "`: " << (!lst_image_[i].empty() ? "Found" : "Not found"));

        // Override user choice if not possible to publish images:
        if (lst_image_[i].empty()) publish_image_[i] = false;
    }

    // Load sensors calibration:
    {
        const std::string fil_calib = seq_dir + std::string("/calib.txt");
        ASSERT_FILE_EXISTS_(fil_calib);

        // Load projection matrices:
        auto calib = YAML::LoadFile(fil_calib);
        ENSURE_YAML_ENTRY_EXISTS(calib, "P0");
        ENSURE_YAML_ENTRY_EXISTS(calib, "P1");
        ENSURE_YAML_ENTRY_EXISTS(calib, "P2");
        ENSURE_YAML_ENTRY_EXISTS(calib, "P3");
        ENSURE_YAML_ENTRY_EXISTS(calib, "Tr");

        Eigen::Matrix<double, 3, 4> P[4], Tr;
        parse_calib_line(calib["Tr"].as<std::string>(), Tr);
        MRPT_LOG_DEBUG_STREAM("Tr:\n" << Tr);
        for (unsigned int i = 0; i < 4; i++)
        {
            parse_calib_line(
                calib["P" + std::to_string(i)].as<std::string>(), P[i]);
            MRPT_LOG_DEBUG_STREAM("P" << i << " :\n" << P[i]);
        }

        // Camera intrinsics:
        for (unsigned int i = 0; i < 4; i++)
        {
            const double fx = P[i](0, 0), fy = P[i](1, 1), cx = P[i](0, 2),
                         cy = P[i](1, 2);
            cam_intrinsics_[i].setIntrinsicParamsFromValues(fx, fy, cx, cy);

            // Resolution: try loading the first image:
            if (!lst_image_[i].empty())
            {
                mrpt::img::CImage im;
                if (im.loadFromFile(lst_image_[i][0]))
                {
                    cam_intrinsics_[i].ncols = im.getWidth();
                    cam_intrinsics_[i].nrows = im.getHeight();
                }
            }

            // Camera extrinsic params/ pose wrt vehicle origin:
            // cam_poses_[i]
            MRPT_TODO("End calib & sensor pose load");
        }

        MRPT_TODO("Velodyne calib");
    }

    MRPT_END
}
void KittiOdometryDataset::spinOnce()
{
    MRPT_START

    // Starting time:
    if (!replay_started_)
    {
        replay_begin_time_ = mrpt::Clock::now();
        replay_started_    = true;
    }

    // get current replay time:
    const double t =
        mrpt::system::timeDifference(replay_begin_time_, mrpt::Clock::now()) *
        time_warp_scale_;

    if (replay_next_tim_index_ >= lst_timestamps_.size())
    {
        MRPT_LOG_THROTTLE_INFO(
            2.0, "End of dataset reached! Nothing else to publish...");
        return;
    }

    const std::string seq_dir =
        kitti_basedir_ + "/sequences/" + replay_selected_seq_;

    // We have to publish all observations until "t":
    while (replay_next_tim_index_ < lst_timestamps_.size() &&
           t >= lst_timestamps_[replay_next_tim_index_])
    {
        MRPT_LOG_DEBUG_STREAM(
            "Sending observations for replay time: "
            << mrpt::system::formatTimeInterval(t));

        // Save one single timestamp for all observations, since they are in
        // theory shynchronized in the Kitti datasets:
        const auto obs_tim = mrpt::Clock::now();

        if (publish_lidar_)
        {
            // Load velodyne pointcloud:
            const auto f = seq_dir + std::string("/velodyne/") +
                           lst_velodyne_[replay_next_tim_index_];

            // Kitti dataset doesn't contain raw ranges, but point clouds.
            // Load as a PC and convert into a MRPT's observation, leaving empty
            // the fields for raw LiDAR ranges, etc.
            mrpt::maps::CPointsMapXYZI pc;
            if (!pc.loadFromKittiVelodyneFile(f))
                THROW_EXCEPTION_FMT(
                    "Error loading Kitti pointcloud file: `%s`", f.c_str());

            auto obs         = mrpt::obs::CObservationVelodyneScan::Create();
            obs->sensorLabel = "lidar";
            obs->timestamp   = obs_tim;
            const auto N     = pc.getPointsBufferRef_x().size();
            obs->point_cloud.x.resize(N);
            obs->point_cloud.y.resize(N);
            obs->point_cloud.z.resize(N);
            obs->point_cloud.intensity.resize(N);

            // We cannot std::move PC -> obs, due to different std::vector
            // allocators.. (sigh)
            std::memcpy(
                &obs->point_cloud.x[0], &pc.getPointsBufferRef_x()[0],
                sizeof(float) * N);
            std::memcpy(
                &obs->point_cloud.y[0], &pc.getPointsBufferRef_y()[0],
                sizeof(float) * N);
            std::memcpy(
                &obs->point_cloud.z[0], &pc.getPointsBufferRef_z()[0],
                sizeof(float) * N);
            for (std::size_t i = 0; i < N; i++)
                obs->point_cloud.intensity[i] =
                    static_cast<uint8_t>(255.f * pc.getPointIntensity_fast(i));

            MRPT_TODO("Load calib & sensor pose");
            auto o = mrpt::ptr_cast<mrpt::obs::CObservation>::from(obs);
            this->sendObservationsToFrontEnds(o);
        }

        for (unsigned int i = 0; i < 4; i++)
        {
            if (!publish_image_[i]) continue;

            auto obs         = mrpt::obs::CObservationImage::Create();
            obs->sensorLabel = std::string("image_") + std::to_string(i);
            obs->timestamp   = obs_tim;

            ASSERTMSG_(
                lst_image_[i].size() > replay_next_tim_index_,
                mrpt::format("Missing image files for image_%u", i));
            const auto f = seq_dir + std::string("/image_") +
                           std::to_string(i) + std::string("/") +
                           lst_image_[i][replay_next_tim_index_];

            obs->image.setExternalStorage(f);

            MRPT_TODO("Load calib & sensor pose");
            auto o = mrpt::ptr_cast<mrpt::obs::CObservation>::from(obs);
            this->sendObservationsToFrontEnds(o);
        }

        replay_next_tim_index_++;
    }

    MRPT_END
}
