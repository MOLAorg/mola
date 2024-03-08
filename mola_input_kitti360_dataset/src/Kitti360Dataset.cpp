/* -------------------------------------------------------------------------
 *   A Modular Optimization framework for Localization and mApping  (MOLA)
 * Copyright (C) 2018-2024 Jose Luis Blanco, University of Almeria
 * See LICENSE for license information.
 * ------------------------------------------------------------------------- */
/**
 * @file   Kitti360Dataset.cpp
 * @brief  RawDataSource from Kitti-360 dataset
 * @author Jose Luis Blanco Claraco
 * @date   Feb 23, 2024
 */

/** \defgroup mola_input_kitti360_dataset_grp mola-input-kitti360-dataset.
 * RawDataSource from Kitti-360 datasets.
 *
 *
 */

#include <mola_input_kitti360_dataset/Kitti360Dataset.h>
#include <mola_yaml/yaml_helpers.h>
#include <mrpt/containers/yaml.h>
#include <mrpt/core/initializer.h>
#include <mrpt/core/round.h>
#include <mrpt/io/CTextFileLinesParser.h>
#include <mrpt/io/vector_loadsave.h>
#include <mrpt/maps/CPointsMapXYZI.h>
#include <mrpt/maps/CPointsMapXYZIRT.h>
#include <mrpt/obs/CObservationImage.h>
#include <mrpt/obs/CObservationPointCloud.h>
#include <mrpt/obs/CObservationRobotPose.h>
#include <mrpt/obs/CObservationRotatingScan.h>
#include <mrpt/system/CDirectoryExplorer.h>
#include <mrpt/system/filesystem.h>  //ASSERT_DIRECTORY_EXISTS_()

#include <Eigen/Dense>
#include <regex>

using namespace mola;

// arguments: class_name, parent_class, class namespace
IMPLEMENTS_MRPT_OBJECT(Kitti360Dataset, RawDataSourceBase, mola)

MRPT_INITIALIZER(do_register_Kitti360Dataset)
{
    MOLA_REGISTER_MODULE(Kitti360Dataset);
}

Kitti360Dataset::Kitti360Dataset() = default;

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

template <typename MATRIX>
void parse_calib_line(const std::string& line, MATRIX& M)
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

// format:
// 2013-05-28 08:46:02.904295089
// 2013-05-28 08:46:03.008671417
// ...
std::vector<double> parseKitti360Timestamps(const std::string& fil)
{
    std::vector<double> ts;

    mrpt::io::CTextFileLinesParser parser(fil);

    std::string line;
    while (parser.getNextLine(line))
    {
        ASSERT_(line.size() > 25);
        unsigned int year, month, day, hour, min;
        double       sec;

        int nParsedFields = ::sscanf(
            line.c_str(), "%04u-%02u-%02u %02u:%02u:%lf", &year, &month, &day,
            &hour, &min, &sec);
        ASSERT_EQUAL_(nParsedFields, 6);

        mrpt::system::TTimeParts tp;
        tp.year   = year;
        tp.month  = month;
        tp.day    = day;
        tp.hour   = hour;
        tp.minute = min;
        tp.second = sec;

        const auto t = mrpt::system::buildTimestampFromParts(tp);

        ts.push_back(mrpt::Clock::toDouble(t));
    }

    return ts;
}

}  // namespace

void Kitti360Dataset::initialize_rds(const Yaml& c)
{
    using namespace std::string_literals;

    MRPT_START
    ProfilerEntry tle(profiler_, "initialize");

    MRPT_LOG_DEBUG_STREAM("Initializing with these params:\n" << c);

    // Mandatory parameters:
    ENSURE_YAML_ENTRY_EXISTS(c, "params");
    auto cfg = c["params"];

    YAML_LOAD_MEMBER_REQ(base_dir, std::string);
    YAML_LOAD_MEMBER_REQ(sequence, std::string);

    const std::string regExpr =
        "(00|02|03|04|05|06|07|08|09|10|18|test_0|test_1|test_2|test_3)";
    const auto seqRegex = std::regex(regExpr);

    const std::string regExprTests  = "(test_0|test_1|test_2|test_3)";
    const auto        seqRegexTests = std::regex(regExprTests);
    const bool        seqIsTest = std::regex_search(sequence_, seqRegexTests);

    if (!std::regex_search(sequence_, seqRegex))
    {
        THROW_EXCEPTION_FMT(
            "Error: sequence is '%s' but must be one of '%s'",
            sequence_.c_str(), regExpr.c_str());
    }

    // Build path to lidar and images (see .h for expected tree layout)

    // the "velodyne_points" dir, if it exists
    std::string lidar_dir;
    // the image_?? dirs, if they exists
    std::array<std::string, 4> images_dirs;
    std::string                gt_file;

    if (seqIsTest)
    {
        const std::map<char, std::string> syncDirs = {
            {'0', "2013_05_28_drive_0008_sync"},
            {'1', "2013_05_28_drive_0008_sync"},
            {'2', "2013_05_28_drive_0004_sync"},
            {'3', "2013_05_28_drive_0002_sync"},
        };
        const std::string syncDir = syncDirs.at(sequence_.back());

        lidar_dir = mrpt::system::pathJoin(
            {base_dir_, "data_3d_test_slam", sequence_, syncDir,
             "velodyne_points"});
        MRPT_LOG_DEBUG_STREAM("Expected LIDAR dir: " << lidar_dir);

        for (int i = 0; i < 4; i++)
            images_dirs[i] = mrpt::system::pathJoin(
                {base_dir_, "data_2d_test_slam", sequence_, syncDir,
                 mrpt::format("image_%02i", i)});
    }
    else
    {
        const std::string syncDir =
            mrpt::format("2013_05_28_drive_00%s_sync", sequence_.c_str());

        lidar_dir = mrpt::system::pathJoin(
            {base_dir_, "data_3d_raw", syncDir, "velodyne_points"});
        MRPT_LOG_DEBUG_STREAM("Expected LIDAR dir: " << lidar_dir);

        for (int i = 0; i < 4; i++)
        {
            images_dirs[i] = mrpt::system::pathJoin(
                {base_dir_, "data_2d_raw", syncDir,
                 mrpt::format("image_%02i", i)});

            MRPT_LOG_DEBUG_STREAM("Expected images dir: " << images_dirs[i]);
        }

        gt_file = mrpt::system::pathJoin(
            {base_dir_, "data_3d_raw", syncDir, "poses.txt"});
        MRPT_LOG_DEBUG_STREAM("Expected GT file: " << gt_file);
    }

    // clear if does not exist:
    if (!mrpt::system::directoryExists(lidar_dir)) lidar_dir.clear();

    for (auto& imDir : images_dirs)
        if (!mrpt::system::directoryExists(imDir)) imDir.clear();

    if (!mrpt::system::fileExists(gt_file))
    {
        MRPT_LOG_WARN_STREAM(
            "Ground truth poses: not found. Expected file: " << gt_file);
        gt_file.clear();
    }

    // Optional params with default values:
    YAML_LOAD_MEMBER_OPT(time_warp_scale, double);
    paused_ = cfg.getOrDefault<bool>("start_paused", paused_);

    YAML_LOAD_MEMBER_OPT(publish_lidar, bool);
    YAML_LOAD_MEMBER_OPT(generate_lidar_timestamps, bool);
    YAML_LOAD_MEMBER_OPT(publish_ground_truth, bool);

    for (unsigned int i = 0; i < 4; i++)
        publish_image_[i] = cfg.getOrDefault<bool>(
            mrpt::format("publish_image_%u", i), publish_image_[i]);

    // Make list of all existing files and preload everything we may need later
    // to quickly replay the dataset in realtime:
    MRPT_LOG_INFO_STREAM("Loading kitti-360 dataset sequence: " << sequence_);

    // Load LIDAR timestamps:
    auto filLidarTimes = mrpt::system::pathJoin({lidar_dir, "timestamps.txt"});
    ASSERT_FILE_EXISTS_(filLidarTimes);  // TODO: make lidar optional?

    lstLidarTimestamps_ = parseKitti360Timestamps(filLidarTimes);

    const auto N = lstLidarTimestamps_.size();

    lst_velodyne_basedir_ = mrpt::system::pathJoin({lidar_dir, "data"});
    build_list_files(lst_velodyne_basedir_, "bin", lst_velodyne_);
    if (!lst_velodyne_.empty())
        ASSERTMSG_(lst_velodyne_.size() == N, "Velodyne: invalid file count");

    MRPT_LOG_INFO_STREAM(
        "Velodyne pointclouds: "
        << (!lst_velodyne_.empty()
                ? "Found ("s + std::to_string(lst_velodyne_.size()) + ")"s
                : "Not found"));

    for (unsigned int i = 0; i < 4; i++)
    {
        lst_image_basedir_[i] =
            mrpt::system::pathJoin({images_dirs[i], "data_rect"});
        build_list_files(lst_image_basedir_[i], "png", lst_image_[i]);
        if (!lst_image_[i].empty())
            ASSERTMSG_(
                lst_image_[i].size() == N,
                mrpt::format("image_%u: invalid file count", i));

        MRPT_LOG_INFO_STREAM(
            "Camera channel `image_"
            << i << "`: "
            << (!lst_image_[i].empty()
                    ? "Found ("s + std::to_string(lst_image_[i].size()) + ")"s
                    : "Not found"));

        // Override user choice if not possible to publish images:
        if (lst_image_[i].empty()) publish_image_[i] = false;
    }

    // Load sensors calibration:
    const std::string filCalibCamToPose = mrpt::system::pathJoin(
        {base_dir_, "calibration", "calib_cam_to_pose.txt"});
    const std::string filCalibCamToVelo = mrpt::system::pathJoin(
        {base_dir_, "calibration", "calib_cam_to_velo.txt"});
    const std::string filCalibCameras00_01 =
        mrpt::system::pathJoin({base_dir_, "calibration", "perspective.txt"});

    ASSERT_FILE_EXISTS_(filCalibCamToPose);
    ASSERT_FILE_EXISTS_(filCalibCamToVelo);
    ASSERT_FILE_EXISTS_(filCalibCameras00_01);

    // Load projection matrices:
    const auto calib = mrpt::containers::yaml::FromFile(filCalibCameras00_01);
    ENSURE_YAML_ENTRY_EXISTS(calib, "P_rect_00");
    ENSURE_YAML_ENTRY_EXISTS(calib, "R_rect_00");
    ENSURE_YAML_ENTRY_EXISTS(calib, "P_rect_01");
    ENSURE_YAML_ENTRY_EXISTS(calib, "R_rect_01");

    std::array<Eigen::Matrix<double, 3, 4>, 2> P01;  // proj matrix (3x4)
    std::array<Eigen::Matrix<double, 1, 2>, 2> S01;  // resolution (1x2)

    // (T,R): not needed for rectified images
    // std::array<Eigen::Matrix<double, 3, 1>, 2> T01;  // translation (3x1)
    // std::array<Eigen::Matrix<double, 3, 3>, 2> R01;  // rotation (3x3)

    for (int i = 0; i < 2; i++)
    {
        parse_calib_line(
            calib[mrpt::format("P_rect_%02i", i)].as<std::string>(), P01[i]);
        parse_calib_line(
            calib[mrpt::format("S_rect_%02i", i)].as<std::string>(), S01[i]);

        // parse_calib_line(calib[mrpt::format("T%02i", i)].as<std::string>(),
        // T01[i]);
        // parse_calib_line(calib[mrpt::format("R_rect_%02i",
        // i)].as<std::string>(), R01[i]);
    }

    // IMU/GPS is the reference pose, but it's flipped (+Z downwards):
    using namespace mrpt::literals;  // _deg
    const auto T_veh_gps =
        mrpt::poses::CPose3D::FromYawPitchRoll(0.0_deg, 0.0_deg, 180.0_deg);

    // Camera intrinsics:
    for (unsigned int i = 0; i < 2; i++)
    {
        const double fx = P01[i](0, 0), fy = P01[i](1, 1), cx = P01[i](0, 2),
                     cy = P01[i](1, 2);
        cam_intrinsics_[i].setIntrinsicParamsFromValues(fx, fy, cx, cy);

        // Resolution:
        cam_intrinsics_[i].ncols = S01[i][0];
        cam_intrinsics_[i].nrows = S01[i][1];
    }

    // Velodyne pose wrt vehicle =
    const auto yamlCamToPose =
        mrpt::containers::yaml::FromFile(filCalibCamToPose);
    for (int i = 0; i < 4; i++)
    {
        Eigen::Matrix4d T  = Eigen::Matrix4d::Identity();
        auto            Tr = Eigen::Matrix<double, 3, 4>();
        parse_calib_line(
            yamlCamToPose[mrpt::format("image_%02i", i)].as<std::string>(), Tr);
        T.block<3, 4>(0, 0) = Tr;

        cam_poses_[i] = mrpt::poses::CPose3D::FromHomogeneousMatrix(T);
        cam_poses_[i] = T_veh_gps + cam_poses_[i];

        MRPT_LOG_DEBUG_STREAM("Camera 0" << i << " pose: " << cam_poses_[i]);
    }

    // velodyne -> image_00:
    {
        auto Tr = Eigen::Matrix<double, 3, 4>();
        parse_calib_line(mrpt::io::file_get_contents(filCalibCamToVelo), Tr);

        Eigen::Matrix4d T_cam00_to_velo   = Eigen::Matrix4d::Identity();
        T_cam00_to_velo.block<3, 4>(0, 0) = Tr;

        const auto cam00_to_velo =
            mrpt::poses::CPose3D::FromHomogeneousMatrix(T_cam00_to_velo);

        // T_velo = cam00 (+) ((-) cam00_to_velo)
        velodyne_pose_ = cam_poses_[0] + (-cam00_to_velo);

        MRPT_LOG_DEBUG_STREAM("cam00_to_velo    : " << cam00_to_velo);
        MRPT_LOG_DEBUG_STREAM("cam00_to_velo^-1 : " << (-cam00_to_velo));
        MRPT_LOG_DEBUG_STREAM("Velodyne pose: " << velodyne_pose_);
    }

    // Load ground truth poses, if available:
    if (!gt_file.empty())
    {
        mrpt::math::CMatrixDouble M;
        M.loadFromTextFile(gt_file);

        ASSERT_EQUAL_(M.cols(), 12U + 1U);

        // 1st column: index
        // rest: 3x4 matrix

        // Convert into the format expected by MOLA generic interface:
        mrpt::math::CMatrixDouble44 m = mrpt::math::CMatrixDouble44::Identity();
        // T_veh_gps

        for (int i = 0; i < M.rows(); i++)
        {
            const size_t idx = mrpt::round(M(i, 0));

            for (int row = 0, ij_idx = 1 + 0; row < 3; row++)
                for (int col = 0; col < 4; col++, ij_idx++)
                    m(row, col) = M(i, ij_idx);

            // ground truth is for GPS/IMU:
            const auto gtIMU = mrpt::poses::CPose3D::FromHomogeneousMatrix(m);

            // Convert it to the vehicle frame, for consistency with all MOLA
            // datasets:
            const auto gtPose = gtIMU + T_veh_gps;

            groundTruthTrajectory_.insert(
                mrpt::Clock::fromDouble(lstLidarTimestamps_.at(idx)), gtPose);
        }

        MRPT_LOG_INFO("Ground truth poses: Found");
    }

    initialized_ = true;

    MRPT_END
}  // end initialize()

void Kitti360Dataset::spinOnce()
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

    // override by an special teleport order?
    if (teleport_here.has_value() &&
        *teleport_here < lstLidarTimestamps_.size())
    {
        replay_next_tim_index_ = *teleport_here;
        last_dataset_time_     = lstLidarTimestamps_[replay_next_tim_index_];
    }
    else
    {
        if (paused) return;
        // move forward replayed dataset time:
        last_dataset_time_ += dt;
    }

    if (replay_next_tim_index_ >= lstLidarTimestamps_.size())
    {
        onDatasetPlaybackEnds();  // notify base class

        MRPT_LOG_THROTTLE_INFO(
            10.0,
            "End of dataset reached! Nothing else to publish (CTRL+C to quit)");
        return;
    }
    else if (!lstLidarTimestamps_.empty())
    {
        MRPT_LOG_THROTTLE_INFO_FMT(
            5.0, "Dataset replay progress: %lu / %lu  (%4.02f%%)",
            static_cast<unsigned long>(replay_next_tim_index_),
            static_cast<unsigned long>(lstLidarTimestamps_.size()),
            (100.0 * replay_next_tim_index_) / (lstLidarTimestamps_.size()));
    }

    const double t0 = lstLidarTimestamps_.front();

    // We have to publish all observations until "t":
    while (replay_next_tim_index_ < lstLidarTimestamps_.size() &&
           last_dataset_time_ >=
               (lstLidarTimestamps_[replay_next_tim_index_] - t0))
    {
        MRPT_LOG_DEBUG_STREAM(
            "Sending observations for replay time: "
            << mrpt::system::formatTimeInterval(last_dataset_time_));

        // Save one single timestamp for all observations, since they are in
        // theory shynchronized in the Kitti datasets:
        const auto obs_tim = mrpt::Clock::fromDouble(
            lstLidarTimestamps_[replay_next_tim_index_]);

        if (publish_lidar_)
        {
            ProfilerEntry tle(profiler_, "spinOnce.publishLidar");
            load_lidar(replay_next_tim_index_);
            auto o = read_ahead_lidar_obs_[replay_next_tim_index_];
            // o->timestamp = obs_tim; // already done in load_lidar()
            this->sendObservationsToFrontEnds(o);
        }

        for (unsigned int i = 0; i < 4; i++)
        {
            if (!publish_image_[i]) continue;
            ProfilerEntry tle(profiler_, "spinOnce.publishImage");
            load_img(i, replay_next_tim_index_);
            auto o = read_ahead_image_obs_[replay_next_tim_index_][i];
            // o->timestamp = obs_tim; // already done in load_img()
            this->sendObservationsToFrontEnds(o);
        }

        if (publish_ground_truth_ &&
            replay_next_tim_index_ < groundTruthTrajectory_.size())
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
        read_ahead_image_obs_.erase(replay_next_tim_index_);

        replay_next_tim_index_++;
    }

    {
        auto lck = mrpt::lockHelper(dataset_ui_mtx_);
        last_used_tim_index_ =
            replay_next_tim_index_ > 0 ? replay_next_tim_index_ - 1 : 0;
    }

    // Read ahead to save delays in the next iteration:
    if (replay_next_tim_index_ < lstLidarTimestamps_.size())
    {
        ProfilerEntry tle(profiler_, "spinOnce.read_ahead");
        if (0 == read_ahead_image_obs_.count(replay_next_tim_index_))
        {
            for (unsigned int i = 0; i < 4; i++)
            {
                if (!publish_image_[i]) continue;
                load_img(i, replay_next_tim_index_);
            }
        }
        if (0 == read_ahead_lidar_obs_.count(replay_next_tim_index_))
        {
            if (publish_lidar_) load_lidar(replay_next_tim_index_);
        }
    }

    MRPT_END
}

void Kitti360Dataset::load_img(
    const unsigned int cam_idx, const timestep_t step) const
{
    MRPT_START

    // unload() very old observations.
    autoUnloadOldEntries();

    // Already loaded?
    if (read_ahead_image_obs_[step][cam_idx]) return;

    ProfilerEntry tleg(profiler_, "load_img");

    auto obs         = mrpt::obs::CObservationImage::Create();
    obs->sensorLabel = std::string("image_") + std::to_string(cam_idx);

    ASSERTMSG_(
        lst_image_[cam_idx].size() > replay_next_tim_index_,
        mrpt::format("Missing image files for image_%u", cam_idx));
    const auto f = mrpt::system::pathJoin(
        {lst_image_basedir_[cam_idx], lst_image_[cam_idx][step]});

    obs->image.setExternalStorage(f);

    // Use this thread time to load images from disk, instead of
    // delegating it to the first use of the image in the consumer:
    obs->image.forceLoad();

    obs->cameraParams = cam_intrinsics_[cam_idx];
    obs->setSensorPose(mrpt::poses::CPose3D(cam_poses_[cam_idx]));
    obs->timestamp = mrpt::Clock::fromDouble(lstLidarTimestamps_.at(step));

    auto o = mrpt::ptr_cast<mrpt::obs::CObservation>::from(obs);
    read_ahead_image_obs_[step][cam_idx] = std::move(o);

    MRPT_END
}

void Kitti360Dataset::load_lidar(timestep_t step) const
{
    MRPT_START

    // unload() very old observations.
    autoUnloadOldEntries();

    // Already loaded?
    if (read_ahead_lidar_obs_[step]) return;

    ProfilerEntry tleg(profiler_, "load_lidar");

    // Load velodyne pointcloud:
    const auto f =
        mrpt::system::pathJoin({lst_velodyne_basedir_, lst_velodyne_[step]});

    auto obs         = mrpt::obs::CObservationPointCloud::Create();
    obs->sensorLabel = "lidar";
    obs->sensorPose  = velodyne_pose_;
    obs->setAsExternalStorage(
        f,
        mrpt::obs::CObservationPointCloud::ExternalStorageFormat::KittiBinFile);
    obs->load();  // force loading now from disk
    ASSERTMSG_(
        obs->pointcloud,
        mrpt::format("Error loading kitti scan file: '%s'", f.c_str()));

    // Correct wrong intrinsic calibration in the original kitti datasets:
    // Refer to these works & implementations (on which this solution is based
    // on):
    // - IMLS-SLAM
    // - CT-ICP
    // - KISS-ICP
    //
    // See:
    // "IMLS-SLAM: scan-to-model matching based on 3D data", JE Deschaud, 2018.
    //

    // We need to "elevate" each point by this angle: VERTICAL_ANGLE_OFFSET
    if (VERTICAL_ANGLE_OFFSET != 0)
    {
        // Due to the ring-like, rotating nature of 3D LIDARs, we cannot do this
        // in any more efficient way than go through the points one by one:
        auto& xs = obs->pointcloud->getPointsBufferRef_x();
        auto& ys = obs->pointcloud->getPointsBufferRef_y();
        auto& zs = obs->pointcloud->getPointsBufferRef_z();

        const Eigen::Vector3d uz(0., 0., 1.);
        for (size_t i = 0; i < xs.size(); i++)
        {
            const Eigen::Vector3d pt(xs[i], ys[i], zs[i]);
            const Eigen::Vector3d rotationVector = pt.cross(uz);

            const auto aa = Eigen::AngleAxisd(
                VERTICAL_ANGLE_OFFSET, rotationVector.normalized());
            const Eigen::Vector3d newPt = aa * pt;

            obs->pointcloud->setPoint(i, {newPt.x(), newPt.y(), newPt.z()});
        }
    }

    // estimate timestamps based on azimuth?
    if (generate_lidar_timestamps_)
    {
        const auto& xs = obs->pointcloud->getPointsBufferRef_x();
        const auto& ys = obs->pointcloud->getPointsBufferRef_y();

        auto newPts = mrpt::maps::CPointsMapXYZIRT::Create();
        newPts->reserve_XYZIRT(xs.size(), true /*I*/, false /*R*/, true /*T*/);

        auto* trgTs = newPts->getPointsBufferRef_timestamp();
        ASSERT_(trgTs);

        for (size_t i = 0; i < xs.size(); i++)
        {
            newPts->insertPointFrom(*obs->pointcloud, i);

            const auto  azimuth = std::atan2(ys[i], xs[i]);
            const float ptTime  = -0.05f * (azimuth + M_PIf) / (2.0f * M_PIf);
            (*trgTs).push_back(ptTime);
        }

        obs->pointcloud = newPts;
    }

    // Pose: velodyne is at the origin of the vehicle coordinates in
    // Kitti datasets.
    obs->timestamp = mrpt::Clock::fromDouble(lstLidarTimestamps_.at(step));

#if 0  // Export clouds to txt for debugging externally (e.g. python, matlab)
    obs->pointcloud->save3D_to_text_file(
        mrpt::format("kitti_%s_%06zu.txt", sequence_.c_str(), step));
#endif

    mrpt::obs::CObservation::Ptr o;

    o = std::dynamic_pointer_cast<mrpt::obs::CObservation>(obs);

    // Store in the output queue:
    read_ahead_lidar_obs_[step] = std::move(o);

    MRPT_END
}

mrpt::obs::CObservation::Ptr Kitti360Dataset::getPointCloud(
    timestep_t step) const
{
    ASSERT_(initialized_);
    ASSERT_LT_(step, lstLidarTimestamps_.size());

    load_lidar(step);
    auto o = read_ahead_lidar_obs_.at(step);
    return o;
}

std::shared_ptr<mrpt::obs::CObservationImage> Kitti360Dataset::getImage(
    const unsigned int cam_idx, timestep_t step) const
{
    ASSERT_(initialized_);
    ASSERT_LT_(step, lstLidarTimestamps_.size());

    load_img(cam_idx, step);
    auto o = std::dynamic_pointer_cast<mrpt::obs::CObservationImage>(
        read_ahead_image_obs_.at(step).at(cam_idx));
    ASSERT_(o);
    return o;
}

size_t Kitti360Dataset::datasetSize() const
{
    ASSERT_(initialized_);
    return lstLidarTimestamps_.size();
}

mrpt::obs::CSensoryFrame::Ptr Kitti360Dataset::datasetGetObservations(
    size_t timestep) const
{
    {
        auto lck             = mrpt::lockHelper(dataset_ui_mtx_);
        last_used_tim_index_ = timestep;
    }

    auto sf = mrpt::obs::CSensoryFrame::Create();

    for (size_t i = 0; i < publish_image_.size(); i++)
    {
        if (!publish_image_[i]) continue;
        sf->insert(getImage(i, timestep));
    }

    if (publish_lidar_) { sf->insert(getPointCloud(timestep)); }

    return sf;
}

constexpr size_t MAX_UNLOAD_LEN = 250;

void Kitti360Dataset::autoUnloadOldEntries() const
{
    while (read_ahead_lidar_obs_.size() > MAX_UNLOAD_LEN)
        read_ahead_lidar_obs_.erase(read_ahead_lidar_obs_.begin());

    while (read_ahead_image_obs_.size() > MAX_UNLOAD_LEN)
        read_ahead_image_obs_.erase(read_ahead_image_obs_.begin());
}
