/* -------------------------------------------------------------------------
 *   A Modular Optimization framework for Localization and mApping  (MOLA)
 * Copyright (C) 2018-2019 Jose Luis Blanco, University of Almeria
 * See LICENSE for license information.
 * ------------------------------------------------------------------------- */
/**
 * @file   EurocDataset.cpp
 * @brief  RawDataSource from Euroc odometry/SLAM datasets
 * @author Jose Luis Blanco Claraco
 * @date   Jan 11, 2019
 */

/** \defgroup mola_sensor_euroc_dataset_grp mola_sensor_euroc_dataset_grp.
 * RawDataSource from Euroc odometry/SLAM datasets.
 *
 *
 */

#include <mola-kernel/variant_helper.h>
#include <mola-kernel/yaml_helpers.h>
#include <mola-sensor-euroc-dataset/EurocDataset.h>
#include <mrpt/core/initializer.h>
#include <mrpt/math/CMatrixTemplateNumeric.h>
#include <mrpt/obs/CObservationIMU.h>
#include <mrpt/obs/CObservationImage.h>
#include <mrpt/system/filesystem.h>  //ASSERT_DIRECTORY_EXISTS_()
#include <yaml-cpp/yaml.h>

using namespace mola;
using namespace mola::euroc_dataset;

MRPT_INITIALIZER(do_register){MOLA_REGISTER_MODULE(EurocDataset)}

EurocDataset::EurocDataset() = default;

// Based on: https://stackoverflow.com/a/39146048/1631514
template <typename MATRIX>
static void load_csv(const std::string& path, MATRIX& M)
{
    std::ifstream indata;
    indata.open(path);
    std::string                          line;
    std::vector<typename MATRIX::Scalar> values;
    uint                                 rows = 0;
    while (std::getline(indata, line))
    {
        if (!line.empty() && line[0] == '#') continue;
        std::stringstream lineStream(line);
        std::string       cell;
        while (std::getline(lineStream, cell, ','))
        {
            std::stringstream       cs(cell);
            typename MATRIX::Scalar val;
            cs >> val;
            values.push_back(val);
        }
        ++rows;
    }
    // Convert from RowMajor if needed!
    M = Eigen::Map<const Eigen::Matrix<
        typename MATRIX::Scalar, MATRIX::RowsAtCompileTime,
        MATRIX::ColsAtCompileTime, Eigen::RowMajor>>(
        values.data(), rows, values.size() / rows);
}

void EurocDataset::initialize(const std::string& cfg_block)
{
    using namespace std::string_literals;

    MRPT_START
    ProfilerEntry tle(profiler_, "initialize");

    // Mandatory parameters:
    auto c = YAML::Load(cfg_block);

    ENSURE_YAML_ENTRY_EXISTS(c, "params");
    auto cfg = c["params"];
    MRPT_LOG_DEBUG_STREAM("Initializing with these params:\n" << cfg);

    YAML_LOAD_MEMBER_REQ(base_dir, std::string);
    YAML_LOAD_MEMBER_REQ(sequence, std::string);

    seq_dir_ = base_dir_ + sequence_ + "/mav0"s;
    ASSERT_DIRECTORY_EXISTS_(seq_dir_);
    ASSERT_DIRECTORY_EXISTS_(seq_dir_ + "/cam0"s);
    ASSERT_DIRECTORY_EXISTS_(seq_dir_ + "/cam1"s);
    ASSERT_DIRECTORY_EXISTS_(seq_dir_ + "/imu0"s);

    // Optional params with default values:
    YAML_LOAD_MEMBER_OPT(time_warp_scale, double);

    // Preload everything we may need later to quickly replay the dataset in
    // realtime:
    MRPT_LOG_INFO_STREAM("Loading EUROC dataset from: " << seq_dir_);

    // Cameras: cam{0,1}
    for (uint8_t cam_id = 0; cam_id < 2; cam_id++)
    {
        Eigen::Matrix<uint64_t, Eigen::Dynamic, Eigen::Dynamic> dat;

        const auto cam_data_fil =
            seq_dir_ + "/cam"s + std::to_string(cam_id) + "/data.csv"s;
        ASSERT_FILE_EXISTS_(cam_data_fil);

        load_csv(cam_data_fil, dat);
        ASSERT_(dat.cols() == 2);
        ASSERT_(dat.rows() > 10);

        SensorCamera se_cam;
        se_cam.sensor_name = "cam"s + std::to_string(cam_id);
        se_cam.cam_idx     = cam_id;

        for (int row = 0; row < dat.rows(); row++)
        {
            const auto t = static_cast<euroc_timestamp_t>(dat(row, 0));
            se_cam.img_file_name =
                "/cam"s + std::to_string(cam_id) + "/data/"s +
                std::to_string(static_cast<euroc_timestamp_t>(dat(row, 1))) +
                ".png"s;

            dataset_.emplace_hint(dataset_.end(), t, se_cam);
        }

        MRPT_LOG_INFO_STREAM(
            "cam" << std::to_string(cam_id) << ": Loaded " << dat.rows()
                  << " entries.");

        // Load calibration:
        const auto fil_calib =
            seq_dir_ + "/cam"s + std::to_string(cam_id) + "/sensor.yaml"s;
        ASSERT_FILE_EXISTS_(fil_calib);
        auto cal = YAML::LoadFile(fil_calib);
        ENSURE_YAML_ENTRY_EXISTS(cal, "T_BS");
        auto T_BS = cal["T_BS"];
        ENSURE_YAML_ENTRY_EXISTS(T_BS, "data");
        auto cam_pose = T_BS["data"];

        MRPT_TODO("Load camera poses / intrinsics");

        // cam_poses_
        // cam_intrinsics_
    }  // end for each camera

    // IMU:
    {
        Eigen::MatrixXd dat;

        const auto imu_data_fil = seq_dir_ + "/imu0/data.csv"s;
        ASSERT_FILE_EXISTS_(imu_data_fil);

        load_csv(imu_data_fil, dat);

        // Example rows:
        // clang-format off
        // #timestamp [ns],w_RS_S_x [rad s^-1],w_RS_S_y [rad s^-1],w_RS_S_z [rad s^-1],a_RS_S_x [m s^-2],a_RS_S_y [m s^-2],a_RS_S_z [m s^-2]
        // 1403636579758555392,-0.099134701513277898,0.14730578886832138,0.02722713633111154,8.1476917083333333,-0.37592158333333331,-2.4026292499999999
        // clang-format on
        ASSERT_(dat.cols() == 7);
        ASSERT_(dat.rows() > 10);

        SensorIMU se_imu;
        se_imu.sensor_name = "imu0";

        for (int row = 0; row < dat.rows(); row++)
        {
            const auto t = static_cast<euroc_timestamp_t>(dat(row, 0));
            se_imu.wx    = dat(row, 1);
            se_imu.wy    = dat(row, 2);
            se_imu.wz    = dat(row, 3);
            se_imu.accx  = dat(row, 4);
            se_imu.accy  = dat(row, 5);
            se_imu.accz  = dat(row, 6);

            dataset_.emplace_hint(dataset_.end(), t, se_imu);
        }

        MRPT_LOG_INFO_STREAM("imu0: Loaded " << dat.rows() << " entries.");
    }

    // Debug: dump poses.
    for (unsigned int i = 0; i < cam_poses_.size(); i++)
    {
        mrpt::poses::CPose3D        p(cam_poses_[i]);
        mrpt::math::CMatrixDouble44 T;
        p.getHomogeneousMatrix(T);
        MRPT_LOG_DEBUG_STREAM(
            "cam" << i << " pose on vehicle: " << cam_poses_[i]
                  << "\nTransf. matrix:\n"
                  << T);
    }

    // Start at the dataset begin:
    dataset_next_ = dataset_.begin();

    MRPT_END
}  // end initialize()

void EurocDataset::spinOnce()
{
    MRPT_START

    ProfilerEntry tleg(profiler_, "spinOnce");

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

    // time in [ns]
    const euroc_timestamp_t tim =
        static_cast<euroc_timestamp_t>(t * 1e9) + dataset_.begin()->first;

    if (dataset_next_ == dataset_.end())
    {
        MRPT_LOG_THROTTLE_INFO(
            2.0, "End of dataset reached! Nothing else to publish...");
        return;
    }

    // We have to publish all observations until "t":
    while (dataset_next_ != dataset_.end() && tim >= dataset_next_->first)
    {
        // Convert from dataset format:
        const auto obs_tim =
            mrpt::Clock::fromDouble(dataset_next_->first * 1e-9);

        std::visit(
            overloaded{[&](std::monostate&) {
                           THROW_EXCEPTION("Un-initialized entry!");
                       },
                       [&](SensorCamera& cam) {
                           build_dataset_entry_obs(cam);
                           cam.obs->timestamp = obs_tim;
                           this->sendObservationsToFrontEnds(cam.obs);
                           cam.obs.reset();  // free mem
                       },
                       [&](SensorIMU& imu) {
                           build_dataset_entry_obs(imu);
                           imu.obs->timestamp = obs_tim;
                           this->sendObservationsToFrontEnds(imu.obs);
                           imu.obs.reset();  // free mem
                       }},
            dataset_next_->second);

        // Advance:
        ++dataset_next_;
    }

    // Read ahead to save delays in the next iteration:
    {
        ProfilerEntry tle(profiler_, "spinOnce.read_ahead");

        const unsigned int READ_AHEAD_COUNT = 15;
        auto               peeker           = dataset_next_;
        ++peeker;
        for (unsigned int i = 0;
             i < READ_AHEAD_COUNT && peeker != dataset_.end(); ++i, ++peeker)
        {
            //
            std::visit(
                overloaded{
                    [&](std::monostate&) {
                        THROW_EXCEPTION("Un-initialized entry!");
                    },
                    [&](SensorCamera& cam) { build_dataset_entry_obs(cam); },
                    [&](SensorIMU& imu) { build_dataset_entry_obs(imu); }},
                peeker->second);
        }
    }

    MRPT_END
}

void EurocDataset::build_dataset_entry_obs(SensorCamera& s)
{
    if (s.obs) return;  // already done

    ProfilerEntry tleg(profiler_, "build_obs_img");

    auto obs         = mrpt::obs::CObservationImage::Create();
    obs->sensorLabel = s.sensor_name;

    const auto f = seq_dir_ + s.img_file_name;
    obs->image.setExternalStorage(f);

    // Use this thread time to load images from disk, instead of
    // delegating it to the first use of the image in the consumer:
    obs->image.forceLoad();

    obs->cameraParams = cam_intrinsics_[s.cam_idx];
    obs->setSensorPose(mrpt::poses::CPose3D(cam_poses_[s.cam_idx]));

    s.obs = mrpt::ptr_cast<mrpt::obs::CObservation>::from(obs);
}

void EurocDataset::build_dataset_entry_obs(SensorIMU& s)
{
    using namespace mrpt::obs;

    if (s.obs) return;  // already done

    ProfilerEntry tleg(profiler_, "build_obs_imu");

    MRPT_TODO("Port to CObservationIMU::CreateAlloc() with mem pool");

    auto obs         = CObservationIMU::Create();
    obs->sensorLabel = s.sensor_name;

    obs->dataIsPresent[IMU_WX]    = true;
    obs->dataIsPresent[IMU_WY]    = true;
    obs->dataIsPresent[IMU_WZ]    = true;
    obs->dataIsPresent[IMU_X_ACC] = true;
    obs->dataIsPresent[IMU_Y_ACC] = true;
    obs->dataIsPresent[IMU_Z_ACC] = true;

    obs->rawMeasurements[IMU_WX]    = s.wx;
    obs->rawMeasurements[IMU_WY]    = s.wy;
    obs->rawMeasurements[IMU_X_ACC] = s.accx;
    obs->rawMeasurements[IMU_Y_ACC] = s.accy;
    obs->rawMeasurements[IMU_Z_ACC] = s.accz;

    s.obs = mrpt::ptr_cast<mrpt::obs::CObservation>::from(obs);
}
