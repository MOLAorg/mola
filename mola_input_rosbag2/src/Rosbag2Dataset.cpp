/* -------------------------------------------------------------------------
 *   A Modular Optimization framework for Localization and mApping  (MOLA)
 * Copyright (C) 2018-2023 Jose Luis Blanco, University of Almeria
 * See LICENSE for license information.
 * ------------------------------------------------------------------------- */
/**
 * @file   Rosbag2Dataset.h
 * @brief  RawDataSource for datasets in rosbag2 format
 * @author Jose Luis Blanco Claraco
 * @date   Dec 23, 2023
 */

/** \defgroup mola_input_rosbag2_grp mola_input_rosbag2_grp
 * RawDataSource for datasets in rosbag2 format
 *
 * Portions of this program source code are based on
 * rosbag2rawlog (MRPT project), Hunter Laux, 2018, JLBC, 2018-2023.
 */

#include <mola_input_rosbag2/Rosbag2Dataset.h>
#include <mola_yaml/yaml_helpers.h>
#include <mrpt/containers/yaml.h>
#include <mrpt/core/initializer.h>
#include <mrpt/obs/CActionRobotMovement3D.h>
#include <mrpt/obs/CObservation2DRangeScan.h>
#include <mrpt/obs/CObservation3DRangeScan.h>
#include <mrpt/obs/CObservationIMU.h>
#include <mrpt/obs/CObservationOdometry.h>
#include <mrpt/obs/CObservationPointCloud.h>
#include <mrpt/obs/CObservationRotatingScan.h>
#include <mrpt/ros2bridge/imu.h>
#include <mrpt/ros2bridge/laser_scan.h>
#include <mrpt/ros2bridge/point_cloud2.h>
#include <mrpt/ros2bridge/pose.h>
#include <mrpt/ros2bridge/time.h>
#include <mrpt/system/filesystem.h>
#include <tf2/convert.h>
#include <tf2/exceptions.h>

#include <cv_bridge/cv_bridge.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <std_msgs/msg/int32.hpp>
#include <tf2_msgs/msg/tf_message.hpp>

using namespace mola;

// arguments: class_name, parent_class, class namespace
IMPLEMENTS_MRPT_OBJECT(Rosbag2Dataset, RawDataSourceBase, mola)

MRPT_INITIALIZER(do_register_Rosbag2Dataset)
{
    MOLA_REGISTER_MODULE(Rosbag2Dataset);
}

Rosbag2Dataset::Rosbag2Dataset() { this->setLoggerName("Rosbag2Dataset"); }

void Rosbag2Dataset::initialize(const Yaml& c)
{
    using namespace std::string_literals;

    const std::map<std::string, std::string> mapTopic2Class = {
        {"sensor_msgs/msg/Imu", "CObservationIMU"},
        {"sensor_msgs/msg/Image", "CObservationImage"},
        {"sensor_msgs/msg/PointCloud2", "CObservationPointCloud"},
        {"sensor_msgs/msg/LaserScan", "CObservation2DRangeScan"},
    };

    MRPT_START
    ProfilerEntry tle(profiler_, "initialize");

    // Mandatory parameters:
    ENSURE_YAML_ENTRY_EXISTS(c, "params");
    const auto cfg = c["params"];
    MRPT_LOG_DEBUG_STREAM("Initializing with these params:\n" << cfg);

    YAML_LOAD_MEMBER_REQ(rosbag_filename, std::string);
    YAML_LOAD_MEMBER_OPT(time_warp_scale, double);
    YAML_LOAD_MEMBER_OPT(rosbag_storage_id, std::string);
    YAML_LOAD_MEMBER_OPT(rosbag_serialization, std::string);
    YAML_LOAD_MEMBER_OPT(base_link_frame_id, std::string);
    YAML_LOAD_MEMBER_OPT(read_ahead_length, size_t);

    ASSERT_FILE_EXISTS_(rosbag_filename_);

    // Open input ros bag:
    rosbag2_storage::StorageOptions storage_options;

    storage_options.uri        = rosbag_filename_;
    storage_options.storage_id = rosbag_storage_id_;

    rosbag2_cpp::ConverterOptions converter_options;
    converter_options.input_serialization_format  = rosbag_serialization_;
    converter_options.output_serialization_format = rosbag_serialization_;

    MRPT_LOG_INFO_STREAM("Opening: " << storage_options.uri);

    reader_.open(storage_options, converter_options);

    topics_ = reader_.get_all_topics_and_types();

    bagMetaData_ = reader_.get_metadata();

    MRPT_LOG_INFO_STREAM(
        "List of topics found in the bag (" << bagMetaData_.message_count
                                            << " msgs"
                                            << ")");

    // Build map: topic name -> type:
    std::map<std::string, std::string> topic2type;

    for (const auto& t : topics_)
    {
        topic2type[t.name] = t.type;

        MRPT_LOG_INFO_STREAM(" " << t.name << " (" << t.type << ")");
    }

    read_ahead_.clear();
    read_ahead_.resize(bagMetaData_.message_count);
    rosbag_next_idx_ = 0;

    // Begin of code adapted from "Transcriber" class from rosbag2rawlog:

    // Either follow the user-provided "sensors" YAML list, or build it
    // automatically from the list of sensors:
    mrpt::containers::yaml sensorsYaml;

    if (cfg.has("sensors"))
    {
        // Get from the user config:
        ASSERT_(cfg["sensors"].isSequence());

        std::stringstream ss;
        cfg["sensors"].printAsYAML(ss);
        sensorsYaml = mrpt::containers::yaml::FromStream(ss);
    }
    else
    {
        MRPT_LOG_INFO("Automatically building list of mapped topics:");

        // create list automatically:
        sensorsYaml = mrpt::containers::yaml::Sequence();

        for (const auto& t : topics_)
        {
            auto itType = mapTopic2Class.find(t.type);
            if (itType == mapTopic2Class.end())
            {
                MRPT_LOG_INFO_FMT(
                    "- Skipped %25s (%30s): no known mapping to MOLA",
                    t.name.c_str(), t.type.c_str());
                continue;
            }

            mrpt::containers::yaml s = mrpt::containers::yaml::Map();

            s["topic"] = t.name;
            s["type"]  = itType->second;

            sensorsYaml.push_back(s);

            MRPT_LOG_INFO_FMT(
                "- ADDED   %25s (%30s): as %s", t.name.c_str(), t.type.c_str(),
                itType->second.c_str());
        }
    }

    // Start creating topic observers for /tf and all sensors:
    lookup_["/tf"].emplace_back(
        [=](const rosbag2_storage::SerializedBagMessage& rosmsg) {
            return toTf<false>(rosmsg);
        });
    lookup_["/tf_static"].emplace_back(
        [=](const rosbag2_storage::SerializedBagMessage& rosmsg) {
            return toTf<true>(rosmsg);
        });

    for (auto& sensorNode : sensorsYaml.asSequence())
    {
        const auto&       sensor = sensorNode.asMap();
        const std::string topic  = sensor.at("topic").as<std::string>();

        std::string sensorLabel = topic;
        if (sensor.count("sensorLabel"))
            sensorLabel = sensor.at("sensorLabel").as<std::string>();

        // Map to MOLA class: auto or manual:
        std::string sensorType;

        if (sensor.count("type"))
            sensorType = sensor.at("type").as<std::string>();
        else
        {
            ASSERTMSG_(
                topic2type.count(topic),
                mrpt::format(
                    "'sensors' contains topic '%s' which is not found in the "
                    "rosbag!",
                    topic.c_str()));

            auto itType = mapTopic2Class.find(topic2type.at(topic));
            if (itType == mapTopic2Class.end())
            {
                THROW_EXCEPTION_FMT(
                    "'sensors' contains topic '%s' without a 'type' entry, but "
                    "could not automatically determine its mapping to "
                    "mrpt::obs classes.",
                    topic.c_str());
            }
            sensorType = itType->second;
        }

        // Optional: fixed sensorPose (then ignores/don't need "tf" data):
        std::optional<mrpt::poses::CPose3D> fixedSensorPose;
        if (sensor.count("fixed_sensor_pose") != 0)
        {
            fixedSensorPose = mrpt::poses::CPose3D::FromString(
                "["s + sensor.at("fixed_sensor_pose").as<std::string>() + "]"s);
        }

        if (sensorType == "CObservationPointCloud")
        {
            auto callback =
                [=](const rosbag2_storage::SerializedBagMessage& m) {
                    return toPointCloud2(sensorLabel, m, fixedSensorPose);
                };
            lookup_[topic].emplace_back(callback);
        }
#if 0
			else if (sensorType == "CObservation3DRangeScan")
			{
				bool rangeIsDepth = sensor.count("rangeIsDepth")
										? sensor.at("rangeIsDepth").as<bool>()
										: true;
				auto callback = [=](const sensor_msgs::Image::Ptr& image,
									const sensor_msgs::CameraInfo::Ptr& info) {
					return toRangeImage(sensorName, image, info, rangeIsDepth);
				};
			}
#endif
        else if (sensorType == "CObservationImage")
        {
            auto callback =
                [=](const rosbag2_storage::SerializedBagMessage& m) {
                    return toImage(sensorLabel, m, fixedSensorPose);
                };
            lookup_[topic].emplace_back(callback);
        }
        else if (sensorType == "CObservation2DRangeScan")
        {
            auto callback =
                [=](const rosbag2_storage::SerializedBagMessage& m) {
                    return toLidar2D(sensorLabel, m, fixedSensorPose);
                };

            lookup_[topic].emplace_back(callback);
        }
        else if (sensorType == "CObservationRotatingScan")
        {
            auto callback =
                [=](const rosbag2_storage::SerializedBagMessage& m) {
                    return toRotatingScan(sensorLabel, m, fixedSensorPose);
                };
            lookup_[topic].emplace_back(callback);
        }
        else if (sensorType == "CObservationIMU")
        {
            auto callback =
                [=](const rosbag2_storage::SerializedBagMessage& m) {
                    return toIMU(sensorLabel, m, fixedSensorPose);
                };
            lookup_[topic].emplace_back(callback);
        }
        else if (sensorType == "CObservationOdometry")
        {
            auto callback =
                [=](const rosbag2_storage::SerializedBagMessage& m) {
                    return toOdometry(sensorLabel, m);
                };
            lookup_[topic].emplace_back(callback);
        }

        // TODO: Handle more cases?

    }  // end for each "sensor"

    initialized_ = true;
    MRPT_END
}  // end initialize()

void Rosbag2Dataset::spinOnce()
{
    using mrpt::system::timeDifference;

    ASSERTMSG_(initialized_, "You must call initialize() first");

    MRPT_START
    ProfilerEntry tleg(profiler_, "spinOnce");

    // Starting time:
    if (!replay_started_)
    {
        replay_begin_time_ = mrpt::Clock::now();
        replay_started_    = true;
    }

    // get current replay time:
    const double t = timeDifference(replay_begin_time_, mrpt::Clock::now()) *
                     time_warp_scale_;

    if (rosbag_next_idx_ >= read_ahead_.size())
    {
        MRPT_LOG_THROTTLE_INFO(
            10.0,
            "End of dataset reached! Nothing else to publish (CTRL+C to "
            "quit)");
        return;
    }

    // Publish observations up to current time:
    for (;;)
    {
        if (rosbag_next_idx_ >= rosbag_next_idx_write_)
            doReadAhead(rosbag_next_idx_);

        // EOF?
        if (rosbag_next_idx_ >= read_ahead_.size()) break;

        // current dataset entry:
        auto& de = read_ahead_.at(rosbag_next_idx_);
        ASSERT_(de.has_value());

        // Already past the time?
        // First rawlog timestamp?
        if (auto& de_tim = de->timestamp; de_tim)
        {
            if (!rawlog_begin_time_) rawlog_begin_time_ = de_tim.value();

            if (rawlog_begin_time_ &&
                t < timeDifference(*rawlog_begin_time_, de_tim.value()))
            {
                break;
            }
        }

        // Send observations out:
        if (SF::Ptr sf = de->obs; sf)
        {
            for (const auto& obs : *sf)
            {
                this->sendObservationsToFrontEnds(obs);

                if (already_pub_sensor_labels_.count(obs->sensorLabel) == 0)
                {
                    already_pub_sensor_labels_.insert(obs->sensorLabel);
                    MRPT_LOG_INFO_STREAM(
                        "Starting streaming of '"
                        << obs->sensorLabel << "' ("
                        << obs->GetRuntimeClass()->className
                        << ") from the rosbag");
                }

                MRPT_LOG_DEBUG_STREAM(
                    "Publishing "
                    << obs->GetRuntimeClass()->className
                    << " sensorLabel: " << obs->sensorLabel << " for t=" << t
                    << " observation timestamp="
                    << mrpt::system::dateTimeLocalToString(obs->timestamp));
            }
        }

        // Free memory in read-ahead buffer:
        read_ahead_.at(rosbag_next_idx_).reset();

        // Move on:
        rosbag_next_idx_++;
    }
    MRPT_END
}

void Rosbag2Dataset::doReadAhead(const std::optional<size_t>& requestedIndex)
{
    MRPT_START

    // ensure we have observation data at the desired read point, plus a few
    // more:
    const auto startIdx = rosbag_next_idx_;

    ASSERT_GT_(read_ahead_length_, 0);

    // End of read segment:
    size_t endIdx = startIdx + read_ahead_length_;
    if (requestedIndex)
        mrpt::keep_max(endIdx, *requestedIndex + read_ahead_length_);

    mrpt::saturate<size_t>(endIdx, 0, read_ahead_.size());

    for (size_t idx = rosbag_next_idx_; idx < endIdx; idx++)
    {
        if (read_ahead_.at(idx).has_value()) continue;  // already read:

        // serialized data
        ASSERT_EQUAL_(rosbag_next_idx_write_, idx);
        rosbag_next_idx_write_++;

        auto    serialized_message = reader_.read_next();
        SF::Ptr sf                 = to_mrpt(*serialized_message);
        ASSERT_(sf);

        DatasetEntry& de = read_ahead_.at(idx).emplace();

        de.obs = sf;

        if (!sf->empty())
            de.timestamp = sf->getObservationByIndex(0)->timestamp;
    }

    MRPT_END
}

// See docs in base class:
size_t Rosbag2Dataset::datasetSize() const
{
    ASSERTMSG_(initialized_, "You must call initialize() first");

    return bagMetaData_.message_count;
}

mrpt::obs::CSensoryFrame::Ptr Rosbag2Dataset::datasetGetObservations(
    size_t timestep) const
{
    ASSERTMSG_(initialized_, "You must call initialize() first");

    auto& me = const_cast<Rosbag2Dataset&>(*this);

    me.doReadAhead(timestep);

    ASSERT_(read_ahead_.at(timestep).has_value());

    return read_ahead_.at(timestep)->obs;
}

bool Rosbag2Dataset::findOutSensorPose(
    mrpt::poses::CPose3D& des, const std::string& target_frame,
    const std::string&                         source_frame,
    const std::optional<mrpt::poses::CPose3D>& fixedSensorPose)
{
    if (fixedSensorPose)
    {
        des = fixedSensorPose.value();
        return true;
    }

    try
    {
        geometry_msgs::msg::TransformStamped ref_to_trgFrame =
            tfBuffer_->lookupTransform(
                target_frame, source_frame, {} /*latest value*/);

        tf2::Transform tf;
        tf2::fromMsg(ref_to_trgFrame.transform, tf);
        des = mrpt::ros2bridge::fromROS(tf);

        MRPT_LOG_INFO_FMT(
            "[findOutSensorPose] Found pose %s -> %s: %s", source_frame.c_str(),
            target_frame.c_str(), des.asString().c_str());

        return true;
    }
    catch (const tf2::TransformException& ex)
    {
        MRPT_LOG_ERROR_STREAM("findOutSensorPose: " << ex.what());
        return false;
    }
}

Rosbag2Dataset::Obs Rosbag2Dataset::toPointCloud2(
    std::string_view label, const rosbag2_storage::SerializedBagMessage& rosmsg,
    const std::optional<mrpt::poses::CPose3D>& fixedSensorPose)
{
    rclcpp::SerializedMessage serMsg(*rosmsg.serialized_data);
    static rclcpp::Serialization<sensor_msgs::msg::PointCloud2> serializer;

    sensor_msgs::msg::PointCloud2 pts;
    serializer.deserialize_message(&serMsg, &pts);

    auto ptsObs         = mrpt::obs::CObservationPointCloud::Create();
    ptsObs->sensorLabel = label;
    ptsObs->timestamp   = mrpt::ros2bridge::fromROS(pts.header.stamp);

    bool sensorPoseOK = findOutSensorPose(
        ptsObs->sensorPose, pts.header.frame_id, base_link_frame_id_,
        fixedSensorPose);
    ASSERT_(sensorPoseOK);

    // Convert points:
    std::set<std::string> fields = mrpt::ros2bridge::extractFields(pts);

    // We need X Y Z:
    if (!fields.count("x") || !fields.count("y") || !fields.count("z"))
        return {};

    if (fields.count("ring") || fields.count("time"))
    {
        // XYZIRT
        auto mrptPts       = mrpt::maps::CPointsMapXYZIRT::Create();
        ptsObs->pointcloud = mrptPts;

        if (!mrpt::ros2bridge::fromROS(pts, *mrptPts))
        {
            THROW_EXCEPTION(
                "Could not convert pointcloud from ROS to "
                "CPointsMapXYZIRT");
        }
        else
        {  // converted ok:
            return {ptsObs};
        }
    }

    if (fields.count("intensity"))
    {
        // XYZI
        auto mrptPts       = mrpt::maps::CPointsMapXYZI::Create();
        ptsObs->pointcloud = mrptPts;

        if (!mrpt::ros2bridge::fromROS(pts, *mrptPts))
        {
            MRPT_LOG_ONCE_WARN(
                "Could not convert pointcloud from ROS to "
                "CPointsMapXYZI. Trying with XYZ");
        }
        else
        {  // converted ok:
            return {ptsObs};
        }
    }

    {
        // XYZ
        auto mrptPts       = mrpt::maps::CSimplePointsMap::Create();
        ptsObs->pointcloud = mrptPts;

        if (!mrpt::ros2bridge::fromROS(pts, *mrptPts))
            THROW_EXCEPTION(
                "Could not convert pointcloud from ROS to "
                "CSimplePointsMap");
    }

    return {ptsObs};
}

Rosbag2Dataset::Obs Rosbag2Dataset::toLidar2D(
    std::string_view msg, const rosbag2_storage::SerializedBagMessage& rosmsg,
    const std::optional<mrpt::poses::CPose3D>& fixedSensorPose)
{
    rclcpp::SerializedMessage serMsg(*rosmsg.serialized_data);
    static rclcpp::Serialization<sensor_msgs::msg::LaserScan> serializer;

    sensor_msgs::msg::LaserScan scan;
    serializer.deserialize_message(&serMsg, &scan);

    auto scanObs = mrpt::obs::CObservation2DRangeScan::Create();

    // Extract sensor pose from tf frames, if enabled:
    mrpt::poses::CPose3D sensorPose;
    mrpt::ros2bridge::fromROS(scan, sensorPose, *scanObs);

    scanObs->sensorLabel = msg;
    scanObs->timestamp   = mrpt::ros2bridge::fromROS(scan.header.stamp);

    bool sensorPoseOK = findOutSensorPose(
        scanObs->sensorPose, scan.header.frame_id, base_link_frame_id_,
        fixedSensorPose);
    ASSERT_(sensorPoseOK);

    return {scanObs};
}

Rosbag2Dataset::Obs Rosbag2Dataset::toRotatingScan(
    std::string_view msg, const rosbag2_storage::SerializedBagMessage& rosmsg,
    const std::optional<mrpt::poses::CPose3D>& fixedSensorPose)
{
    rclcpp::SerializedMessage serMsg(*rosmsg.serialized_data);
    static rclcpp::Serialization<sensor_msgs::msg::PointCloud2> serializer;

    sensor_msgs::msg::PointCloud2 pts;
    serializer.deserialize_message(&serMsg, &pts);

    // Convert points:
    std::set<std::string> fields = mrpt::ros2bridge::extractFields(pts);

    // We need X Y Z:
    if (!fields.count("x") || !fields.count("y") || !fields.count("z") ||
        !fields.count("ring"))
        return {};

    // As a structured 2D range images, if we have ring numbers:
    auto obsRotScan = mrpt::obs::CObservationRotatingScan::Create();
    // MRPT_TODO("Extract sensor pose from tf frames");
    const mrpt::poses::CPose3D sensorPose;

    if (!mrpt::ros2bridge::fromROS(pts, *obsRotScan, sensorPose))
    {
        THROW_EXCEPTION(
            "Could not convert pointcloud from ROS to "
            "CObservationRotatingScan. Trying another format.");
    }

    obsRotScan->sensorLabel = msg;
    obsRotScan->timestamp   = mrpt::ros2bridge::fromROS(pts.header.stamp);

    bool sensorPoseOK = findOutSensorPose(
        obsRotScan->sensorPose, pts.header.frame_id, base_link_frame_id_,
        fixedSensorPose);
    ASSERT_(sensorPoseOK);

    return {obsRotScan};
}

Rosbag2Dataset::Obs Rosbag2Dataset::toIMU(
    std::string_view msg, const rosbag2_storage::SerializedBagMessage& rosmsg,
    const std::optional<mrpt::poses::CPose3D>& fixedSensorPose)
{
    rclcpp::SerializedMessage serMsg(*rosmsg.serialized_data);
    static rclcpp::Serialization<sensor_msgs::msg::Imu> serializer;

    sensor_msgs::msg::Imu imu;
    serializer.deserialize_message(&serMsg, &imu);

    auto imuObs = mrpt::obs::CObservationIMU::Create();

    imuObs->sensorLabel = msg;
    imuObs->timestamp   = mrpt::ros2bridge::fromROS(imu.header.stamp);

    // Convert data:
    mrpt::ros2bridge::fromROS(imu, *imuObs);

    bool sensorPoseOK = findOutSensorPose(
        imuObs->sensorPose, imu.header.frame_id, base_link_frame_id_,
        fixedSensorPose);
    ASSERT_(sensorPoseOK);

    return {imuObs};
}

Rosbag2Dataset::Obs Rosbag2Dataset::toOdometry(
    std::string_view msg, const rosbag2_storage::SerializedBagMessage& rosmsg)
{
    rclcpp::SerializedMessage serMsg(*rosmsg.serialized_data);
    static rclcpp::Serialization<nav_msgs::msg::Odometry> serializer;

    nav_msgs::msg::Odometry odo;
    serializer.deserialize_message(&serMsg, &odo);

    auto mrptObs = mrpt::obs::CObservationOdometry::Create();

    mrptObs->sensorLabel = msg;
    mrptObs->timestamp   = mrpt::ros2bridge::fromROS(odo.header.stamp);

    // Convert data:
    const auto pose   = mrpt::ros2bridge::fromROS(odo.pose);
    mrptObs->odometry = {pose.mean.x(), pose.mean.y(), pose.mean.yaw()};

    mrptObs->hasVelocities       = true;
    mrptObs->velocityLocal.vx    = odo.twist.twist.linear.x;
    mrptObs->velocityLocal.vy    = odo.twist.twist.linear.y;
    mrptObs->velocityLocal.omega = odo.twist.twist.angular.z;

    return {mrptObs};
}

Rosbag2Dataset::Obs Rosbag2Dataset::toImage(
    std::string_view msg, const rosbag2_storage::SerializedBagMessage& rosmsg,
    const std::optional<mrpt::poses::CPose3D>& fixedSensorPose)
{
    rclcpp::SerializedMessage serMsg(*rosmsg.serialized_data);
    static rclcpp::Serialization<sensor_msgs::msg::Image> serializer;

    auto image = std::make_shared<sensor_msgs::msg::Image>();
    serializer.deserialize_message(&serMsg, image.get());

    auto imgObs = mrpt::obs::CObservationImage::Create();

    imgObs->sensorLabel = msg;
    imgObs->timestamp   = mrpt::ros2bridge::fromROS(image->header.stamp);

    auto cv_ptr = cv_bridge::toCvShare(image);

    imgObs->image = mrpt::img::CImage(cv_ptr->image, mrpt::img::DEEP_COPY);

    bool sensorPoseOK = findOutSensorPose(
        imgObs->cameraPose, image->header.frame_id, base_link_frame_id_,
        fixedSensorPose);
    ASSERT_(sensorPoseOK);

    return {imgObs};
}

template <bool isStatic>
Rosbag2Dataset::Obs Rosbag2Dataset::toTf(
    const rosbag2_storage::SerializedBagMessage& rosmsg)
{
    static rclcpp::Serialization<tf2_msgs::msg::TFMessage> tfSerializer;

    tf2_msgs::msg::TFMessage  tfs;
    rclcpp::SerializedMessage msgData(*rosmsg.serialized_data);
    tfSerializer.deserialize_message(&msgData, &tfs);

    // tf2_msgs::msg::to_block_style_yaml(msg, std::cout);

    for (auto& tf : tfs.transforms)
    {
        try
        {
            tfBuffer_->setTransform(tf, "bagfile", isStatic);
        }
        catch (const tf2::TransformException& ex)
        {
            MRPT_LOG_ERROR_STREAM(ex.what());
        }
    }
    return {};
}

Rosbag2Dataset::SF::Ptr Rosbag2Dataset::to_mrpt(
    const rosbag2_storage::SerializedBagMessage& rosmsg)
{
    auto rets = Rosbag2Dataset::SF::Create();

    auto topic = rosmsg.topic_name;

    if (auto search = lookup_.find(topic); search != lookup_.end())
    {
        for (const auto& callback : search->second)
        {
            auto obs = callback(rosmsg);

            for (const auto& o : obs)  // insert observation:
                rets->insert(o);
        }
    }
    else
    {
        if (unhandledTopics_.count(topic) == 0)
        {
            unhandledTopics_.insert(topic);
            MRPT_LOG_WARN_STREAM("Warning: unhandled topic '" << topic << "'");
        }
    }
    return rets;
}  // end to_mrpt()
