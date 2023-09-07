/* -------------------------------------------------------------------------
 *   A Modular Optimization framework for Localization and mApping  (MOLA)
 * Copyright (C) 2018-2023 Jose Luis Blanco, University of Almeria
 * See LICENSE for license information.
 * ------------------------------------------------------------------------- */
/**
 * @file   InputROS2.cpp
 * @brief  RawDataSource from ROS1 topics
 * @author Jose Luis Blanco Claraco
 * @date   Aug 12, 2019
 */

/** \defgroup mola_input_ros1_grp mola_input_ros1_grp.
 * RawDataSource for datasets in MRPT rawlog format
 *
 */

#include <mola_input_ros2/InputROS2.h>
#include <mola_kernel/pretty_print_exception.h>
#include <mola_yaml/yaml_helpers.h>
#include <mrpt/containers/yaml.h>
#include <mrpt/core/initializer.h>
#include <mrpt/maps/CPointsMapXYZI.h>
#include <mrpt/maps/CSimplePointsMap.h>
#include <mrpt/obs/CObservationOdometry.h>
#include <mrpt/obs/CObservationPointCloud.h>
#include <mrpt/ros2bridge/point_cloud2.h>
#include <mrpt/ros2bridge/pose.h>
#include <mrpt/ros2bridge/time.h>
#include <mrpt/system/filesystem.h>

#include <rclcpp/node.hpp>

using namespace mola;

// arguments: class_name, parent_class, class namespace
IMPLEMENTS_MRPT_OBJECT(InputROS2, RawDataSourceBase, mola)

MRPT_INITIALIZER(do_register_InputROS2) { MOLA_REGISTER_MODULE(InputROS2); }

InputROS2::InputROS2() = default;

// The ROS node starts with MOLA::initialize() and ends with its dtor
void InputROS2::ros_node_thread_main(Yaml cfg)
{
    using std::placeholders::_1;

    const char* NODE_NAME = "mola_input_ros2";

    try
    {
        const int         argc    = 1;
        char const* const argv[2] = {NODE_NAME, nullptr};

        // Initialize ROS:
        rclcpp::init(argc, argv);
        auto node = std::make_shared<rclcpp::Node>(NODE_NAME);

        ros_clock_ = node->get_clock();

        // TF:
        tf_buffer_ = std::make_shared<tf2_ros::Buffer>(ros_clock_);
        tf_listener_ =
            std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

        // Subscribe to topics as described by MOLA YAML parameters:
        auto ds_subscribe = cfg["subscribe"];
        if (!ds_subscribe)
        {
            throw std::runtime_error(
                "No topic found for subscription under YAML entry `subscribe`. "
                "It is certainly pointless invoking this MOLA module without "
                "any topic, thus this is understood as a fatal error and will "
                "abort.");
        }

        for (auto topicItem : ds_subscribe.asSequence())
        {
            auto topic = mrpt::containers::yaml(topicItem.asMap());

            ENSURE_YAML_ENTRY_EXISTS(topic, "topic");
            ENSURE_YAML_ENTRY_EXISTS(topic, "type");
            ENSURE_YAML_ENTRY_EXISTS(topic, "output_sensor_label");

            const auto topic_name = topic["topic"].as<std::string>();
            const auto type       = topic["type"].as<std::string>();
            const auto output_sensor_label =
                topic["output_sensor_label"].as<std::string>();
            const auto queue_size = topic.getOrDefault<int>("queue_size", 100);

            // Optional: fixed sensorPose (then ignores/don't need "tf" data):
            std::optional<mrpt::poses::CPose3D> fixedSensorPose;
            if (topic.has("fixed_sensor_pose"))
            {
                fixedSensorPose = mrpt::poses::CPose3D::FromString(
                    topic["fixed_sensor_pose"].as<std::string>());
            }

            if (type == "PointCloud2")
            {
                subsPointCloud_.emplace_back(
                    node->create_subscription<sensor_msgs::msg::PointCloud2>(
                        topic_name, queue_size,
                        [this, output_sensor_label, fixedSensorPose](
                            const sensor_msgs::msg::PointCloud2& o) {
                            this->callbackOnPointCloud2(
                                o, output_sensor_label, fixedSensorPose);
                        }));
            }
            else
            {
                THROW_EXCEPTION_FMT(
                    "Unhandled type=`%s` for topic=`%s`", type.c_str(),
                    topic_name.c_str());
            }
        }

        // Spin:
        rclcpp::spin(node);
        rclcpp::shutdown();
    }
    catch (const std::exception& e)
    {
        mola::pretty_print_exception(e, "InputROS2::ros_node_thread_main");
    }
}

void InputROS2::initialize(const Yaml& c)
{
    using namespace std::string_literals;

    MRPT_START
    ProfilerEntry tle(profiler_, "initialize");

    // Mandatory parameters:
    ENSURE_YAML_ENTRY_EXISTS(c, "params");
    auto cfg = c["params"];

    std::stringstream ss;
    cfg.printAsYAML(ss);
    const Yaml cfgCopy = Yaml::FromStream(ss);

    MRPT_LOG_DEBUG_STREAM("Initializing with these params:\n" << cfgCopy);

    // General params:
    YAML_LOAD_OPT(params_, base_link_frame, std::string);
    YAML_LOAD_OPT(params_, odom_frame, std::string);
    YAML_LOAD_OPT(params_, odom_reference_frame, std::string);
    YAML_LOAD_OPT(params_, publish_odometry, bool);
    YAML_LOAD_OPT(params_, wait_for_tf_timeout_milliseconds, int);

    // Launch ROS node:
    rosNodeThread_ =
        std::thread(&InputROS2::ros_node_thread_main, this, cfgCopy);

    MRPT_END
}  // end initialize()

void InputROS2::spinOnce()
{
    using mrpt::system::timeDifference;

    MRPT_START
    ProfilerEntry tleg(profiler_, "spinOnce");

    if (!rclcpp::ok())
    {
        MRPT_LOG_THROTTLE_ERROR(
            5.0 /*seconds*/, "ROS 2 is in error state (rclcpp::ok()==false)");
        return;
    }

    // Publish odometry?
    publishOdometry();

    MRPT_END
}

void InputROS2::callbackOnPointCloud2(
    const sensor_msgs::msg::PointCloud2& o, const std::string& outSensorLabel,
    const std::optional<mrpt::poses::CPose3D>& fixedSensorPose)
{
    MRPT_START
    ProfilerEntry tle(profiler_, "callbackOnPointCloud2");

    const std::set<std::string> fields = mrpt::ros2bridge::extractFields(o);

    mrpt::maps::CPointsMap::Ptr mapPtr;

    if (fields.count("intensity"))
    {
        auto p = mrpt::maps::CPointsMapXYZI::Create();
        if (!mrpt::ros2bridge::fromROS(o, *p))
            throw std::runtime_error("Error converting ros->mrpt(?)");

        mapPtr = p;
    }
    else
    {
        auto p = mrpt::maps::CSimplePointsMap::Create();
        if (!mrpt::ros2bridge::fromROS(o, *p))
            throw std::runtime_error("Error converting ros->mrpt(?)");

        mapPtr = p;
    }

    auto obs_pc         = mrpt::obs::CObservationPointCloud::Create();
    obs_pc->timestamp   = mrpt::ros2bridge::fromROS(o.header.stamp);
    obs_pc->sensorLabel = outSensorLabel;
    obs_pc->pointcloud  = mapPtr;

    // Sensor pose wrt robot base:
    if (fixedSensorPose)
    {
        // use a fixed, user-provided sensor pose:
        obs_pc->sensorPose = fixedSensorPose.value();
    }
    else
    {
        // Get pose from tf:
        bool ok = waitForTransform(
            obs_pc->sensorPose, o.header.frame_id, params_.base_link_frame,
            o.header.stamp, params_.wait_for_tf_timeout_milliseconds);
        ASSERTMSG_(
            ok,
            mrpt::format(
                "Timeout waiting for /tf transform '%s'->'%s' for timestamp=%f",
                params_.base_link_frame.c_str(), o.header.frame_id.c_str(),
                o.header.stamp.sec + o.header.stamp.nanosec * 1e-9));
    }

    // send it out:
    this->sendObservationsToFrontEnds(obs_pc);

    MRPT_END
}

bool InputROS2::waitForTransform(
    mrpt::poses::CPose3D& des, const std::string& target_frame,
    const std::string& source_frame, const rclcpp::Time& time,
    const int timeoutMilliseconds)
{
    const rclcpp::Duration timeout(0, 1000 * timeoutMilliseconds);
    try
    {
        geometry_msgs::msg::TransformStamped ref_to_trgFrame =
            tf_buffer_->lookupTransform(
                target_frame, source_frame, time,
                tf2::durationFromSec(timeout.seconds()));

        tf2::Transform tf;
        tf2::fromMsg(ref_to_trgFrame.transform, tf);
        des = mrpt::ros2bridge::fromROS(tf);

        MRPT_LOG_DEBUG_FMT(
            "[waitForTransform] Found pose %s -> %s: %s", source_frame.c_str(),
            target_frame.c_str(), des.asString().c_str());

        return true;
    }
    catch (const tf2::TransformException& ex)
    {
        MRPT_LOG_ERROR(ex.what());
        return false;
    }
}

void InputROS2::publishOdometry()
{
    if (!params_.publish_odometry) return;
    ASSERT_(ros_clock_);

    // Get pose from tf:
    mrpt::poses::CPose3D odomPose;

    const auto now = ros_clock_->now();

    bool odom_tf_ok = waitForTransform(
        odomPose, params_.odom_frame, params_.odom_reference_frame, now,
        params_.wait_for_tf_timeout_milliseconds);
    ASSERT_(odom_tf_ok);

    auto obs         = mrpt::obs::CObservationOdometry::Create();
    obs->sensorLabel = "odom";
    obs->timestamp   = mrpt::ros2bridge::fromROS(now);
    obs->odometry    = mrpt::poses::CPose2D(odomPose);

    sendObservationsToFrontEnds(obs);
}
