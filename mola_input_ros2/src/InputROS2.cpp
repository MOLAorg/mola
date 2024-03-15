/* -------------------------------------------------------------------------
 *   A Modular Optimization framework for Localization and mApping  (MOLA)
 * Copyright (C) 2018-2024 Jose Luis Blanco, University of Almeria
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
#include <mrpt/maps/CPointsMapXYZIRT.h>
#include <mrpt/maps/CSimplePointsMap.h>
#include <mrpt/obs/CObservation2DRangeScan.h>
#include <mrpt/obs/CObservationOdometry.h>
#include <mrpt/obs/CObservationPointCloud.h>
#include <mrpt/ros2bridge/laser_scan.h>
#include <mrpt/ros2bridge/point_cloud2.h>
#include <mrpt/ros2bridge/pose.h>
#include <mrpt/ros2bridge/time.h>
#include <mrpt/system/filesystem.h>

#include <rclcpp/node.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

using namespace mola;

// arguments: class_name, parent_class, class namespace
IMPLEMENTS_MRPT_OBJECT(InputROS2, RawDataSourceBase, mola)

MRPT_INITIALIZER(do_register_InputROS2) { MOLA_REGISTER_MODULE(InputROS2); }

InputROS2::InputROS2() = default;

InputROS2::~InputROS2()
{
    try
    {
        rclcpp::shutdown();
        if (rosNodeThread_.joinable()) rosNodeThread_.join();
    }
    catch (const std::exception& e)
    {
        std::cerr << "[~InputROS2] Exception in destructor:\n" << e.what();
    }
}

// The ROS node starts with MOLA::initialize() and ends with its dtor
void InputROS2::ros_node_thread_main(Yaml cfg)
{
    using std::placeholders::_1;
    using namespace std::string_literals;

    const char* NODE_NAME = "mola_input_ros2";

    try
    {
        const int         argc    = 1;
        char const* const argv[2] = {NODE_NAME, nullptr};

        // Initialize ROS:
        // Initialize ROS (only once):
        if (!rclcpp::ok()) { rclcpp::init(argc, argv); }

        auto node = std::make_shared<rclcpp::Node>(NODE_NAME);

        {
            auto lck   = mrpt::lockHelper(ros_clock_mtx_);
            ros_clock_ = node->get_clock();
        }

        // TF:
        tf_buffer_ = std::make_shared<tf2_ros::Buffer>(ros_clock_);
        tf_buffer_->setUsingDedicatedThread(true);
        tf_listener_ =
            std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

        // TODO: Expose QoS params?
        rmw_qos_profile_t qosProfile;
        qosProfile.reliability = RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT;

        // Subscribe to topics as described by MOLA YAML parameters:
        auto ds_subscribe = cfg["subscribe"];
        if (!ds_subscribe.isSequence() || ds_subscribe.asSequence().empty())
        {
            throw std::runtime_error(
                "No topic found for subscription under YAML entry `subscribe`. "
                "It is certainly pointless invoking this MOLA module without "
                "any topic, thus this is understood as a fatal error and will "
                "abort.");
        }

        for (const auto& topicItem : ds_subscribe.asSequence())
        {
            const auto topic = mrpt::containers::yaml(topicItem);

            ENSURE_YAML_ENTRY_EXISTS(topic, "topic");
            ENSURE_YAML_ENTRY_EXISTS(topic, "type");
            ENSURE_YAML_ENTRY_EXISTS(topic, "output_sensor_label");

            const auto topic_name = topic["topic"].as<std::string>();
            const auto type       = topic["type"].as<std::string>();
            const auto output_sensor_label =
                topic["output_sensor_label"].as<std::string>();
            const auto queue_size = topic.getOrDefault<int>("queue_size", 100);

            MRPT_LOG_DEBUG_STREAM(
                "Creating ros2 subscriber for topic='" << topic_name << "' ("
                                                       << type << ")");

            // Optional: fixed sensorPose (then ignores/don't need "tf" data):
            std::optional<mrpt::poses::CPose3D> fixedSensorPose;
            if (topic.has("fixed_sensor_pose") &&
                (!topic.has("use_fixed_sensor_pose") ||
                 !topic["use_fixed_sensor_pose"].as<bool>()))
            {
                fixedSensorPose = mrpt::poses::CPose3D::FromString(
                    "["s + topic["fixed_sensor_pose"].as<std::string>() + "]"s);
            }

            qosProfile.depth = queue_size;
            const auto qosInit =
                rclcpp::QoSInitialization::from_rmw(qosProfile);
            const rclcpp::QoS qos{qosInit, qosProfile};

            if (type == "PointCloud2")
            {
                subsPointCloud_.emplace_back(
                    node->create_subscription<sensor_msgs::msg::PointCloud2>(
                        topic_name, qos,
                        [this, output_sensor_label, fixedSensorPose](
                            const sensor_msgs::msg::PointCloud2& o) {
                            this->callbackOnPointCloud2(
                                o, output_sensor_label, fixedSensorPose);
                        }));
            }
            else if (type == "LaserScan")
            {
                subsLaserScan_.emplace_back(
                    node->create_subscription<sensor_msgs::msg::LaserScan>(
                        topic_name, qos,
                        [this, output_sensor_label, fixedSensorPose](
                            const sensor_msgs::msg::LaserScan& o) {
                            this->callbackOnLaserScan(
                                o, output_sensor_label, fixedSensorPose);
                        }));
            }
            else if (type == "Odometry")
            {
                subsOdometry_.emplace_back(
                    node->create_subscription<nav_msgs::msg::Odometry>(
                        topic_name, qos,
                        [this, output_sensor_label](
                            const nav_msgs::msg::Odometry& o) {
                            this->callbackOnOdometry(o, output_sensor_label);
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

void InputROS2::initialize_rds(const Yaml& c)
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
    YAML_LOAD_OPT(params_, publish_odometry_from_tf, bool);
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

#if 0
    if (!rclcpp::ok())
    {
        MRPT_LOG_THROTTLE_ERROR(
            5.0 /*seconds*/, "ROS 2 is in error state (rclcpp::ok()==false)");
        return;
    }
#endif

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

    if (fields.count("time") || fields.count("timestamp") ||
        fields.count("ring"))
    {
        auto p = mrpt::maps::CPointsMapXYZIRT::Create();
        if (!mrpt::ros2bridge::fromROS(o, *p))
            throw std::runtime_error("Error converting ros->mrpt(?)");

        mapPtr = p;
    }
    else if (fields.count("intensity"))
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
            o.header.stamp, params_.wait_for_tf_timeout_milliseconds,
            true /*print errors*/);
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
    const int timeoutMilliseconds, bool printErrors)
{
    const rclcpp::Duration timeout(0, 1000 * timeoutMilliseconds);
    try
    {
        geometry_msgs::msg::TransformStamped ref_to_trgFrame =
            tf_buffer_->lookupTransform(
                source_frame, target_frame, time,
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
        if (printErrors) MRPT_LOG_ERROR(ex.what());
        return false;
    }
}

void InputROS2::callbackOnOdometry(
    const nav_msgs::msg::Odometry& o, const std::string& outSensorLabel)
{
    MRPT_START
    ProfilerEntry tle(profiler_, "callbackOnOdometry");

    auto obs         = mrpt::obs::CObservationOdometry::Create();
    obs->timestamp   = mrpt::ros2bridge::fromROS(o.header.stamp);
    obs->sensorLabel = outSensorLabel;
    obs->odometry =
        mrpt::poses::CPose2D(mrpt::ros2bridge::fromROS(o.pose.pose));

    obs->hasVelocities       = true;
    obs->velocityLocal.vx    = o.twist.twist.linear.x;
    obs->velocityLocal.vy    = o.twist.twist.linear.y;
    obs->velocityLocal.omega = o.twist.twist.angular.z;

    // send it out:
    this->sendObservationsToFrontEnds(obs);

    MRPT_END
}

void InputROS2::publishOdometry()
{
    if (!params_.publish_odometry_from_tf) return;

    // Is the node already initialized?
    {
        auto lck = mrpt::lockHelper(ros_clock_mtx_);
        if (!ros_clock_) return;  // nope...
    }

    // Get pose from tf:
    mrpt::poses::CPose3D odomPose;

    // ros_clock_->now();
    const auto now = rclcpp::Time();  // last one.

    bool odom_tf_ok = waitForTransform(
        odomPose, params_.base_link_frame, params_.odom_frame, now,
        params_.wait_for_tf_timeout_milliseconds, false /*dont print errors*/);
    if (!odom_tf_ok)
    {
        MRPT_LOG_THROTTLE_WARN_FMT(
            5.0,
            "publish_odometry_from_tf=true, but could not resolve /tf for "
            "odometry: "
            "'%s'->'%s'",
            params_.base_link_frame.c_str(), params_.odom_frame.c_str());
        return;
    }

    auto obs         = mrpt::obs::CObservationOdometry::Create();
    obs->sensorLabel = "odom";
    obs->timestamp   = mrpt::ros2bridge::fromROS(now);
    obs->odometry    = mrpt::poses::CPose2D(odomPose);

    sendObservationsToFrontEnds(obs);
}

void InputROS2::callbackOnLaserScan(
    const sensor_msgs::msg::LaserScan& o, const std::string& outSensorLabel,
    const std::optional<mrpt::poses::CPose3D>& fixedSensorPose)
{
    MRPT_START
    ProfilerEntry tle(profiler_, "callbackOnLaserScan");

    // Sensor pose wrt robot base:
    mrpt::poses::CPose3D sensorPose;
    if (fixedSensorPose)
    {
        // use a fixed, user-provided sensor pose:
        sensorPose = fixedSensorPose.value();
    }
    else
    {
        // Get pose from tf:
        bool ok = waitForTransform(
            sensorPose, o.header.frame_id, params_.base_link_frame,
            o.header.stamp, params_.wait_for_tf_timeout_milliseconds,
            true /*print errors*/);
        ASSERTMSG_(
            ok,
            mrpt::format(
                "Timeout waiting for /tf transform '%s'->'%s' for timestamp=%f",
                params_.base_link_frame.c_str(), o.header.frame_id.c_str(),
                o.header.stamp.sec + o.header.stamp.nanosec * 1e-9));
    }

    auto obs = mrpt::obs::CObservation2DRangeScan::Create();
    mrpt::ros2bridge::fromROS(o, sensorPose, *obs);

    obs->sensorLabel = outSensorLabel;

    // send it out:
    this->sendObservationsToFrontEnds(obs);

    MRPT_END
}
