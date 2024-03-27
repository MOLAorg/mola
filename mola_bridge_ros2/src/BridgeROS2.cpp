/* -------------------------------------------------------------------------
 *   A Modular Optimization framework for Localization and mApping  (MOLA)
 * Copyright (C) 2018-2024 Jose Luis Blanco, University of Almeria
 * See LICENSE for license information.
 * ------------------------------------------------------------------------- */
/**
 * @file   BridgeROS2.cpp
 * @brief  Bridge between MOLA-ROS2
 * @author Jose Luis Blanco Claraco
 * @date   Sep 7, 2023
 */

/** \defgroup mola_bridge_ros2_grp mola_bridge_ros2
 * Bidirectional bridge ROS2-MOLA
 *
 */

// me:
#include <mola_bridge_ros2/BridgeROS2.h>

// MOLA/MRPT:
#include <mola_kernel/pretty_print_exception.h>
#include <mola_yaml/yaml_helpers.h>
#include <mrpt/containers/yaml.h>
#include <mrpt/core/initializer.h>
#include <mrpt/maps/CPointsMapXYZI.h>
#include <mrpt/maps/CPointsMapXYZIRT.h>
#include <mrpt/maps/CSimplePointsMap.h>
#include <mrpt/obs/CObservation2DRangeScan.h>
#include <mrpt/obs/CObservationGPS.h>
#include <mrpt/obs/CObservationIMU.h>
#include <mrpt/obs/CObservationOdometry.h>
#include <mrpt/obs/CObservationPointCloud.h>
#include <mrpt/obs/CObservationRobotPose.h>
#include <mrpt/ros2bridge/gps.h>
#include <mrpt/ros2bridge/image.h>
#include <mrpt/ros2bridge/imu.h>
#include <mrpt/ros2bridge/laser_scan.h>
#include <mrpt/ros2bridge/point_cloud2.h>
#include <mrpt/ros2bridge/pose.h>
#include <mrpt/ros2bridge/time.h>
#include <mrpt/system/filesystem.h>

// ROS 2:
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/node.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

using namespace mola;

// arguments: class_name, parent_class, class namespace
IMPLEMENTS_MRPT_OBJECT(BridgeROS2, RawDataSourceBase, mola)

MRPT_INITIALIZER(do_register_InputROS2) { MOLA_REGISTER_MODULE(BridgeROS2); }

BridgeROS2::BridgeROS2() = default;

BridgeROS2::~BridgeROS2()
{
    try
    {
        rclcpp::shutdown();
        if (rosNodeThread_.joinable()) rosNodeThread_.join();
    }
    catch (const std::exception& e)
    {
        std::cerr << "[~BridgeROS2] Exception in destructor:\n" << e.what();
    }
}

// The ROS node starts with MOLA::initialize() and ends with its dtor
void BridgeROS2::ros_node_thread_main(Yaml cfg)
{
    using namespace std::string_literals;

    const char* NODE_NAME = "mola_bridge_ros2";

    try
    {
        const int         argc    = 1;
        char const* const argv[2] = {NODE_NAME, nullptr};

        // Initialize ROS:
        // Initialize ROS (only once):
        if (!rclcpp::ok()) { rclcpp::init(argc, argv); }

        auto lckNode = mrpt::lockHelper(rosNodeMtx_);

        rosNode_ = std::make_shared<rclcpp::Node>(NODE_NAME);
        lckNode.unlock();

        {
            auto lck   = mrpt::lockHelper(ros_clock_mtx_);
            ros_clock_ = rosNode_->get_clock();
        }

        // TF buffer:
        tf_buffer_ = std::make_shared<tf2_ros::Buffer>(ros_clock_);
        tf_buffer_->setUsingDedicatedThread(true);
        tf_listener_ =
            std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

        // TF broadcaster:
        tf_bc_ = std::make_shared<tf2_ros::TransformBroadcaster>(rosNode_);

        // It seems /tf does not find the connection between frames correctly if
        // using tf_static (!)
        tf_static_bc_ =
            std::make_shared<tf2_ros::StaticTransformBroadcaster>(rosNode_);

        // Subscribe to topics as described by MOLA YAML parameters:
        auto ds_subscribe = cfg["subscribe"];
        if (!ds_subscribe.isSequence() || ds_subscribe.asSequence().empty())
        {
            MRPT_LOG_INFO(
                "No ROS2 topic found for subscription under YAML entry "
                "`subscribe`.");
        }
        else
        {
            internalAnalyzeTopicsToSubscribe(ds_subscribe);
        }

        auto timerLoc = rosNode_->create_wall_timer(
            std::chrono::microseconds(static_cast<unsigned int>(
                1e6 * params_.period_publish_new_localization)),
            [this]() { timerPubLocalization(); });

        auto timerMap = rosNode_->create_wall_timer(
            std::chrono::microseconds(static_cast<unsigned int>(
                1e6 * params_.period_publish_new_map)),
            [this]() { timerPubMap(); });

        // Static tf:
        auto timerStaticTFs = rosNode_->create_wall_timer(
            std::chrono::microseconds(static_cast<unsigned int>(
                1e6 * params_.period_publish_static_tfs)),
            [this]() { publishStaticTFs(); });

        // Spin:
        rclcpp::spin(rosNode_);

        rclcpp::shutdown();
    }
    catch (const std::exception& e)
    {
        mola::pretty_print_exception(e, "BridgeROS2::ros_node_thread_main");
    }
}

void BridgeROS2::initialize_rds(const Yaml& c)
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

    // params of the ROS2->MOLA part:
    YAML_LOAD_OPT(params_, base_link_frame, std::string);
    YAML_LOAD_OPT(params_, odom_frame, std::string);
    YAML_LOAD_OPT(params_, base_footprint_frame, std::string);

    YAML_LOAD_OPT(params_, forward_ros_tf_as_mola_odometry_observations, bool);
    YAML_LOAD_OPT(params_, wait_for_tf_timeout_milliseconds, int);

    if (cfg.has("base_footprint_to_base_link_tf"))
    {
        const auto s = cfg["base_footprint_to_base_link_tf"].as<std::string>();

        // Format: "[x y z yaw pitch roll]" (meters & degrees)
        params_.base_footprint_to_base_link_tf =
            mrpt::math::TPose3D::FromString(s);
    }

    // params of the MOLA-ROS2 part:
    YAML_LOAD_OPT(params_, base_link_frame, std::string);
    YAML_LOAD_OPT(params_, reference_frame, std::string);
    YAML_LOAD_OPT(params_, publish_odometry_msgs_from_slam, bool);
    YAML_LOAD_OPT(params_, publish_in_sim_time, bool);
    YAML_LOAD_OPT(params_, period_publish_new_localization, double);
    YAML_LOAD_OPT(params_, period_publish_new_map, double);
    YAML_LOAD_OPT(params_, publish_tf_from_robot_pose_observations, bool);

    // Launch ROS node:
    rosNodeThread_ =
        std::thread(&BridgeROS2::ros_node_thread_main, this, cfgCopy);

    MRPT_END
}  // end initialize()

void BridgeROS2::spinOnce()
{
    using mrpt::system::timeDifference;

    MRPT_START
    ProfilerEntry tleg(profiler_, "spinOnce");

    // Publish odometry?
    publishOdometry();

    // Check for new mola data sources?
    if (mrpt::Clock::nowDouble() - lastTimeCheckMolaSubs_ >
        params_.period_check_new_mola_subs)
    {
        lastTimeCheckMolaSubs_ = mrpt::Clock::nowDouble();
        doLookForNewMolaSubs();
    }

    MRPT_END
}

void BridgeROS2::callbackOnPointCloud2(
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

        if (!ok)
        {
            MRPT_LOG_ERROR_FMT(
                "Could not forward ROS2 observation to MOLA due to timeout "
                "waiting for /tf transform '%s'->'%s' for timestamp=%f.",
                params_.base_link_frame.c_str(), o.header.frame_id.c_str(),
                o.header.stamp.sec + o.header.stamp.nanosec * 1e-9);
            return;
        }
    }

    // send it out:
    this->sendObservationsToFrontEnds(obs_pc);

    MRPT_END
}

bool BridgeROS2::waitForTransform(
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

void BridgeROS2::callbackOnOdometry(
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

void BridgeROS2::publishOdometry()
{
    if (!params_.forward_ros_tf_as_mola_odometry_observations) return;

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
            "forward_ros_tf_as_mola_odometry_observations=true, but could not "
            "resolve /tf for "
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

void BridgeROS2::callbackOnLaserScan(
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

        if (!ok)
        {
            MRPT_LOG_ERROR_FMT(
                "Could not forward ROS2 observation to MOLA due to timeout "
                "waiting for /tf transform '%s'->'%s' for timestamp=%f.",
                params_.base_link_frame.c_str(), o.header.frame_id.c_str(),
                o.header.stamp.sec + o.header.stamp.nanosec * 1e-9);
            return;
        }
    }

    auto obs = mrpt::obs::CObservation2DRangeScan::Create();
    mrpt::ros2bridge::fromROS(o, sensorPose, *obs);

    obs->sensorLabel = outSensorLabel;

    // send it out:
    this->sendObservationsToFrontEnds(obs);

    MRPT_END
}

void BridgeROS2::callbackOnImu(
    const sensor_msgs::msg::Imu& o, const std::string& outSensorLabel,
    const std::optional<mrpt::poses::CPose3D>& fixedSensorPose)
{
    MRPT_START
    ProfilerEntry tle(profiler_, "callbackOnImu");

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
        if (!ok)
        {
            MRPT_LOG_ERROR_FMT(
                "Could not forward ROS2 observation to MOLA due to timeout "
                "waiting for /tf transform '%s'->'%s' for timestamp=%f.",
                params_.base_link_frame.c_str(), o.header.frame_id.c_str(),
                o.header.stamp.sec + o.header.stamp.nanosec * 1e-9);
            return;
        }
    }

    auto obs = mrpt::obs::CObservationIMU::Create();
    mrpt::ros2bridge::fromROS(o, *obs);

    obs->sensorPose  = sensorPose;
    obs->sensorLabel = outSensorLabel;

    // send it out:
    this->sendObservationsToFrontEnds(obs);

    MRPT_END
}

void BridgeROS2::callbackOnNavSatFix(
    const sensor_msgs::msg::NavSatFix& o, const std::string& outSensorLabel,
    const std::optional<mrpt::poses::CPose3D>& fixedSensorPose)
{
    MRPT_START
    ProfilerEntry tle(profiler_, "callbackOnNavSatFix");

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
        if (!ok)
        {
            MRPT_LOG_ERROR_FMT(
                "Could not forward ROS2 observation to MOLA due to timeout "
                "waiting for /tf transform '%s'->'%s' for timestamp=%f.",
                params_.base_link_frame.c_str(), o.header.frame_id.c_str(),
                o.header.stamp.sec + o.header.stamp.nanosec * 1e-9);
            return;
        }
    }

    auto obs = mrpt::obs::CObservationGPS::Create();
    mrpt::ros2bridge::fromROS(o, *obs);

    obs->sensorPose  = sensorPose;
    obs->sensorLabel = outSensorLabel;

    // send it out:
    this->sendObservationsToFrontEnds(obs);

    MRPT_END
}

void BridgeROS2::onNewObservation(const CObservation::Ptr& o)
{
    using namespace mrpt::obs;

    ASSERT_(o);

    // TODO(jlbc): Add some filter not to publish everything to ROS?

    if (auto oImg = std::dynamic_pointer_cast<CObservationImage>(o); oImg)
    {
        return internalOn(*oImg);
    }
    else if (auto oPc = std::dynamic_pointer_cast<CObservationPointCloud>(o);
             oPc)
    {
        return internalOn(*oPc);
    }
    else if (auto oLS = std::dynamic_pointer_cast<CObservation2DRangeScan>(o);
             oLS)
    {
        return internalOn(*oLS);
    }
    else if (auto oRP = std::dynamic_pointer_cast<CObservationRobotPose>(o);
             oRP)
    {
        return internalOn(*oRP);
    }
    else if (auto oGPS = std::dynamic_pointer_cast<CObservationGPS>(o); oGPS)
    {
        return internalOn(*oGPS);
    }
    else
    {
        MRPT_LOG_THROTTLE_WARN_FMT(
            5.0,
            "Do not know how to publish to ROS an observation of type '%s' "
            "with sensorLabel='%s'",
            o->GetRuntimeClass()->className, o->sensorLabel.c_str());
    }
}

void BridgeROS2::internalOn(const mrpt::obs::CObservationImage& obs)
{
    auto lck = mrpt::lockHelper(rosPubsMtx_);

    // Create the publisher the first time an observation arrives:
    const bool is_1st_pub = rosPubs_.pub_sensors.find(obs.sensorLabel) ==
                            rosPubs_.pub_sensors.end();
    auto& pub = rosPubs_.pub_sensors[obs.sensorLabel];

    if (is_1st_pub)
    {
        // REP-2003: Sensor sources should use SystemDefaultsQoS
        // See: https://ros.org/reps/rep-2003.html
        pub = rosNode()->create_publisher<sensor_msgs::msg::Image>(
            obs.sensorLabel, rclcpp::SystemDefaultsQoS());
    }
    lck.unlock();

    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pubImg =
        std::dynamic_pointer_cast<rclcpp::Publisher<sensor_msgs::msg::Image>>(
            pub);
    ASSERT_(pubImg);

    const std::string sSensorFrameId = obs.sensorLabel;

    // Send TF:
    mrpt::poses::CPose3D sensorPose;
    obs.getSensorPose(sensorPose);

    tf2::Transform transform = mrpt::ros2bridge::toROS_tfTransform(sensorPose);

    geometry_msgs::msg::TransformStamped tfStmp;
    tfStmp.transform       = tf2::toMsg(transform);
    tfStmp.child_frame_id  = sSensorFrameId;
    tfStmp.header.frame_id = params_.base_link_frame;
    tfStmp.header.stamp    = myNow(obs.timestamp);
    tf_bc_->sendTransform(tfStmp);

    // Send observation:
    {
        obs.load();

        // Convert observation MRPT -> ROS
        sensor_msgs::msg::Image msg_img;
        std_msgs::msg::Header   msg_header;
        msg_header.stamp    = myNow(obs.timestamp);
        msg_header.frame_id = sSensorFrameId;

        msg_img = mrpt::ros2bridge::toROS(obs.image, msg_header);

        pubImg->publish(msg_img);
    }
}

void BridgeROS2::internalOn(const mrpt::obs::CObservation2DRangeScan& obs)
{
    auto lck = mrpt::lockHelper(rosPubsMtx_);

    // Create the publisher the first time an observation arrives:
    const bool is_1st_pub = rosPubs_.pub_sensors.find(obs.sensorLabel) ==
                            rosPubs_.pub_sensors.end();
    auto& pub = rosPubs_.pub_sensors[obs.sensorLabel];

    if (is_1st_pub)
    {
        // REP-2003: Sensor sources should use SystemDefaultsQoS
        // See: https://ros.org/reps/rep-2003.html
        pub = rosNode()->create_publisher<sensor_msgs::msg::LaserScan>(
            obs.sensorLabel, rclcpp::SystemDefaultsQoS());
    }
    lck.unlock();

    rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr pubLS =
        std::dynamic_pointer_cast<
            rclcpp::Publisher<sensor_msgs::msg::LaserScan>>(pub);
    ASSERT_(pubLS);

    const std::string sSensorFrameId = obs.sensorLabel;

    // Send TF:
    mrpt::poses::CPose3D sensorPose;
    obs.getSensorPose(sensorPose);

    tf2::Transform transform = mrpt::ros2bridge::toROS_tfTransform(sensorPose);

    geometry_msgs::msg::TransformStamped tfStmp;
    tfStmp.transform       = tf2::toMsg(transform);
    tfStmp.child_frame_id  = sSensorFrameId;
    tfStmp.header.frame_id = params_.base_link_frame;
    tfStmp.header.stamp    = myNow(obs.timestamp);
    tf_bc_->sendTransform(tfStmp);

    // Send observation:
    {
        obs.load();

        // Convert observation MRPT -> ROS
        sensor_msgs::msg::LaserScan msg;
        mrpt::ros2bridge::toROS(obs, msg);

        msg.header.stamp    = myNow(obs.timestamp);
        msg.header.frame_id = sSensorFrameId;

        pubLS->publish(msg);
    }
}

void BridgeROS2::internalOn(const mrpt::obs::CObservationPointCloud& obs)
{
    internalOn(
        obs, true /*it is a real sensor, publish its /tf*/,
        obs.sensorLabel /* frame_id */);
}

void BridgeROS2::internalOn(
    const mrpt::obs::CObservationPointCloud& obs, bool isSensorTopic,
    const std::string& sSensorFrameId)
{
    using namespace std::string_literals;

    auto lck = mrpt::lockHelper(rosPubsMtx_);

    const auto lbPoints = obs.sensorLabel + "_points"s;

    // Create the publisher the first time an observation arrives:
    const bool is_1st_pub =
        rosPubs_.pub_sensors.find(lbPoints) == rosPubs_.pub_sensors.end();

    auto& pubPts = rosPubs_.pub_sensors[lbPoints];

    if (is_1st_pub)
    {
        // REP-2003: https://ros.org/reps/rep-2003.html#id5
        // - Sensors: SystemDefaultsQoS()
        // - Maps:  reliable transient-local
        auto qos =
            isSensorTopic
                ? rclcpp::SystemDefaultsQoS()
                : rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable();

        pubPts = rosNode()->create_publisher<sensor_msgs::msg::PointCloud2>(
            lbPoints, qos);
    }
    lck.unlock();

    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pubPoints =
        std::dynamic_pointer_cast<
            rclcpp::Publisher<sensor_msgs::msg::PointCloud2>>(pubPts);
    ASSERT_(pubPoints);

    const std::string sSensorFrameId_points = lbPoints;

    // POINTS
    // --------

    // Send TF:
    if (isSensorTopic)
    {
        mrpt::poses::CPose3D sensorPose = obs.sensorPose;

        tf2::Transform transform =
            mrpt::ros2bridge::toROS_tfTransform(sensorPose);

        geometry_msgs::msg::TransformStamped tfStmp;
        tfStmp.transform       = tf2::toMsg(transform);
        tfStmp.child_frame_id  = sSensorFrameId;
        tfStmp.header.frame_id = params_.base_link_frame;
        tfStmp.header.stamp    = myNow(obs.timestamp);
        tf_bc_->sendTransform(tfStmp);
    }

    // Send observation:
    if (obs.pointcloud)
    {
        // Convert observation MRPT -> ROS
        sensor_msgs::msg::PointCloud2 msg_pts;
        std_msgs::msg::Header         msg_header;
        msg_header.stamp    = myNow(obs.timestamp);
        msg_header.frame_id = sSensorFrameId;

        obs.load();

        if (auto* xyzirt = dynamic_cast<const mrpt::maps::CPointsMapXYZIRT*>(
                obs.pointcloud.get());
            xyzirt)
        {
            mrpt::ros2bridge::toROS(*xyzirt, msg_header, msg_pts);
        }
        else if (auto* xyzi = dynamic_cast<const mrpt::maps::CPointsMapXYZI*>(
                     obs.pointcloud.get());
                 xyzi)
        {
            mrpt::ros2bridge::toROS(*xyzi, msg_header, msg_pts);
        }
        else if (auto* sPts = dynamic_cast<const mrpt::maps::CSimplePointsMap*>(
                     obs.pointcloud.get());
                 sPts)
        {
            mrpt::ros2bridge::toROS(*sPts, msg_header, msg_pts);
        }
        else
        {
            THROW_EXCEPTION_FMT(
                "Do not know how to handle this variant of CPointsMap: "
                "class='%s'",
                obs.pointcloud->GetRuntimeClass()->className);
        }

        pubPoints->publish(msg_pts);
    }
}

void BridgeROS2::internalOn(const mrpt::obs::CObservationRobotPose& obs)
{
    auto lck = mrpt::lockHelper(rosPubsMtx_);

    ASSERT_(!obs.sensorLabel.empty());

    // Create the publisher the first time an observation arrives:
    const bool is_1st_pub = rosPubs_.pub_sensors.find(obs.sensorLabel) ==
                            rosPubs_.pub_sensors.end();
    auto& pub = rosPubs_.pub_sensors[obs.sensorLabel];

    if (is_1st_pub)
    {
        // REP-2003: Sensor sources should use SystemDefaultsQoS
        // See: https://ros.org/reps/rep-2003.html
        pub = rosNode()->create_publisher<nav_msgs::msg::Odometry>(
            obs.sensorLabel, rclcpp::SystemDefaultsQoS());
    }
    lck.unlock();

    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pubOdo =
        std::dynamic_pointer_cast<rclcpp::Publisher<nav_msgs::msg::Odometry>>(
            pub);
    ASSERT_(pubOdo);

    // Send TF:
    if (params_.publish_tf_from_robot_pose_observations)
    {
        tf2::Transform transform =
            mrpt::ros2bridge::toROS_tfTransform(obs.pose.mean);

        geometry_msgs::msg::TransformStamped tfStmp;
        tfStmp.transform       = tf2::toMsg(transform);
        tfStmp.child_frame_id  = params_.base_link_frame;
        tfStmp.header.frame_id = params_.reference_frame;
        tfStmp.header.stamp    = myNow(obs.timestamp);
        tf_bc_->sendTransform(tfStmp);
    }

    // Send observation:
    {
        obs.load();

        // Convert observation MRPT -> ROS
        nav_msgs::msg::Odometry msg;
        msg.header.stamp    = myNow(obs.timestamp);
        msg.header.frame_id = params_.reference_frame;

        msg.pose = mrpt::ros2bridge::toROS_Pose(obs.pose);

        pubOdo->publish(msg);
    }
}

void BridgeROS2::internalOn(const mrpt::obs::CObservationGPS& obs)
{
    auto lck = mrpt::lockHelper(rosPubsMtx_);

    // Create the publisher the first time an observation arrives:
    const bool is_1st_pub = rosPubs_.pub_sensors.find(obs.sensorLabel) ==
                            rosPubs_.pub_sensors.end();
    auto& pub = rosPubs_.pub_sensors[obs.sensorLabel];

    if (is_1st_pub)
    {
        // REP-2003: Sensor sources should use SystemDefaultsQoS
        // See: https://ros.org/reps/rep-2003.html
        pub = rosNode()->create_publisher<sensor_msgs::msg::NavSatFix>(
            obs.sensorLabel, rclcpp::SystemDefaultsQoS());
    }
    lck.unlock();

    rclcpp::Publisher<sensor_msgs::msg::NavSatFix>::SharedPtr pubGPS =
        std::dynamic_pointer_cast<
            rclcpp::Publisher<sensor_msgs::msg::NavSatFix>>(pub);
    ASSERT_(pubGPS);

    const std::string sSensorFrameId = obs.sensorLabel;

    // Send TF:
    mrpt::poses::CPose3D sensorPose;
    obs.getSensorPose(sensorPose);

    tf2::Transform transform = mrpt::ros2bridge::toROS_tfTransform(sensorPose);

    geometry_msgs::msg::TransformStamped tfStmp;
    tfStmp.transform       = tf2::toMsg(transform);
    tfStmp.child_frame_id  = sSensorFrameId;
    tfStmp.header.frame_id = params_.base_link_frame;
    tfStmp.header.stamp    = myNow(obs.timestamp);
    tf_bc_->sendTransform(tfStmp);

    // Send observation:
    {
        obs.load();

        // Convert observation MRPT -> ROS
        std_msgs::msg::Header header;
        header.stamp    = myNow(obs.timestamp);
        header.frame_id = sSensorFrameId;

        sensor_msgs::msg::NavSatFix msg;
        mrpt::ros2bridge::toROS(obs, header, msg);

        pubGPS->publish(msg);
    }
}

void BridgeROS2::doLookForNewMolaSubs()
{
    using namespace std::string_literals;

    auto lck = mrpt::lockHelper(molaSubsMtx_);

    // RawDataSourceBase:
    auto listRDS = this->findService<mola::RawDataSourceBase>();
    for (auto& module : listRDS)
    {
        auto rds = std::dynamic_pointer_cast<mola::RawDataSourceBase>(module);
        ASSERT_(rds);

        // Skip myself!
        if (std::string(rds->GetRuntimeClass()->className) ==
            "mola::BridgeROS2"s)
            continue;

        if (molaSubs_.dataSources.count(rds) == 0)
        {
            MRPT_LOG_INFO_STREAM(
                "Subscribing to MOLA data source module '"
                << rds->getModuleInstanceName() << "'");

            // a new one:
            molaSubs_.dataSources.insert(rds);
            rds->attachToDataConsumer(*this);
        }
    }

    // LocalizationSourceBase:
    auto listLoc = this->findService<mola::LocalizationSourceBase>();
    for (auto& module : listLoc)
    {
        auto loc =
            std::dynamic_pointer_cast<mola::LocalizationSourceBase>(module);
        ASSERT_(loc);

        if (molaSubs_.locSources.count(loc) == 0)
        {
            MRPT_LOG_INFO_STREAM(
                "Subscribing to MOLA localization source module '"
                << module->getModuleInstanceName() << "'");

            // a new one:
            molaSubs_.locSources.insert(loc);
            loc->subscribeToLocalizationUpdates(
                [this](const auto& l) { onNewLocalization(l); });
        }
    }

    // MapSourceBase
    auto listMap = this->findService<mola::MapSourceBase>();
    for (auto& module : listMap)
    {
        auto ms = std::dynamic_pointer_cast<mola::MapSourceBase>(module);
        ASSERT_(ms);

        if (molaSubs_.mapSources.count(ms) == 0)
        {
            MRPT_LOG_INFO_STREAM(
                "Subscribing to MOLA map source module '"
                << module->getModuleInstanceName() << "'");

            // a new one:
            molaSubs_.mapSources.insert(ms);
            ms->subscribeToMapUpdates([this](const auto& m) { onNewMap(m); });
        }
    }
}

rclcpp::Time BridgeROS2::myNow(const mrpt::Clock::time_point& observationStamp)
{
    if (params_.publish_in_sim_time)
        return mrpt::ros2bridge::toROS(observationStamp);
    else
        return mrpt::ros2bridge::toROS(mrpt::Clock::now());
}

void BridgeROS2::onNewLocalization(
    const mola::LocalizationSourceBase::LocalizationUpdate& l)
{
    auto lck = mrpt::lockHelper(lastLocMapMtx_);

    lastLoc_ = l;
}

void BridgeROS2::onNewMap(const mola::MapSourceBase::MapUpdate& m)
{
    auto lck = mrpt::lockHelper(lastLocMapMtx_);

    lastMaps_[m.map_name] = m;
}

void BridgeROS2::timerPubLocalization()
{
    using namespace std::string_literals;

    // get a copy of the data minimizing the time owning the mutex:
    std::optional<mola::LocalizationSourceBase::LocalizationUpdate> l;
    {
        auto lck = mrpt::lockHelper(lastLocMapMtx_);
        l        = lastLoc_;
        lastLoc_.reset();
    }
    if (!l) return;

    MRPT_LOG_DEBUG_STREAM(
        "New localization available from '"
        << l->method << "' frame: '" << l->reference_frame
        << "' t=" << mrpt::system::dateTimeLocalToString(l->timestamp)
        << " pose=" << l->pose.asString());

    // 1/2: Publish to /tf:
    const std::string locLabel =
        (l->method.empty() ? "slam"s : l->method) + "/pose"s;

    auto lck = mrpt::lockHelper(rosPubsMtx_);

    // Create the publisher the first time an observation arrives:
    const bool is_1st_pub =
        rosPubs_.pub_sensors.find(locLabel) == rosPubs_.pub_sensors.end();
    auto& pub = rosPubs_.pub_sensors[locLabel];

    if (is_1st_pub)
    {
        pub = rosNode()->create_publisher<nav_msgs::msg::Odometry>(
            locLabel, rclcpp::SystemDefaultsQoS());
    }
    lck.unlock();

    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pubOdo =
        std::dynamic_pointer_cast<rclcpp::Publisher<nav_msgs::msg::Odometry>>(
            pub);
    ASSERT_(pubOdo);

    // Send TF:
    tf2::Transform transform = mrpt::ros2bridge::toROS_tfTransform(l->pose);

    geometry_msgs::msg::TransformStamped tfStmp;
    tfStmp.transform       = tf2::toMsg(transform);
    tfStmp.child_frame_id  = params_.base_link_frame;
    tfStmp.header.frame_id = params_.reference_frame;
    tfStmp.header.stamp    = myNow(l->timestamp);
    tf_bc_->sendTransform(tfStmp);

    // 2/2: Publish Odometry msg:
    if (params_.publish_odometry_msgs_from_slam)
    {
        // Convert observation MRPT -> ROS
        nav_msgs::msg::Odometry msg;
        msg.header.stamp    = myNow(l->timestamp);
        msg.header.frame_id = params_.reference_frame;

        mrpt::poses::CPose3DPDFGaussian posePdf;
        posePdf.mean = mrpt::poses::CPose3D(l->pose);
        if (l->cov) posePdf.cov = l->cov.value();

        msg.pose = mrpt::ros2bridge::toROS_Pose(posePdf);

        pubOdo->publish(msg);
    }
}

void BridgeROS2::timerPubMap()
{
    using namespace std::string_literals;

    // get a copy of the data minimizing the time owning the mutex:
    std::map<std::string /*map_name*/, mola::MapSourceBase::MapUpdate> m;
    {
        auto lck  = mrpt::lockHelper(lastLocMapMtx_);
        m         = std::move(lastMaps_);
        lastMaps_ = {};
    }
    if (m.empty()) return;

    MRPT_LOG_DEBUG_STREAM("New map layers (" << m.size() << ") received");

    for (const auto& [layerName, mu] : m)
    {
        const std::string mapTopic =
            (mu.method.empty() ? "slam"s : mu.method) + "/"s + layerName;

        // Reuse code for point cloud observations: build a "fake" observation:
        mrpt::obs::CObservationPointCloud obs;
        obs.sensorLabel = mapTopic;
        obs.pointcloud =
            std::dynamic_pointer_cast<mrpt::maps::CPointsMap>(mu.map);
        if (!obs.pointcloud)
        {
            MRPT_LOG_WARN_STREAM(
                "Do not know how to publish map layer '"
                << layerName << "' of type '"
                << mu.map->GetRuntimeClass()->className << "'");
            continue;
        }

        internalOn(obs, false /*no tf*/, mu.reference_frame);
    }
}

void BridgeROS2::internalAnalyzeTopicsToSubscribe(
    const mrpt::containers::yaml& ds_subscribe)
{
    using namespace std::string_literals;

    // Should be used to subscribe to sensor topics, per REP-2003:
    // https://ros.org/reps/rep-2003.html
    const rclcpp::QoS qos = rclcpp::SensorDataQoS();

    for (const auto& topicItem : ds_subscribe.asSequence())
    {
        const auto topic = mrpt::containers::yaml(topicItem);

        ENSURE_YAML_ENTRY_EXISTS(topic, "topic");
        ENSURE_YAML_ENTRY_EXISTS(topic, "msg_type");
        ENSURE_YAML_ENTRY_EXISTS(topic, "output_sensor_label");

        const auto topic_name = topic["topic"].as<std::string>();
        const auto type       = topic["msg_type"].as<std::string>();
        const auto output_sensor_label =
            topic["output_sensor_label"].as<std::string>();

        MRPT_LOG_DEBUG_STREAM(
            "Creating ros2 subscriber for topic='" << topic_name << "' ("
                                                   << type << ")");

        // Optional: fixed sensorPose (then ignores/don't need "tf" data):
        std::optional<mrpt::poses::CPose3D> fixedSensorPose;
        if (topic.has("fixed_sensor_pose") &&
            (!topic.has("use_fixed_sensor_pose") ||
             topic["use_fixed_sensor_pose"].as<bool>()))
        {
            fixedSensorPose = mrpt::poses::CPose3D::FromString(
                "["s + topic["fixed_sensor_pose"].as<std::string>() + "]"s);
        }

        if (type == "PointCloud2")
        {
            subsPointCloud_.emplace_back(
                rosNode_->create_subscription<sensor_msgs::msg::PointCloud2>(
                    topic_name, qos,
                    [this, output_sensor_label,
                     fixedSensorPose](const sensor_msgs::msg::PointCloud2& o) {
                        this->callbackOnPointCloud2(
                            o, output_sensor_label, fixedSensorPose);
                    }));
        }
        else if (type == "LaserScan")
        {
            subsLaserScan_.emplace_back(
                rosNode_->create_subscription<sensor_msgs::msg::LaserScan>(
                    topic_name, qos,
                    [this, output_sensor_label,
                     fixedSensorPose](const sensor_msgs::msg::LaserScan& o) {
                        this->callbackOnLaserScan(
                            o, output_sensor_label, fixedSensorPose);
                    }));
        }
        else if (type == "Imu")
        {
            subsImu_.emplace_back(
                rosNode_->create_subscription<sensor_msgs::msg::Imu>(
                    topic_name, qos,
                    [this, output_sensor_label,
                     fixedSensorPose](const sensor_msgs::msg::Imu& o) {
                        this->callbackOnImu(
                            o, output_sensor_label, fixedSensorPose);
                    }));
        }
        else if (type == "NavSatFix")
        {
            subsGNNS_.emplace_back(
                rosNode_->create_subscription<sensor_msgs::msg::NavSatFix>(
                    topic_name, qos,
                    [this, output_sensor_label,
                     fixedSensorPose](const sensor_msgs::msg::NavSatFix& o) {
                        this->callbackOnNavSatFix(
                            o, output_sensor_label, fixedSensorPose);
                    }));
        }
        else if (type == "Odometry")
        {
            subsOdometry_.emplace_back(
                rosNode_->create_subscription<nav_msgs::msg::Odometry>(
                    topic_name, qos,
                    [this,
                     output_sensor_label](const nav_msgs::msg::Odometry& o) {
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
}

void BridgeROS2::publishStaticTFs()
{
    if (!params_.base_footprint_frame.empty())
    {
        const tf2::Transform transform = mrpt::ros2bridge::toROS_tfTransform(
            params_.base_footprint_to_base_link_tf);

        geometry_msgs::msg::TransformStamped tfStmp;

        tfStmp.transform       = tf2::toMsg(transform);
        tfStmp.child_frame_id  = params_.base_link_frame;
        tfStmp.header.frame_id = params_.base_footprint_frame;
        tfStmp.header.stamp    = myNow(mrpt::Clock::now());

        tf_static_bc_->sendTransform(tfStmp);
        // tf_bc_->sendTransform(tfStmp);
    }
}
