/* -------------------------------------------------------------------------
 *   A Modular Optimization framework for Localization and mApping  (MOLA)
 * Copyright (C) 2018-2024 Jose Luis Blanco, University of Almeria
 * See LICENSE for license information.
 * ------------------------------------------------------------------------- */
/**
 * @file   OutputROS2.h
 * @brief  Bridge MOLA->ROS2
 * @author Jose Luis Blanco Claraco
 * @date   Mar 7, 2024
 */

/** \defgroup mola_output_ros2_grp mola_output_ros2_grp.
 * Bridge: MOLA ==> ROS2
 *
 */

// MOLA interfaces:
#include <mola_kernel/pretty_print_exception.h>

// me:
#include <mola_output_ros2/OutputROS2.h>

// MOLA/MRPT:
#include <mola_yaml/yaml_helpers.h>
#include <mrpt/containers/yaml.h>
#include <mrpt/core/initializer.h>
#include <mrpt/maps/CPointsMapXYZI.h>
#include <mrpt/maps/CPointsMapXYZIRT.h>
#include <mrpt/maps/CSimplePointsMap.h>
#include <mrpt/obs/CObservation2DRangeScan.h>
#include <mrpt/obs/CObservationOdometry.h>
#include <mrpt/obs/CObservationPointCloud.h>
#include <mrpt/obs/CObservationRobotPose.h>
#include <mrpt/ros2bridge/image.h>
#include <mrpt/ros2bridge/laser_scan.h>
#include <mrpt/ros2bridge/point_cloud2.h>
#include <mrpt/ros2bridge/pose.h>
#include <mrpt/ros2bridge/time.h>
#include <mrpt/system/filesystem.h>

// ROS 2:
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/node.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

using namespace mola;

// arguments: class_name, parent_class, class namespace
IMPLEMENTS_MRPT_OBJECT(OutputROS2, mola::ExecutableBase, mola)

MRPT_INITIALIZER(do_register_OutputROS2) { MOLA_REGISTER_MODULE(OutputROS2); }

OutputROS2::OutputROS2()
{
    mrpt::system::COutputLogger::setLoggerName("OutputROS2");
}

OutputROS2::~OutputROS2()
{
    try
    {
        rclcpp::shutdown();
        if (rosNodeThread_.joinable()) rosNodeThread_.join();
    }
    catch (const std::exception& e)
    {
        std::cerr << "[~OutputROS2] Exception in destructor:\n" << e.what();
    }
}

// The ROS node starts with MOLA::initialize() and ends with its dtor
void OutputROS2::ros_node_thread_main([[maybe_unused]] Yaml cfg)
{
    using std::placeholders::_1;
    using namespace std::string_literals;

    const char* NODE_NAME = "mola_output_ros2";

    try
    {
        const int         argc    = 1;
        char const* const argv[2] = {NODE_NAME, nullptr};

        // Initialize ROS:
        rclcpp::init(argc, argv);

        auto lckNode = mrpt::lockHelper(rosNodeMtx_);

        rosNode_ = std::make_shared<rclcpp::Node>(NODE_NAME);

        lckNode.unlock();

        {
            auto lck   = mrpt::lockHelper(ros_clock_mtx_);
            ros_clock_ = rosNode_->get_clock();
        }

        tf_bc_ = std::make_shared<tf2_ros::TransformBroadcaster>(rosNode_);

        auto timerLoc = rosNode_->create_wall_timer(
            std::chrono::microseconds(static_cast<unsigned int>(
                1e6 * params_.period_publish_new_localization)),
            std::bind(&OutputROS2::timerPubLocalization, this));

        auto timerMap = rosNode_->create_wall_timer(
            std::chrono::microseconds(static_cast<unsigned int>(
                1e6 * params_.period_publish_new_map)),
            std::bind(&OutputROS2::timerPubMap, this));

        // Spin:
        rclcpp::spin(rosNode_);
        rclcpp::shutdown();
    }
    catch (const std::exception& e)
    {
        mola::pretty_print_exception(e, "OutputROS2::ros_node_thread_main");
    }
}

void OutputROS2::initialize(const Yaml& c)
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
    YAML_LOAD_OPT(params_, reference_frame, std::string);
    YAML_LOAD_OPT(params_, publish_odometry_msgs_from_slam, bool);
    YAML_LOAD_OPT(params_, publish_in_sim_time, bool);
    YAML_LOAD_OPT(params_, period_publish_new_localization, double);
    YAML_LOAD_OPT(params_, period_publish_new_map, double);
    YAML_LOAD_OPT(params_, publish_tf_from_robot_pose_observations, bool);

    // Launch ROS node:
    rosNodeThread_ =
        std::thread(&OutputROS2::ros_node_thread_main, this, cfgCopy);

    MRPT_END
}  // end initialize()

void OutputROS2::spinOnce()
{
    using mrpt::system::timeDifference;

    MRPT_START
    ProfilerEntry tleg(profiler_, "spinOnce");

    // Check for new mola data sources?
    if (mrpt::Clock::nowDouble() - lastTimeCheckMolaSubs_ >
        params_.period_check_new_mola_subs)
    {
        lastTimeCheckMolaSubs_ = mrpt::Clock::nowDouble();
        doLookForNewMolaSubs();
    }

    MRPT_END
}

void OutputROS2::onNewObservation(const CObservation::Ptr& o)
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
    else
    {
        MRPT_LOG_THROTTLE_WARN_FMT(
            5.0,
            "Do not know how to publish to ROS an observation of type '%s' "
            "with sensorLabel='%s'",
            o->GetRuntimeClass()->className, o->sensorLabel.c_str());
    }
}

void OutputROS2::internalOn(const mrpt::obs::CObservationImage& obs)
{
    auto lck = mrpt::lockHelper(rosPubsMtx_);

    // Create the publisher the first time an observation arrives:
    const bool is_1st_pub = rosPubs_.pub_sensors.find(obs.sensorLabel) ==
                            rosPubs_.pub_sensors.end();
    auto& pub = rosPubs_.pub_sensors[obs.sensorLabel];

    if (is_1st_pub)
    {
        pub = rosNode()->create_publisher<sensor_msgs::msg::Image>(
            obs.sensorLabel, params_.publisher_history_len);
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

void OutputROS2::internalOn(const mrpt::obs::CObservation2DRangeScan& obs)
{
    auto lck = mrpt::lockHelper(rosPubsMtx_);

    // Create the publisher the first time an observation arrives:
    const bool is_1st_pub = rosPubs_.pub_sensors.find(obs.sensorLabel) ==
                            rosPubs_.pub_sensors.end();
    auto& pub = rosPubs_.pub_sensors[obs.sensorLabel];

    if (is_1st_pub)
    {
        pub = rosNode()->create_publisher<sensor_msgs::msg::LaserScan>(
            obs.sensorLabel, params_.publisher_history_len);
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

void OutputROS2::internalOn(const mrpt::obs::CObservationPointCloud& obs)
{
    internalOn(
        obs, true /*it is a real sensor, publish its /tf*/,
        obs.sensorLabel /* frame_id */);
}

void OutputROS2::internalOn(
    const mrpt::obs::CObservationPointCloud& obs, bool publishSensorPoseToTF,
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
        pubPts = rosNode()->create_publisher<sensor_msgs::msg::PointCloud2>(
            lbPoints, params_.publisher_history_len);
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
    if (publishSensorPoseToTF)
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

void OutputROS2::internalOn(const mrpt::obs::CObservationRobotPose& obs)
{
    auto lck = mrpt::lockHelper(rosPubsMtx_);

    ASSERT_(!obs.sensorLabel.empty());

    // Create the publisher the first time an observation arrives:
    const bool is_1st_pub = rosPubs_.pub_sensors.find(obs.sensorLabel) ==
                            rosPubs_.pub_sensors.end();
    auto& pub = rosPubs_.pub_sensors[obs.sensorLabel];

    if (is_1st_pub)
    {
        pub = rosNode()->create_publisher<nav_msgs::msg::Odometry>(
            obs.sensorLabel, params_.publisher_history_len);
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

void OutputROS2::doLookForNewMolaSubs()
{
    auto lck = mrpt::lockHelper(molaSubsMtx_);

    // RawDataSourceBase:
    auto listRDS = this->findService<mola::RawDataSourceBase>();
    for (auto& module : listRDS)
    {
        auto rds = std::dynamic_pointer_cast<mola::RawDataSourceBase>(module);
        ASSERT_(rds);

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

rclcpp::Time OutputROS2::myNow(const mrpt::Clock::time_point& observationStamp)
{
    if (params_.publish_in_sim_time)
        return mrpt::ros2bridge::toROS(observationStamp);
    else
        return mrpt::ros2bridge::toROS(mrpt::Clock::now());
}

void OutputROS2::onNewLocalization(
    const mola::LocalizationSourceBase::LocalizationUpdate& l)
{
    auto lck = mrpt::lockHelper(lastLocMapMtx_);

    lastLoc_ = l;
}

void OutputROS2::onNewMap(const mola::MapSourceBase::MapUpdate& m)
{
    auto lck = mrpt::lockHelper(lastLocMapMtx_);

    lastMaps_[m.map_name] = m;
}

void OutputROS2::timerPubLocalization()
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
            locLabel, params_.publisher_history_len);
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

void OutputROS2::timerPubMap()
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
