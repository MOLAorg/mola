/* -------------------------------------------------------------------------
 *   A Modular Optimization framework for Localization and mApping  (MOLA)
 * Copyright (C) 2018-2024 Jose Luis Blanco, University of Almeria
 * See LICENSE for license information.
 * ------------------------------------------------------------------------- */
/**
 * @file   BridgeROS2.h
 * @brief  Bridge between MOLA-ROS2
 * @author Jose Luis Blanco Claraco
 * @date   Sep 7, 2023
 */
#pragma once

// MOLA virtual interfaces:
#include <mola_kernel/interfaces/ExecutableBase.h>
#include <mola_kernel/interfaces/LocalizationSourceBase.h>
#include <mola_kernel/interfaces/MapSourceBase.h>
#include <mola_kernel/interfaces/RawDataConsumer.h>
#include <mola_kernel/interfaces/RawDataSourceBase.h>

// MRPT:
#include <mrpt/obs/CObservationGPS.h>
#include <mrpt/obs/CObservationImage.h>
#include <mrpt/obs/CObservationPointCloud.h>

// ROS & others
#include <tf2_ros/buffer.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>

#include <nav_msgs/msg/odometry.hpp>
#include <optional>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

namespace mola
{
/** A bridge to read sensor observations from ROS 2 topics and forwarding them
 * to other MOLA subsystems, and to publish MOLA topics, localization, and maps
 * to ROS 2.
 *
 * ## Bridge ROS2=>MOLA
 * The MOLA nodelet execution rate (Hz) determines the rate of publishing
 * odometry observations, if enabled.
 * All other subscribed sensors are forwarded to the MOLA system without delay
 * as they are received from ROS.
 *
 * ## Bridge MOLA=>ROS2
 *  - Publish datasets from any of the MOLA dataset input modules to ROS 2.
 *  - Expose the results of a MOLA SLAM/odometry system to the rest of a ROS 2
 *    system.
 *
 * Every few seconds, this module will peek the MOLA subsystem for modules of
 * these types, and subscribes to all their outputs:
 *  - mola::RawDataSourceBase: For all their sensor readings (as
 *    children classes of mrpt::obs::CObservation). See list of mappings below.
 *  - mola::LocalizationSourceBase: For SLAM/odometry method outputs.
 *  - mola::MapSourceBase: For SLAM/odometry metric maps. Point maps are
 *    published as sensor_msgs/PointCloud2
 *
 * The following mappings are currently implemented between MOLA=>ROS2:
 *  - mrpt::obs::CObservation2DRangeScan  ==> sensor_msgs/LaserScan
 *  - mrpt::obs::CObservationPointCloud   ==> sensor_msgs/PointCloud2
 *  - mrpt::obs::CObservationImage        ==> sensor_msgs/Image
 *  - mrpt::obs::CObservationRobotPose    ==> nav_msgs/Odometry with
 *    frame_id=the robot (default: `base_link`) and parent frame the global one
 *    (default:`map`, may make sense to change to `odom` depending on your
 *    application). It will be also published as a transform to `/tf`.
 *
 * For all sensors, the mrpt::obs::CObservation::sensorPose will be also
 * broadcasted to `/tf` with respect to the vehicle frame (default:
 * `base_link`).
 *
 * See example launch files in `mola_demos` and in `mola_lidar_odometry`.
 *
 * \ingroup mola_bridge_ros2_grp
 */
class BridgeROS2 : public RawDataSourceBase, public mola::RawDataConsumer
{
    DEFINE_MRPT_OBJECT(BridgeROS2, mola)

   public:
    BridgeROS2();
    ~BridgeROS2() override;

    // See docs in base class
    void spinOnce() override;

    // RawDataConsumer implementation:
    void onNewObservation(const CObservation::Ptr& o) override;

   protected:
    // See docs in base class
    void initialize_rds(const Yaml& cfg) override;

   private:
    std::thread rosNodeThread_;

    struct Params
    {
        /// tf frame name with respect to sensor poses are measured:
        std::string base_link_frame = "base_link";

        /// If not empty, the node will broadcast a static /tf from base_link to
        /// base_footprint with the TF base_footprint_to_base_link_tf at start
        /// up.
        /// Normally: "base_footprint"
        std::string base_footprint_frame;  // Disabled by default

        /// YAML format: "[x y z yaw pitch roll]" (meters & degrees)
        mrpt::math::TPose3D base_footprint_to_base_link_tf = {0, 0, 0, 0, 0, 0};

        /// tf frame name for odometry's frame of reference:
        std::string odom_frame = "odom";

        /// tf frame name for odometry's frame of reference:
        std::string reference_frame = "map";

        bool forward_ros_tf_as_mola_odometry_observations = false;
        bool publish_odometry_msgs_from_slam              = true;

        bool publish_tf_from_robot_pose_observations = true;

        /// If true, the original dataset timestamps will be used to publish.
        /// Otherwise, the wallclock time will be used.
        bool publish_in_sim_time = false;

        double period_publish_new_localization = 0.2;  // [s]
        double period_publish_new_map          = 5.0;  // [s]
        double period_publish_static_tfs       = 1.0;  // [s]

        double period_check_new_mola_subs = 1.0;  // [s]

        int wait_for_tf_timeout_milliseconds = 100;
    };

    Params params_;

    // Yaml is NOT a reference on purpose.
    void ros_node_thread_main(Yaml cfg);

    std::shared_ptr<tf2_ros::Buffer>            tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

    std::shared_ptr<tf2_ros::TransformBroadcaster>       tf_bc_;
    std::shared_ptr<tf2_ros::StaticTransformBroadcaster> tf_static_bc_;

    rclcpp::Clock::SharedPtr ros_clock_;
    std::mutex               ros_clock_mtx_;

    std::shared_ptr<rclcpp::Node> rosNode_;
    std::mutex                    rosNodeMtx_;

    /// Returns a copy of the shared_ptr to the ROS 2 node, or empty if not
    /// initialized yet.
    std::shared_ptr<rclcpp::Node> rosNode()
    {
        auto lck = mrpt::lockHelper(rosNodeMtx_);
        return rosNode_;
    }
    std::vector<rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr>
        subsPointCloud_;

    std::vector<rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr>
        subsLaserScan_;

    std::vector<rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr>
        subsOdometry_;

    std::vector<rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr>
        subsImu_;

    std::vector<rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr>
        subsGNNS_;

    void callbackOnPointCloud2(
        const sensor_msgs::msg::PointCloud2&       o,
        const std::string&                         outSensorLabel,
        const std::optional<mrpt::poses::CPose3D>& fixedSensorPose);

    void callbackOnLaserScan(
        const sensor_msgs::msg::LaserScan& o, const std::string& outSensorLabel,
        const std::optional<mrpt::poses::CPose3D>& fixedSensorPose);

    void callbackOnImu(
        const sensor_msgs::msg::Imu& o, const std::string& outSensorLabel,
        const std::optional<mrpt::poses::CPose3D>& fixedSensorPose);

    void callbackOnNavSatFix(
        const sensor_msgs::msg::NavSatFix& o, const std::string& outSensorLabel,
        const std::optional<mrpt::poses::CPose3D>& fixedSensorPose);

    void callbackOnOdometry(
        const nav_msgs::msg::Odometry& o, const std::string& outSensorLabel);

    bool waitForTransform(
        mrpt::poses::CPose3D& des, const std::string& target_frame,
        const std::string& source_frame, const rclcpp::Time& time,
        const int timeoutMilliseconds, bool printErrors);

    void publishOdometry();

    /// Returns either the wallclock "now" (params_.use_sim_time = false)
    /// or the equivalent of the passed argument in ROS 2 format otherwise.
    rclcpp::Time myNow(const mrpt::Clock::time_point& observationStamp);

    struct RosPubs
    {
        /// Map <sensor_label> => publisher
        std::map<
            std::string, rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr>
            pub_poses;

        /// Map <sensor_label> => publisher
        std::map<std::string, rclcpp::PublisherBase::SharedPtr> pub_sensors;

        // rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr
        // pub_markers;
    };

    RosPubs    rosPubs_;
    std::mutex rosPubsMtx_;

    struct MolaSubs
    {
        std::set<mola::RawDataSourceBase::Ptr>                  dataSources;
        std::set<std::shared_ptr<mola::LocalizationSourceBase>> locSources;
        std::set<std::shared_ptr<mola::MapSourceBase>>          mapSources;
    };

    MolaSubs   molaSubs_;
    std::mutex molaSubsMtx_;

    void onNewLocalization(
        const mola::LocalizationSourceBase::LocalizationUpdate& l);

    void onNewMap(const mola::MapSourceBase::MapUpdate& m);

    std::mutex lastLocMapMtx_;
    std::optional<mola::LocalizationSourceBase::LocalizationUpdate> lastLoc_;
    std::map<std::string /*map_name*/, mola::MapSourceBase::MapUpdate>
        lastMaps_;

    void timerPubLocalization();
    void timerPubMap();

    double lastTimeCheckMolaSubs_ = 0;
    void   doLookForNewMolaSubs();

    void internalOn(const mrpt::obs::CObservationImage& obs);
    void internalOn(const mrpt::obs::CObservation2DRangeScan& obs);
    void internalOn(const mrpt::obs::CObservationPointCloud& obs);
    void internalOn(const mrpt::obs::CObservationRobotPose& obs);
    void internalOn(const mrpt::obs::CObservationGPS& obs);

    void internalOn(
        const mrpt::obs::CObservationPointCloud& obs, bool isSensorTopic,
        const std::string& sSensorFrameId);

    void internalAnalyzeTopicsToSubscribe(
        const mrpt::containers::yaml& ds_subscribe);

    void publishStaticTFs();
};

}  // namespace mola
