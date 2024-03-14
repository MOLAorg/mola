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
#pragma once

// MOLA virtual interfaces:
#include <mola_kernel/interfaces/ExecutableBase.h>
#include <mola_kernel/interfaces/LocalizationSourceBase.h>
#include <mola_kernel/interfaces/MapSourceBase.h>
#include <mola_kernel/interfaces/RawDataConsumer.h>
#include <mola_kernel/interfaces/RawDataSourceBase.h>

// MRPT:
#include <mrpt/obs/CObservationImage.h>
#include <mrpt/obs/CObservationPointCloud.h>

// ROS & others
#include <tf2_ros/transform_broadcaster.h>

#include <nav_msgs/msg/odometry.hpp>
#include <optional>

namespace mola
{
/** Bridge: MOLA ==> ROS2
 *
 * This class can be used to:
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
 * Another bridge in the inverse direction ROS2=>MOLA is provided in
 * the package mola_input_ros2.
 *
 * \ingroup mola_output_ros2_grp
 */
class OutputROS2 : public mola::ExecutableBase, public mola::RawDataConsumer
{
    DEFINE_MRPT_OBJECT(OutputROS2, mola)

   public:
    OutputROS2();
    ~OutputROS2() override;

    // See docs in base class
    void initialize(const Yaml& cfg) override;
    void spinOnce() override;

    // RawDataConsumer implementation:
    void onNewObservation(const CObservation::Ptr& o) override;

   private:
    std::thread rosNodeThread_;

    struct Params
    {
        /// tf frame name with respect to sensor poses are measured:
        std::string base_link_frame = "base_link";

        /// tf frame name for odometry's frame of reference:
        std::string reference_frame = "map";

        bool publish_odometry_msgs_from_slam = true;

        bool publish_tf_from_robot_pose_observations = true;

        /// If true, the original dataset timestamps will be used to publish.
        /// Otherwise, the wallclock time will be used.
        bool publish_in_sim_time = false;

        double period_publish_new_localization = 0.2;  // [s]
        double period_publish_new_map          = 5.0;  // [s]

        double period_check_new_mola_subs = 1.0;  // [s]

        unsigned int publisher_history_len = 10;
    };

    Params params_;

    // Yaml is NOT a reference on purpose.
    void ros_node_thread_main(Yaml cfg);

    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_bc_;

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

    void internalOn(
        const mrpt::obs::CObservationPointCloud& obs,
        bool publishSensorPoseToTF, const std::string& sSensorFrameId);
};

}  // namespace mola
