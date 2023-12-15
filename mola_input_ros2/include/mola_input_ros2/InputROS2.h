/* -------------------------------------------------------------------------
 *   A Modular Optimization framework for Localization and mApping  (MOLA)
 * Copyright (C) 2018-2023 Jose Luis Blanco, University of Almeria
 * See LICENSE for license information.
 * ------------------------------------------------------------------------- */
/**
 * @file   InputROS2.h
 * @brief  RawDataSource from ROS 2 topics
 * @author Jose Luis Blanco Claraco
 * @date   Sep 7, 2023
 */
#pragma once

#include <mola_kernel/interfaces/RawDataSourceBase.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <nav_msgs/msg/odometry.hpp>
#include <optional>
#include <sensor_msgs/msg/point_cloud2.hpp>

namespace mola
{
/**
 *
 * The MOLA nodelet execution rate (Hz) determines the rate of publishing
 * odometry observations, if enabled.
 * All other subscribed sensors are forwarded to the MOLA system without delay
 * as they are received from ROS.
 *
 * \ingroup mola_input_ros2_grp */
class InputROS2 : public RawDataSourceBase
{
    DEFINE_MRPT_OBJECT(InputROS2, mola)

   public:
    InputROS2();
    ~InputROS2() override = default;

    // See docs in base class
    void initialize(const Yaml& cfg) override;
    void spinOnce() override;

   private:
    std::thread rosNodeThread_;

    struct Params
    {
        /// tf frame name with respect to sensor poses are measured:
        std::string base_link_frame = "base_footprint";

        /// tf frame name for odometry's frame of reference:
        std::string odom_frame = "odom";

        bool publish_odometry_from_tf = false;

        int wait_for_tf_timeout_milliseconds = 100;
    };

    Params params_;

    // Yaml is NOT a reference on purpose.
    void ros_node_thread_main(Yaml cfg);

    std::shared_ptr<tf2_ros::Buffer>            tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

    rclcpp::Clock::SharedPtr ros_clock_;
    std::mutex               ros_clock_mtx_;

    std::vector<rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr>
        subsPointCloud_;

    std::vector<rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr>
        subsOdometry_;

    void callbackOnPointCloud2(
        const sensor_msgs::msg::PointCloud2&       o,
        const std::string&                         outSensorLabel,
        const std::optional<mrpt::poses::CPose3D>& fixedSensorPose);

    void callbackOnOdometry(
        const nav_msgs::msg::Odometry& o, const std::string& outSensorLabel);

    bool waitForTransform(
        mrpt::poses::CPose3D& des, const std::string& target_frame,
        const std::string& source_frame, const rclcpp::Time& time,
        const int timeoutMilliseconds, bool printErrors);

    void publishOdometry();
};

}  // namespace mola
