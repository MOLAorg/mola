/*               _
 _ __ ___   ___ | | __ _
| '_ ` _ \ / _ \| |/ _` | Modular Optimization framework for
| | | | | | (_) | | (_| | Localization and mApping (MOLA)
|_| |_| |_|\___/|_|\__,_| https://github.com/MOLAorg/mola

 Copyright (C) 2018-2025 Jose Luis Blanco, University of Almeria,
                         and individual contributors.
 SPDX-License-Identifier: GPL-3.0
 See LICENSE for full license information.
*/

/**
 * @file   BridgeROS2.h
 * @brief  Bridge between MOLA-ROS2
 * @author Jose Luis Blanco Claraco
 * @date   Sep 7, 2023
 */
#pragma once

// MOLA virtual interfaces:
#include <mola_kernel/Georeferencing.h>
#include <mola_kernel/interfaces/ExecutableBase.h>
#include <mola_kernel/interfaces/LocalizationSourceBase.h>
#include <mola_kernel/interfaces/MapServer.h>
#include <mola_kernel/interfaces/MapSourceBase.h>
#include <mola_kernel/interfaces/RawDataConsumer.h>
#include <mola_kernel/interfaces/RawDataSourceBase.h>
#include <mola_kernel/interfaces/Relocalization.h>

// MRPT:
#include <mrpt/maps/COccupancyGridMap2D.h>
#include <mrpt/obs/CObservationGPS.h>
#include <mrpt/obs/CObservationIMU.h>
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

// MOLA <-> ROS services:
#include <mola_msgs/srv/map_load.hpp>
#include <mola_msgs/srv/map_save.hpp>
#include <mola_msgs/srv/mola_runtime_param_get.hpp>
#include <mola_msgs/srv/mola_runtime_param_set.hpp>
#include <mola_msgs/srv/relocalize_from_state_estimator.hpp>
#include <mola_msgs/srv/relocalize_near_pose.hpp>

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
  void onNewObservation(const CObservation::ConstPtr& o) override;

 protected:
  // See docs in base class
  void initialize_rds(const Yaml& cfg) override;

 private:
  std::thread rosNodeThread_;

  struct Params
  {
    /// tf frame name with respect to sensor poses are measured, and also used for publishing
    /// SLAM/localization results (read below).
    std::string base_link_frame = "base_link";

    /// If not empty, the node will broadcast a static /tf from base_footprint to
    /// base_link with the TF base_footprint_to_base_link_tf at start
    /// up.
    /// Normally: "base_footprint"
    std::string base_footprint_frame;  // Disabled by default

    /// YAML format: "[x y z yaw pitch roll]" (meters & degrees)
    mrpt::math::TPose3D base_footprint_to_base_link_tf = {0, 0, 0, 0, 0, 0};

    /// Used for:
    /// (a) importing odometry to MOLA if ``forward_ros_tf_as_mola_odometry_observations=true``
    /// (b) querying ``${odom_frame} => ${base_link_frame}`` when
    ///     ``publish_localization_following_rep105=true``.
    std::string odom_frame = "odom";

    /// tf frame used for:
    /// (a) See ``publish_tf_from_robot_pose_observations``
    /// (b) To follow REP105 (``publish_localization_following_rep105``), this must match
    ///     the frame used as reference in the LocalizationSource (e.g. mola_lidar_odometry)
    std::string reference_frame = "map";

    /// How to publish localization to /tf:
    /// - ``false``(direct mode): reference_frame ("map") -> base_link ("base_link")
    ///   Note that reference_frame in this case comes from the localization source module
    ///   (e.g. mola_lida_odometry), it is not configured here.
    ///
    ///  - ``true`` (indirect mode), following ROS [REP
    ///  105](https://ros.org/reps/rep-0105.html):
    ///   ``map -> odom``  (such as "map -> odom -> base_link" = "map -> base_link")
    bool publish_localization_following_rep105 = true;

    /// If enabled, during spinOnce(), the tf ``${odom_frame} => ${base_link_frame}`` will
    /// be queried and forwarded as an `CObservationOdometry` reading to the MOLA subsystem:
    bool forward_ros_tf_as_mola_odometry_observations = false;

    /// If enabled, SLAM/Localization results will be published as nav_msgs/Odometry messages.
    bool publish_odometry_msgs_from_slam = true;

    // Which source will be forwarded (empty=any)
    std::string publish_odometry_msgs_from_slam_source = {};

    /// If enabled, SLAM/Localization results will be published as tf messages, for frames
    /// according to explained above for `publish_localization_following_rep105`.
    bool publish_tf_from_slam = true;

    // Which source will be forwarded (empty=any)
    std::string publish_tf_from_slam_source = {};

    /// If enabled, robot pose observations (typically, ground truth from datasets), will be
    /// forwarded to ROS as /tf messages: ``${reference_frame} => ${base_link}``
    bool publish_tf_from_robot_pose_observations = true;

    std::string relocalize_from_topic = "/initialpose";  //!< Default in RViz

    /// If true, the original dataset timestamps will be used to publish.
    /// Otherwise, the wallclock time will be used.
    bool publish_in_sim_time = false;

    double period_publish_new_localization = 0.2;  // [s]
    double period_publish_new_map          = 5.0;  // [s]
    double period_publish_static_tfs       = 1.0;  // [s]
    double period_publish_diagnostics      = 1.0;  // [s]

    double period_check_new_mola_subs = 1.0;  // [s]

    int wait_for_tf_timeout_milliseconds = 100;

    std::string georef_map_reference_frame = "map";
    std::string georef_map_utm_frame       = "utm";
    std::string georef_map_enu_frame       = "enu";
  };

  Params params_;

  // Yaml is NOT a reference on purpose.
  void ros_node_thread_main(Yaml cfg);

  // std::shared_ptr<tf2_ros::Buffer>            tf_buffer_;
  std::shared_ptr<tf2::BufferCore>            tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  std::shared_ptr<tf2_ros::TransformBroadcaster>       tf_bc_;
  std::shared_ptr<tf2_ros::StaticTransformBroadcaster> tf_static_bc_;
  std::mutex                                           ros_tf_bc_mtx_;

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
  std::vector<rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr> subsPointCloud_;

  std::vector<rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr> subsLaserScan_;

  std::vector<rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr> subsOdometry_;

  std::vector<rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr> subsImu_;

  std::vector<rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr> subsGNSS_;

  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr subInitPose_;

  void callbackOnPointCloud2(
      const sensor_msgs::msg::PointCloud2& o, const std::string& outSensorLabel,
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

  void callbackOnOdometry(const nav_msgs::msg::Odometry& o, const std::string& outSensorLabel);

  void callbackOnRelocalizeTopic(const geometry_msgs::msg::PoseWithCovarianceStamped& o);

  bool waitForTransform(
      mrpt::poses::CPose3D& des, const std::string& target_frame, const std::string& source_frame,
      bool printErrors);

  void importRosOdometryToMOLA();

  /// Returns either the wallclock "now" (params_.use_sim_time = false)
  /// or the equivalent of the passed argument in ROS 2 format otherwise.
  rclcpp::Time myNow(const mrpt::Clock::time_point& observationStamp);

  /// Generic Map <topic> => publisher
  std::map<std::string, rclcpp::PublisherBase::SharedPtr> rosPubs_;
  std::recursive_mutex                                    rosPubsMtx_;

  /// Gets or creates (upon first use) a publisher for a given type:
  template <typename MSG_TYPE>
  [[nodiscard]] auto get_publisher(const std::string& topic, const rclcpp::QoS& qos)
      -> std::shared_ptr<rclcpp::Publisher<MSG_TYPE>>
  {
    auto lck = mrpt::lockHelper(rosPubsMtx_);

    // Create the publisher the first time an observation arrives:
    const bool is_1st_pub = rosPubs_.find(topic) == rosPubs_.end();
    auto&      pub        = rosPubs_[topic];

    if (is_1st_pub)
    {
      pub = rosNode()->create_publisher<MSG_TYPE>(topic, qos);
    }
    lck.unlock();

    auto ret = std::dynamic_pointer_cast<rclcpp::Publisher<MSG_TYPE>>(pub);
    ASSERT_(ret);
    return ret;
  }

  struct MolaSubs
  {
    // MOLA subscribers:
    std::set<mola::RawDataSourceBase::Ptr>                  dataSources;
    std::set<std::shared_ptr<mola::LocalizationSourceBase>> locSources;
    std::set<std::shared_ptr<mola::MapSourceBase>>          mapSources;
    std::set<std::shared_ptr<mola::Relocalization>>         relocalization;
    std::set<std::shared_ptr<mola::MapServer>>              mapServers;
  };

  MolaSubs   molaSubs_;
  std::mutex molaSubsMtx_;

  // ROS services:
  rclcpp::Service<mola_msgs::srv::RelocalizeFromStateEstimator>::SharedPtr srvRelocSE_;
  rclcpp::Service<mola_msgs::srv::RelocalizeNearPose>::SharedPtr           srvRelocPose_;
  rclcpp::Service<mola_msgs::srv::MapLoad>::SharedPtr                      srvMapLoad_;
  rclcpp::Service<mola_msgs::srv::MapSave>::SharedPtr                      srvMapSave_;
  rclcpp::Service<mola_msgs::srv::MolaRuntimeParamGet>::SharedPtr          srvParamGet_;
  rclcpp::Service<mola_msgs::srv::MolaRuntimeParamSet>::SharedPtr          srvParamSet_;

  void service_relocalize_from_se(
      const std::shared_ptr<mola_msgs::srv::RelocalizeFromStateEstimator::Request> request,
      std::shared_ptr<mola_msgs::srv::RelocalizeFromStateEstimator::Response>      response);

  void service_relocalize_near_pose(
      const std::shared_ptr<mola_msgs::srv::RelocalizeNearPose::Request> request,
      std::shared_ptr<mola_msgs::srv::RelocalizeNearPose::Response>      response);

  void service_map_load(
      const std::shared_ptr<mola_msgs::srv::MapLoad::Request> request,
      std::shared_ptr<mola_msgs::srv::MapLoad::Response>      response);

  void service_map_save(
      const std::shared_ptr<mola_msgs::srv::MapSave::Request> request,
      std::shared_ptr<mola_msgs::srv::MapSave::Response>      response);

  void service_param_get(
      const std::shared_ptr<mola_msgs::srv::MolaRuntimeParamGet::Request> request,
      std::shared_ptr<mola_msgs::srv::MolaRuntimeParamGet::Response>      response);

  void service_param_set(
      const std::shared_ptr<mola_msgs::srv::MolaRuntimeParamSet::Request> request,
      std::shared_ptr<mola_msgs::srv::MolaRuntimeParamSet::Response>      response);

  void onNewLocalization(const mola::LocalizationSourceBase::LocalizationUpdate& l);

  void onNewMap(const mola::MapSourceBase::MapUpdate& m);

  std::mutex                                                              lastLocMapMtx_;
  std::vector<mola::LocalizationSourceBase::LocalizationUpdate>           lastLocUpdates_;
  std::multimap<std::string /*map_name*/, mola::MapSourceBase::MapUpdate> lastMaps_;

  void timerPubLocalization();
  void timerPubMap();

  double lastTimeCheckMolaSubs_ = 0;
  void   doLookForNewMolaSubs();

  void internalOn(const mrpt::obs::CObservationImage& obs);
  void internalOn(const mrpt::obs::CObservation2DRangeScan& obs);
  void internalOn(const mrpt::obs::CObservationPointCloud& obs);
  void internalOn(const mrpt::obs::CObservationRobotPose& obs);
  void internalOn(const mrpt::obs::CObservationGPS& obs);
  void internalOn(const mrpt::obs::CObservationIMU& obs);

  void internalOn(
      const mrpt::obs::CObservationPointCloud& obs, bool isSensorTopic,
      const std::string& sSensorFrameId);

  void internalPublishGridMap(
      const mrpt::maps::COccupancyGridMap2D& m, const std::string& sMapTopicName,
      const std::string& sReferenceFrame, const mrpt::Clock::time_point& timestamp);

  void internalAnalyzeTopicsToSubscribe(const mrpt::containers::yaml& ds_subscribe);

  void publishStaticTFs();
  void publishDiagnostics();
  void publishMetricMapGeoreferencingData(
      const mola::Georeferencing& g, const std::string& georefTopic);
};

}  // namespace mola
