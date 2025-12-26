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
#include <mola_kernel/version.h>
#include <mola_yaml/yaml_helpers.h>
#include <mrpt/containers/yaml.h>
#include <mrpt/core/initializer.h>
#include <mrpt/maps/CGenericPointsMap.h>
#include <mrpt/maps/COccupancyGridMap2D.h>
#include <mrpt/maps/CSimplePointsMap.h>
#include <mrpt/maps/CVoxelMap.h>
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
#include <mrpt/ros2bridge/map.h>
#include <mrpt/ros2bridge/point_cloud2.h>
#include <mrpt/ros2bridge/pose.h>
#include <mrpt/ros2bridge/time.h>
#include <mrpt/system/filesystem.h>
#include <mrpt/topography/conversions.h>
#include <mrpt/version.h>

#if MRPT_VERSION < 0x030000  // Support deprecated classes for mrpt < 3.0.0
#include <mrpt/maps/CPointsMapXYZI.h>
#include <mrpt/maps/CPointsMapXYZIRT.h>
#endif

// Other mrpt pkgs:
#include <mrpt_nav_interfaces/msg/georeferencing_metadata.hpp>

// ROS 2:
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/node.hpp>
#include <std_msgs/msg/float32.hpp>
#include <std_msgs/msg/string.hpp>
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
    if (rosNodeThread_.joinable())
    {
      rosNodeThread_.join();
    }
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
    // build argc/argv for ROS:
    std::vector<std::string> rosArgs = {NODE_NAME};
    if (cfg.has("ros_args"))
    {
      ASSERT_(cfg["ros_args"].isSequence());
      for (const auto& p : cfg["ros_args"].asSequenceRange())
      {
        rosArgs.push_back(p.as<std::string>());
      }
    }

    // Convert to good old C (argc,argv):
    std::vector<const char*> rosArgsC;
    rosArgsC.reserve(rosArgs.size());
    for (const auto& s : rosArgs)
    {
      rosArgsC.push_back(s.c_str());
    }

    const int    argc = static_cast<int>(rosArgs.size());
    const char** argv = rosArgsC.data();

    // Initialize ROS (only once):
    if (!rclcpp::ok())
    {
      rclcpp::init(argc, argv);
    }

    auto lckNode = mrpt::lockHelper(rosNodeMtx_);

    rosNode_ = std::make_shared<rclcpp::Node>(NODE_NAME);
    lckNode.unlock();

    {
      auto lck   = mrpt::lockHelper(ros_clock_mtx_);
      ros_clock_ = rosNode_->get_clock();
    }

    // TF buffer:
    tf_buffer_ = std::make_shared<tf2::BufferCore>();  // ros_clock_
    // tf_buffer_->setUsingDedicatedThread(true);
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    // TF broadcaster:
    auto lckTfBc = mrpt::lockHelper(ros_tf_bc_mtx_);

    tf_bc_        = std::make_shared<tf2_ros::TransformBroadcaster>(rosNode_);
    tf_static_bc_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>(rosNode_);

    lckTfBc.unlock();

    // Subscribe to topics as described by MOLA YAML parameters:
    auto ds_subscribe = cfg["subscribe"];
    if (!ds_subscribe.isSequence() || ds_subscribe.asSequence().empty())
    {
      MRPT_LOG_INFO("No ROS2 topic found for subscription under YAML entry `subscribe`.");
    }
    else
    {
      internalAnalyzeTopicsToSubscribe(ds_subscribe);
    }

    auto timerLoc = rosNode_->create_wall_timer(
        std::chrono::microseconds(
            static_cast<unsigned int>(1e6 * params_.period_publish_new_localization)),
        [this]()
        {
          try
          {
            timerPubLocalization();
          }
          catch (const std::exception& e)
          {
            MRPT_LOG_ERROR(e.what());
          }
        });

    auto timerMap = rosNode_->create_wall_timer(
        std::chrono::microseconds(static_cast<unsigned int>(1e6 * params_.period_publish_new_map)),
        [this]()
        {
          try
          {
            timerPubMap();
          }
          catch (const std::exception& e)
          {
            MRPT_LOG_ERROR(e.what());
          }
        });

    auto timerStaticTFs = rosNode_->create_wall_timer(
        std::chrono::microseconds(
            static_cast<unsigned int>(1e6 * params_.period_publish_static_tfs)),
        [this]()
        {
          try
          {
            publishStaticTFs();
          }
          catch (const std::exception& e)
          {
            MRPT_LOG_ERROR(e.what());
          }
        });

    auto timerDiagnostics = rosNode_->create_wall_timer(
        std::chrono::microseconds(
            static_cast<unsigned int>(1e6 * params_.period_publish_diagnostics)),
        [this]()
        {
          try
          {
            publishDiagnostics();
          }
          catch (const std::exception& e)
          {
            MRPT_LOG_ERROR(e.what());
          }
        });

    //
    if (!params_.relocalize_from_topic.empty())
    {
      subInitPose_ = rosNode()->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
          params_.relocalize_from_topic, rclcpp::SystemDefaultsQoS(),
          [this](const geometry_msgs::msg::PoseWithCovarianceStamped& o)
          { this->callbackOnRelocalizeTopic(o); });
    }

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
  const ProfilerEntry tle(profiler_, "initialize");

  // Mandatory parameters:
  ENSURE_YAML_ENTRY_EXISTS(c, "params");
  const auto cfg = c["params"];

  std::stringstream ss;
  cfg.printAsYAML(ss);
  const Yaml cfgCopy = Yaml::FromStream(ss);

  MRPT_LOG_DEBUG_STREAM("Initializing with these params:\n" << cfgCopy);

  // params for ROS2->MOLA:
  YAML_LOAD_OPT(params_, base_link_frame, std::string);
  YAML_LOAD_OPT(params_, odom_frame, std::string);
  YAML_LOAD_OPT(params_, base_footprint_frame, std::string);

  YAML_LOAD_OPT(params_, forward_ros_tf_as_mola_odometry_observations, bool);
  YAML_LOAD_OPT(params_, wait_for_tf_timeout_milliseconds, int);

  YAML_LOAD_OPT(params_, georef_map_reference_frame, std::string);
  YAML_LOAD_OPT(params_, georef_map_utm_frame, std::string);
  YAML_LOAD_OPT(params_, georef_map_enu_frame, std::string);

  YAML_LOAD_OPT(params_, publish_localization_following_rep105, bool);

  if (cfg.has("base_footprint_to_base_link_tf"))
  {
    ASSERT_(cfg["base_footprint_to_base_link_tf"].isSequence());
    ASSERT_EQUAL_(cfg["base_footprint_to_base_link_tf"].asSequenceRange().size(), 6UL);

    // Format: "[x y z yaw pitch roll]" (meters & degrees)
    auto poseSeq = cfg["base_footprint_to_base_link_tf"].toStdVector<double>();
    ASSERT_EQUAL_(poseSeq.size(), 6UL);
    for (int i = 0; i < 3; i++)
    {
      poseSeq[3 + i] = mrpt::DEG2RAD(poseSeq[3 + i]);
    }

    params_.base_footprint_to_base_link_tf = mrpt::math::TPose3D::FromVector(poseSeq);
  }

  // params for MOLA->ROS2:
  YAML_LOAD_OPT(params_, base_link_frame, std::string);
  YAML_LOAD_OPT(params_, reference_frame, std::string);

  YAML_LOAD_OPT(params_, publish_odometry_msgs_from_slam, bool);
  YAML_LOAD_OPT(params_, publish_odometry_msgs_from_slam_source, std::string);

  YAML_LOAD_OPT(params_, publish_tf_from_slam, bool);
  YAML_LOAD_OPT(params_, publish_tf_from_slam_source, std::string);

  YAML_LOAD_OPT(params_, publish_in_sim_time, bool);
  YAML_LOAD_OPT(params_, period_publish_new_localization, double);
  YAML_LOAD_OPT(params_, period_publish_new_map, double);
  YAML_LOAD_OPT(params_, period_publish_static_tfs, double);
  YAML_LOAD_OPT(params_, period_publish_diagnostics, double);

  YAML_LOAD_OPT(params_, publish_tf_from_robot_pose_observations, bool);
  YAML_LOAD_OPT(params_, relocalize_from_topic, std::string);

  // Launch ROS node:
  rosNodeThread_ = std::thread(&BridgeROS2::ros_node_thread_main, this, cfgCopy);

  MRPT_END
}  // end initialize()

void BridgeROS2::spinOnce()
{
  using mrpt::system::timeDifference;

  MRPT_START
  const ProfilerEntry tleg(profiler_, "spinOnce");

  // Publish odometry?
  importRosOdometryToMOLA();

  // Check for new mola data sources?
  if (mrpt::Clock::nowDouble() - lastTimeCheckMolaSubs_ > params_.period_check_new_mola_subs)
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
  const ProfilerEntry tle(profiler_, "callbackOnPointCloud2");

  const std::set<std::string> fields = mrpt::ros2bridge::extractFields(o);

  mrpt::maps::CPointsMap::Ptr mapPtr;

  // If we have anything apart of (x,y,z), use the generic cloud with multiple fields:
  if (fields.size() > 3)
  {
    auto p = mrpt::maps::CGenericPointsMap::Create();
    if (!mrpt::ros2bridge::fromROS(o, *p))
    {
      throw std::runtime_error("Error converting ros->mrpt(?)");
    }

    // Fix timestamps for Livox driver:
    // It uses doubles for timestamps, but they are actually nanoseconds!
#if MRPT_VERSION >= 0x020f03  // 2.15.3
    auto* ts = p->getPointsBufferRef_float_field(mrpt::maps::CPointsMap::POINT_FIELD_TIMESTAMP);
#elif MRPT_VERSION >= 0x020f00  // 2.15.0
    auto* ts =
        p->getPointsBufferRef_float_field(mrpt::maps::CPointsMapXYZIRT::POINT_FIELD_TIMESTAMP);
#else
    auto ts = p->getPointsBufferRef_timestamp();
#endif
    if (ts && !ts->empty())
    {
      const auto [minIt, maxIt] = std::minmax_element(ts->begin(), ts->end());
      const float time_span     = *maxIt - *minIt;
      if (time_span > 1e5F)
      {
        // they must be nanoseconds, convert to seconds:
        for (auto& t : *ts)
        {
          t *= 1e-9F;
        }
      }
    }

    mapPtr = p;
  }
  else
  {
    auto p = mrpt::maps::CSimplePointsMap::Create();
    if (!mrpt::ros2bridge::fromROS(o, *p))
    {
      throw std::runtime_error("Error converting ros->mrpt(?)");
    }

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
        obs_pc->sensorPose, o.header.frame_id, params_.base_link_frame, true /*print errors*/);

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
    mrpt::poses::CPose3D& des, const std::string& frame, const std::string& referenceFrame,
    bool printErrors)
{
  try
  {
    geometry_msgs::msg::TransformStamped ref_to_trgFrame =
        tf_buffer_->lookupTransform(referenceFrame, frame, {});

    tf2::Transform tf;
    tf2::fromMsg(ref_to_trgFrame.transform, tf);
    des = mrpt::ros2bridge::fromROS(tf);

    MRPT_LOG_DEBUG_FMT(
        "[waitForTransform] Found pose %s -> %s: %s", referenceFrame.c_str(), frame.c_str(),
        des.asString().c_str());

    return true;
  }
  catch (const tf2::TransformException& ex)
  {
    if (printErrors)
    {
      MRPT_LOG_ERROR(ex.what());
    }
    return false;
  }
}

void BridgeROS2::callbackOnOdometry(
    const nav_msgs::msg::Odometry& o, const std::string& outSensorLabel)
{
  MRPT_START
  const ProfilerEntry tle(profiler_, "callbackOnOdometry");

  auto obs         = mrpt::obs::CObservationOdometry::Create();
  obs->timestamp   = mrpt::ros2bridge::fromROS(o.header.stamp);
  obs->sensorLabel = outSensorLabel;
  obs->odometry    = mrpt::poses::CPose2D(mrpt::ros2bridge::fromROS(o.pose.pose));

  obs->hasVelocities       = true;
  obs->velocityLocal.vx    = o.twist.twist.linear.x;
  obs->velocityLocal.vy    = o.twist.twist.linear.y;
  obs->velocityLocal.omega = o.twist.twist.angular.z;

  // send it out:
  this->sendObservationsToFrontEnds(obs);

  MRPT_END
}

void BridgeROS2::importRosOdometryToMOLA()
{
  if (!params_.forward_ros_tf_as_mola_odometry_observations)
  {
    return;
  }

  // Is the node already initialized?
  {
    auto lck = mrpt::lockHelper(ros_clock_mtx_);
    if (!ros_clock_)
    {
      return;  // nope...
    }
  }

  // Get pose from tf:
  mrpt::poses::CPose3D odomPose;

  bool odom_tf_ok = waitForTransform(
      odomPose, params_.base_link_frame, params_.odom_frame, false /*dont print errors*/);
  if (!odom_tf_ok)
  {
    MRPT_LOG_THROTTLE_WARN_FMT(
        5.0,
        "forward_ros_tf_as_mola_odometry_observations=true, but could not "
        "resolve /tf for odometry: '%s'->'%s'",
        params_.base_link_frame.c_str(), params_.odom_frame.c_str());
    return;
  }

  const auto now = rclcpp::Time();  // last one.

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
  const ProfilerEntry tle(profiler_, "callbackOnLaserScan");

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
        sensorPose, o.header.frame_id, params_.base_link_frame, true /*print errors*/);

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
  const ProfilerEntry tle(profiler_, "callbackOnImu");

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
        sensorPose, o.header.frame_id, params_.base_link_frame, true /*print errors*/);
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
  const ProfilerEntry tle(profiler_, "callbackOnNavSatFix");

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
        sensorPose, o.header.frame_id, params_.base_link_frame, true /*print errors*/);
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

void BridgeROS2::onNewObservation(const CObservation::ConstPtr& o)
{
  using namespace mrpt::obs;

  ASSERT_(o);

  // TODO(jlbc): Add some filter not to publish everything to ROS?

  if (auto oImg = std::dynamic_pointer_cast<const CObservationImage>(o); oImg)
  {
    return internalOn(*oImg);
  }
  if (auto oPc = std::dynamic_pointer_cast<const CObservationPointCloud>(o); oPc)
  {
    return internalOn(*oPc);
  }
  if (auto oLS = std::dynamic_pointer_cast<const CObservation2DRangeScan>(o); oLS)
  {
    return internalOn(*oLS);
  }
  if (auto oRP = std::dynamic_pointer_cast<const CObservationRobotPose>(o); oRP)
  {
    return internalOn(*oRP);
  }
  if (auto oGPS = std::dynamic_pointer_cast<const CObservationGPS>(o); oGPS)
  {
    return internalOn(*oGPS);
  }
  if (auto oIMU = std::dynamic_pointer_cast<const CObservationIMU>(o); oIMU)
  {
    return internalOn(*oIMU);
  }

  MRPT_LOG_THROTTLE_WARN_FMT(
      5.0,
      "Do not know how to publish to ROS an observation of type '%s' "
      "with sensorLabel='%s'",
      o->GetRuntimeClass()->className, o->sensorLabel.c_str());
}

void BridgeROS2::internalOn(const mrpt::obs::CObservationImage& obs)
{
  // REP-2003: Sensor sources should use SystemDefaultsQoS
  // See: https://ros.org/reps/rep-2003.html
  auto pubImg =
      get_publisher<sensor_msgs::msg::Image>(obs.sensorLabel, rclcpp::SystemDefaultsQoS());

  const std::string sSensorFrameId = obs.sensorLabel;

  // Send TF:
  mrpt::poses::CPose3D sensorPose;
  obs.getSensorPose(sensorPose);

  tf2::Transform transform = mrpt::ros2bridge::toROS_tfTransform(sensorPose);

  if (tf_bc_)
  {
    auto lckTfBc = mrpt::lockHelper(ros_tf_bc_mtx_);

    geometry_msgs::msg::TransformStamped tfStmp;
    tfStmp.transform       = tf2::toMsg(transform);
    tfStmp.child_frame_id  = sSensorFrameId;
    tfStmp.header.frame_id = params_.base_link_frame;
    tfStmp.header.stamp    = myNow(obs.timestamp);
    tf_bc_->sendTransform(tfStmp);
  }

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
  // REP-2003: Sensor sources should use SystemDefaultsQoS
  // See: https://ros.org/reps/rep-2003.html
  auto pubLS =
      get_publisher<sensor_msgs::msg::LaserScan>(obs.sensorLabel, rclcpp::SystemDefaultsQoS());

  const std::string sSensorFrameId = obs.sensorLabel;

  // Send TF:
  mrpt::poses::CPose3D sensorPose;
  obs.getSensorPose(sensorPose);

  tf2::Transform transform = mrpt::ros2bridge::toROS_tfTransform(sensorPose);

  if (tf_bc_)
  {
    auto lckTfBc = mrpt::lockHelper(ros_tf_bc_mtx_);

    geometry_msgs::msg::TransformStamped tfStmp;
    tfStmp.transform       = tf2::toMsg(transform);
    tfStmp.child_frame_id  = sSensorFrameId;
    tfStmp.header.frame_id = params_.base_link_frame;
    tfStmp.header.stamp    = myNow(obs.timestamp);
    tf_bc_->sendTransform(tfStmp);
  }

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
  internalOn(obs, true /*it is a real sensor, publish its /tf*/, obs.sensorLabel /* frame_id */);
}

void BridgeROS2::internalOn(
    const mrpt::obs::CObservationPointCloud& obs, bool isSensorTopic,
    const std::string& sSensorFrameId)
{
  using namespace std::string_literals;

  // REP-2003: https://ros.org/reps/rep-2003.html#id5
  // - Sensors: SystemDefaultsQoS()
  // - Maps:  reliable transient-local
  auto mapQos = isSensorTopic ? rclcpp::SystemDefaultsQoS()
                              : rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable();

  const auto lbPoints = obs.sensorLabel + "_points"s;

  auto pubPoints = get_publisher<sensor_msgs::msg::PointCloud2>(lbPoints, mapQos);

  // POINTS
  // --------

  // Send TF:
  if (isSensorTopic)
  {
    auto lckTfBc = mrpt::lockHelper(ros_tf_bc_mtx_);

    if (tf_bc_)
    {
      mrpt::poses::CPose3D sensorPose = obs.sensorPose;

      tf2::Transform transform = mrpt::ros2bridge::toROS_tfTransform(sensorPose);

      geometry_msgs::msg::TransformStamped tfStmp;
      tfStmp.transform       = tf2::toMsg(transform);
      tfStmp.child_frame_id  = sSensorFrameId;
      tfStmp.header.frame_id = params_.base_link_frame;
      tfStmp.header.stamp    = myNow(obs.timestamp);
      tf_bc_->sendTransform(tfStmp);
    }
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

    if (auto* xyzgen = dynamic_cast<const mrpt::maps::CGenericPointsMap*>(obs.pointcloud.get());
        xyzgen)
    {
      mrpt::ros2bridge::toROS(*xyzgen, msg_header, msg_pts);
    }
#if MRPT_VERSION < 0x030000  // older than v3.0.0, support deprecated classes
    else if (auto* xyzirt = dynamic_cast<const mrpt::maps::CPointsMapXYZIRT*>(obs.pointcloud.get());
             xyzirt)
    {
      mrpt::ros2bridge::toROS(*xyzirt, msg_header, msg_pts);
    }
    else if (auto* xyzi = dynamic_cast<const mrpt::maps::CPointsMapXYZI*>(obs.pointcloud.get());
             xyzi)
    {
      mrpt::ros2bridge::toROS(*xyzi, msg_header, msg_pts);
    }
#endif
    else if (auto* sPts = dynamic_cast<const mrpt::maps::CSimplePointsMap*>(obs.pointcloud.get());
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
  // REP-2003: Sensor sources should use SystemDefaultsQoS
  // See: https://ros.org/reps/rep-2003.html
  auto pubOdo =
      get_publisher<nav_msgs::msg::Odometry>(obs.sensorLabel, rclcpp::SystemDefaultsQoS());

  // Send TF:
  if (params_.publish_tf_from_robot_pose_observations)
  {
    auto lckTfBc = mrpt::lockHelper(ros_tf_bc_mtx_);

    if (tf_bc_)
    {
      tf2::Transform transform = mrpt::ros2bridge::toROS_tfTransform(obs.pose.mean);

      geometry_msgs::msg::TransformStamped tfStmp;
      tfStmp.transform       = tf2::toMsg(transform);
      tfStmp.child_frame_id  = params_.base_link_frame;
      tfStmp.header.frame_id = params_.reference_frame;
      tfStmp.header.stamp    = myNow(obs.timestamp);
      tf_bc_->sendTransform(tfStmp);
    }
  }

  // Send observation:
  {
    obs.load();

    // Convert observation MRPT -> ROS
    nav_msgs::msg::Odometry msg;
    msg.header.stamp    = myNow(obs.timestamp);
    msg.header.frame_id = params_.reference_frame;
    msg.child_frame_id  = params_.base_link_frame;

    msg.pose = mrpt::ros2bridge::toROS_Pose(obs.pose);

    pubOdo->publish(msg);
  }
}

void BridgeROS2::internalOn(const mrpt::obs::CObservationGPS& obs)
{
  auto pubGPS =
      get_publisher<sensor_msgs::msg::NavSatFix>(obs.sensorLabel, rclcpp::SystemDefaultsQoS());

  const std::string sSensorFrameId = obs.sensorLabel;

  // Send TF:
  mrpt::poses::CPose3D sensorPose;
  obs.getSensorPose(sensorPose);

  tf2::Transform transform = mrpt::ros2bridge::toROS_tfTransform(sensorPose);

  if (tf_bc_)
  {
    auto lckTfBc = mrpt::lockHelper(ros_tf_bc_mtx_);

    geometry_msgs::msg::TransformStamped tfStmp;
    tfStmp.transform       = tf2::toMsg(transform);
    tfStmp.child_frame_id  = sSensorFrameId;
    tfStmp.header.frame_id = params_.base_link_frame;
    tfStmp.header.stamp    = myNow(obs.timestamp);
    tf_bc_->sendTransform(tfStmp);
  }

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

void BridgeROS2::internalOn(const mrpt::obs::CObservationIMU& obs)
{
  // REP-2003: Sensor sources should use SystemDefaultsQoS
  // See: https://ros.org/reps/rep-2003.html
  auto pubImu = get_publisher<sensor_msgs::msg::Imu>(obs.sensorLabel, rclcpp::SystemDefaultsQoS());

  const std::string sSensorFrameId = obs.sensorLabel;

  // Send TF:
  mrpt::poses::CPose3D sensorPose;
  obs.getSensorPose(sensorPose);

  if (tf_bc_)
  {
    auto lckTfBc = mrpt::lockHelper(ros_tf_bc_mtx_);

    tf2::Transform transform = mrpt::ros2bridge::toROS_tfTransform(sensorPose);

    geometry_msgs::msg::TransformStamped tfStmp;
    tfStmp.transform       = tf2::toMsg(transform);
    tfStmp.child_frame_id  = sSensorFrameId;
    tfStmp.header.frame_id = params_.base_link_frame;
    tfStmp.header.stamp    = myNow(obs.timestamp);
    tf_bc_->sendTransform(tfStmp);
  }

  // Send observation:
  {
    obs.load();

    // Convert observation MRPT -> ROS
    std_msgs::msg::Header header;
    header.stamp    = myNow(obs.timestamp);
    header.frame_id = sSensorFrameId;

    sensor_msgs::msg::Imu imu;
    mrpt::ros2bridge::toROS(obs, header, imu);

    pubImu->publish(imu);
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
        std::string(this->GetRuntimeClass()->className))
    {
      continue;
    }

    if (molaSubs_.dataSources.count(rds) == 0)
    {
      MRPT_LOG_INFO_STREAM(
          "Subscribing to MOLA data source module '" << rds->getModuleInstanceName() << "'");

      // a new one:
      molaSubs_.dataSources.insert(rds);
      rds->attachToDataConsumer(*this);
    }
  }

  // LocalizationSourceBase:
  auto listLoc = this->findService<mola::LocalizationSourceBase>();
  for (auto& module : listLoc)
  {
    auto loc = std::dynamic_pointer_cast<mola::LocalizationSourceBase>(module);
    ASSERT_(loc);

    if (molaSubs_.locSources.count(loc) == 0)
    {
      MRPT_LOG_INFO_STREAM(
          "Subscribing to MOLA localization source module '" << module->getModuleInstanceName()
                                                             << "'");

      // a new one:
      molaSubs_.locSources.insert(loc);
      loc->subscribeToLocalizationUpdates([this](const auto& l) { onNewLocalization(l); });
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
          "Subscribing to MOLA map source module '" << module->getModuleInstanceName() << "'");

      // a new one:
      molaSubs_.mapSources.insert(ms);
      ms->subscribeToMapUpdates([this](const auto& m) { onNewMap(m); });
    }
  }

  // relocalization
  auto listRelog = this->findService<mola::Relocalization>();
  for (auto& module : listRelog)
  {
    auto ms = std::dynamic_pointer_cast<mola::Relocalization>(module);
    ASSERT_(ms);

    if (molaSubs_.relocalization.count(ms) == 0)
    {
      MRPT_LOG_INFO_STREAM(
          "Subscribing to MOLA relocalization module '" << module->getModuleInstanceName() << "'");

      // a new one:
      molaSubs_.relocalization.insert(ms);
    }
  }

  // Advertise relocalization ROS 2 service now if not done already:
  auto lckNode = mrpt::lockHelper(rosNodeMtx_);

  if (!molaSubs_.relocalization.empty() && !srvRelocSE_ && rosNode_)
  {
    using namespace std::placeholders;

    srvRelocSE_ = rosNode_->create_service<mola_msgs::srv::RelocalizeFromStateEstimator>(
        "relocalize_from_state_estimator",
        std::bind(&BridgeROS2::service_relocalize_from_se, this, _1, _2));

    srvRelocPose_ = rosNode_->create_service<mola_msgs::srv::RelocalizeNearPose>(
        "relocalize_near_pose", std::bind(&BridgeROS2::service_relocalize_near_pose, this, _1, _2));
  }

  // Map server
  auto listMapServers = this->findService<mola::MapServer>();
  for (auto& module : listMapServers)
  {
    auto ms = std::dynamic_pointer_cast<mola::MapServer>(module);
    ASSERT_(ms);

    if (molaSubs_.mapServers.count(ms) == 0)
    {
      MRPT_LOG_INFO_STREAM(
          "Subscribing to MOLA map server module '" << module->getModuleInstanceName() << "'");

      if (molaSubs_.mapServers.size() > 1)
      {
        MRPT_LOG_WARN(
            "More than one MapServer MOLA modules seems to be running. ROS 2 requests will "
            "be forwarded to the first module only.");
      }

      // a new one:
      molaSubs_.mapServers.insert(ms);
    }
  }

  // Advertise map server ROS 2 services now if not done already:
  if (!molaSubs_.mapServers.empty() && !srvMapLoad_ && rosNode_)
  {
    using namespace std::placeholders;

    srvMapLoad_ = rosNode_->create_service<mola_msgs::srv::MapLoad>(
        "map_load", std::bind(&BridgeROS2::service_map_load, this, _1, _2));

    srvMapSave_ = rosNode_->create_service<mola_msgs::srv::MapSave>(
        "map_save", std::bind(&BridgeROS2::service_map_save, this, _1, _2));
  }

  // Advertise runtime MOLA parameters:
  if (!srvParamGet_ && rosNode_)
  {
    using namespace std::placeholders;

    srvParamGet_ = rosNode_->create_service<mola_msgs::srv::MolaRuntimeParamGet>(
        "mola_runtime_param_get", std::bind(&BridgeROS2::service_param_get, this, _1, _2));

    srvParamSet_ = rosNode_->create_service<mola_msgs::srv::MolaRuntimeParamSet>(
        "mola_runtime_param_set", std::bind(&BridgeROS2::service_param_set, this, _1, _2));
  }
}

void BridgeROS2::service_relocalize_from_se(
    [[maybe_unused]] const std::shared_ptr<mola_msgs::srv::RelocalizeFromStateEstimator::Request>
                                                                            request,
    std::shared_ptr<mola_msgs::srv::RelocalizeFromStateEstimator::Response> response)
{
  auto lck = mrpt::lockHelper(rosPubsMtx_);
  if (molaSubs_.relocalization.empty())
  {
    response->accepted = false;
    return;
  }

  for (auto& m : molaSubs_.relocalization)
  {
    m->relocalize_from_gnss();
  }
  response->accepted = true;
}

void BridgeROS2::service_relocalize_near_pose(
    const std::shared_ptr<mola_msgs::srv::RelocalizeNearPose::Request> request,
    std::shared_ptr<mola_msgs::srv::RelocalizeNearPose::Response>      response)
{
  auto lck = mrpt::lockHelper(rosPubsMtx_);
  if (molaSubs_.relocalization.empty())
  {
    response->accepted = false;
    return;
  }

  for (auto m : molaSubs_.relocalization)
  {
    const mrpt::poses::CPose3DPDFGaussian p = mrpt::ros2bridge::fromROS(request->pose.pose);
    m->relocalize_near_pose_pdf(p);
  }

  response->accepted = true;
}

void BridgeROS2::callbackOnRelocalizeTopic(const geometry_msgs::msg::PoseWithCovarianceStamped& o)
{
  auto lck = mrpt::lockHelper(rosPubsMtx_);

  for (auto m : molaSubs_.relocalization)
  {
    const mrpt::poses::CPose3DPDFGaussian p = mrpt::ros2bridge::fromROS(o.pose);
    m->relocalize_near_pose_pdf(p);
  }
}

void BridgeROS2::service_map_load(
    const std::shared_ptr<mola_msgs::srv::MapLoad::Request> request,
    std::shared_ptr<mola_msgs::srv::MapLoad::Response>      response)
{
  auto lck = mrpt::lockHelper(rosPubsMtx_);
  if (molaSubs_.relocalization.empty())
  {
    response->success       = false;
    response->error_message = "No MOLA module with MapServer interface is running.";
    MRPT_LOG_WARN(response->error_message);
    return;
  }

  auto& m = *molaSubs_.mapServers.begin();
  ASSERT_(m);
  const auto& r = m->map_load(request->map_path);

  response->success       = r.success;
  response->error_message = r.error_message;
}

void BridgeROS2::service_map_save(
    const std::shared_ptr<mola_msgs::srv::MapSave::Request> request,
    std::shared_ptr<mola_msgs::srv::MapSave::Response>      response)
{
  auto lck = mrpt::lockHelper(rosPubsMtx_);
  if (molaSubs_.relocalization.empty())
  {
    response->success       = false;
    response->error_message = "No MOLA module with MapServer interface is running.";
    MRPT_LOG_WARN(response->error_message);
    return;
  }

  auto& m = *molaSubs_.mapServers.begin();
  ASSERT_(m);
  const auto& r = m->map_save(request->map_path);

  response->success       = r.success;
  response->error_message = r.error_message;
}

void BridgeROS2::service_param_get(
    [[maybe_unused]] const std::shared_ptr<mola_msgs::srv::MolaRuntimeParamGet::Request> request,
    std::shared_ptr<mola_msgs::srv::MolaRuntimeParamGet::Response>                       response)
{
  auto lck = mrpt::lockHelper(molaSubsMtx_);

  mrpt::containers::yaml allParams = mrpt::containers::yaml::Map();

  // Get all MOLA modules:
  auto listMods = this->findService<mola::ExecutableBase>();
  for (auto& m : listMods)
  {
    // Skip myself!
    if (std::string(m->GetRuntimeClass()->className) ==
        std::string(this->GetRuntimeClass()->className))
    {
      continue;
    }

    // Get params:
    mrpt::containers::yaml params = m->getModuleParameters();
    // add to YAML tree indexed by module name:
    allParams[m->getModuleInstanceName()] = params;
  }

  std::stringstream                 ss;
  mrpt::containers::YamlEmitOptions o;
  o.emitHeader = false;
  allParams.printAsYAML(ss, o);
  response->parameters = ss.str();
}

void BridgeROS2::service_param_set(
    const std::shared_ptr<mola_msgs::srv::MolaRuntimeParamSet::Request> request,
    std::shared_ptr<mola_msgs::srv::MolaRuntimeParamSet::Response>      response)
{
  auto lck = mrpt::lockHelper(molaSubsMtx_);
  try
  {
    mrpt::containers::yaml allParams;
    {
      std::stringstream ss(request->parameters);
      allParams.loadFromStream(ss);
      ASSERT_(allParams.isMap());
    }

    // Get all MOLA modules:
    auto listMods = this->findService<mola::ExecutableBase>();
    std::map<std::string, mola::ExecutableBase::Ptr> modsByName;
    for (auto& m : listMods)
    {
      // Skip myself!
      if (std::string(m->GetRuntimeClass()->className) ==
          std::string(this->GetRuntimeClass()->className))
      {
        continue;
      }
      modsByName[m->getModuleInstanceName()] = m;
    }

    for (const auto& [k, v] : allParams.asMapRange())
    {
      const auto paramsModuleName = k.as<std::string>();
      auto       itMod            = modsByName.find(paramsModuleName);
      if (itMod == modsByName.end())
      {
        THROW_EXCEPTION_FMT(
            "Error: requested setting runtime parameter for non-existing MOLA module '%s'",
            paramsModuleName.c_str());
      }

      auto& m = itMod->second;
      ASSERT_(m);
      // Send change of parameters:
      m->changeParameters(v);
    }
    // If we had no exceptions, it means we are ok.
    response->success = true;
  }
  catch (const std::exception& e)
  {
    response->success       = false;
    response->error_message = e.what();
  }
}

rclcpp::Time BridgeROS2::myNow(const mrpt::Clock::time_point& observationStamp)
{
  if (params_.publish_in_sim_time)
  {
    return mrpt::ros2bridge::toROS(observationStamp);
  }

  return mrpt::ros2bridge::toROS(mrpt::Clock::now());
}

void BridgeROS2::onNewLocalization(const mola::LocalizationSourceBase::LocalizationUpdate& l)
{
  auto lck = mrpt::lockHelper(lastLocMapMtx_);

  lastLocUpdates_.push_back(l);
}

void BridgeROS2::onNewMap(const mola::MapSourceBase::MapUpdate& m)
{
  auto lck = mrpt::lockHelper(lastLocMapMtx_);

  if (m.keep_last_one_only)
  {
    if (auto it = lastMaps_.find(m.map_name); it != lastMaps_.end())
    {
      lastMaps_.erase(it);
    }
  }
  lastMaps_.insert({m.map_name, m});
}

void BridgeROS2::timerPubLocalization()
{
  using namespace std::string_literals;

  // get a copy of the data minimizing the time owning the mutex:
  std::vector<mola::LocalizationSourceBase::LocalizationUpdate> ls;
  {
    auto lck = mrpt::lockHelper(lastLocMapMtx_);
    ls       = lastLocUpdates_;
    lastLocUpdates_.clear();
  }
  if (ls.empty())
  {
    return;
  }

  for (const auto& l : ls)
  {
    MRPT_LOG_DEBUG_STREAM(
        "New localization available from '"
        << l.method << "' ref.frame: '" << l.reference_frame << "' child_frame: '" << l.child_frame
        << "'  <<  t=" << mrpt::system::dateTimeLocalToString(l.timestamp)
        << " pose=" << l.pose.asString());

    // 1/2: Publish to /tf:
    const std::string locLabel        = (l.method.empty() ? "slam"s : l.method) + "/pose"s;
    const std::string locQualityLabel = (l.method.empty() ? "slam"s : l.method) + "/pose_quality"s;

    auto pubOdo = get_publisher<nav_msgs::msg::Odometry>(locLabel, rclcpp::SystemDefaultsQoS());
    auto pubOdoQuality =
        get_publisher<std_msgs::msg::Float32>(locQualityLabel, rclcpp::SystemDefaultsQoS());

    // Send TF with localization result
    // 1) Direct mode:    reference_frame ("map") -> base_link ("base_link")
    // 2) Indirect mode:  map -> odom  (such as "map -> odom -> base_link" = "map -> base_link")
    if (params_.publish_tf_from_slam && (params_.publish_tf_from_slam_source.empty() ||
                                         params_.publish_tf_from_slam_source == l.method))
    {
      tf2::Transform transform = mrpt::ros2bridge::toROS_tfTransform(l.pose);

      geometry_msgs::msg::TransformStamped tfStmp;
      tfStmp.header.stamp = myNow(l.timestamp);

      // Follow REP105 only if we are publishing "map" -> "base_link" poses.
      if (params_.publish_localization_following_rep105 &&
          l.child_frame == params_.base_link_frame && l.reference_frame == params_.reference_frame)
      {
        // Recompute:
        mrpt::poses::CPose3D T_base_to_odom;
        bool                 base_to_odom_ok =
            this->waitForTransform(T_base_to_odom, params_.odom_frame, l.child_frame, true);
        // Note: this wait above typ takes ~50 us

        if (!base_to_odom_ok)
        {
          MRPT_LOG_ERROR_STREAM(
              "publish_localization_following_rep105 is true but could not resolve tf "
              "for "
              "odom "
              "-> base_link");
        }
        else
        {
          const tf2::Transform& baseOnMap_tf = transform;

          const tf2::Transform odomOnBase_tf = mrpt::ros2bridge::toROS_tfTransform(T_base_to_odom);

          tfStmp.transform       = tf2::toMsg(baseOnMap_tf * odomOnBase_tf);
          tfStmp.child_frame_id  = params_.odom_frame;
          tfStmp.header.frame_id = l.reference_frame;
        }
      }
      else
      {
        tfStmp.transform       = tf2::toMsg(transform);
        tfStmp.child_frame_id  = l.child_frame;
        tfStmp.header.frame_id = l.reference_frame;
      }

      auto lckTfBc = mrpt::lockHelper(ros_tf_bc_mtx_);
      if (tf_bc_)
      {
        tf_bc_->sendTransform(tfStmp);
      }
    }

    // 2/2: Publish Odometry msg:
    if (params_.publish_odometry_msgs_from_slam &&
        (params_.publish_odometry_msgs_from_slam_source.empty() ||
         params_.publish_odometry_msgs_from_slam_source == l.method))
    {
      // Convert observation MRPT -> ROS
      nav_msgs::msg::Odometry msg;
      msg.header.stamp    = myNow(l.timestamp);
      msg.child_frame_id  = l.child_frame;
      msg.header.frame_id = l.reference_frame;

      mrpt::poses::CPose3DPDFGaussian posePdf;
      posePdf.mean = mrpt::poses::CPose3D(l.pose);
      if (l.cov)
      {
        posePdf.cov = l.cov.value();
      }

      msg.pose = mrpt::ros2bridge::toROS_Pose(posePdf);

      pubOdo->publish(msg);
    }

    // And always publish quality:
    {
      std_msgs::msg::Float32 msg;
      msg.data = static_cast<float>(l.quality);
      pubOdoQuality->publish(msg);
    }
  }
}

void BridgeROS2::timerPubMap()
{
  using namespace std::string_literals;

  // get a copy of the data minimizing the time owning the mutex:
  std::multimap<std::string /*map_name*/, mola::MapSourceBase::MapUpdate> m;
  {
    auto lck  = mrpt::lockHelper(lastLocMapMtx_);
    m         = std::move(lastMaps_);
    lastMaps_ = {};
  }
  if (m.empty())
  {
    return;
  }

  MRPT_LOG_DEBUG_STREAM("New map layers (" << m.size() << ") received");

  for (const auto& [layerName, mu] : m)
  {
    const std::string mapTopic = (mu.method.empty() ? "slam"s : mu.method) + "/"s + layerName;

    MRPT_LOG_DEBUG_STREAM(
        "Publishing map topic '" << mapTopic << "', source class: "
                                 << (mu.map ? mu.map->GetRuntimeClass()->className : "(null)"));

    // Is it a point cloud?
    if (const auto mapPts = std::dynamic_pointer_cast<const mrpt::maps::CPointsMap>(mu.map); mapPts)
    {
      mrpt::obs::CObservationPointCloud obs;
      obs.sensorLabel = mapTopic;
      obs.timestamp   = mu.timestamp;
      obs.pointcloud  = std::const_pointer_cast<mrpt::maps::CPointsMap>(mapPts);
      // Reuse code for point cloud observations: build a "fake" observation:
      internalOn(obs, false /*no tf*/, mu.reference_frame);
    }
    // Is it a grid map?
    else if (auto grid = std::dynamic_pointer_cast<const mrpt::maps::COccupancyGridMap2D>(mu.map);
             grid)
    {
      internalPublishGridMap(*grid, mapTopic, mu.reference_frame, mu.timestamp);
    }
    // Is it a CVoxelMap?
    else if (auto vox = std::dynamic_pointer_cast<const mrpt::maps::CVoxelMap>(mu.map); vox)
    {
      mrpt::maps::CSimplePointsMap::Ptr pm = vox->getOccupiedVoxels();
      if (pm)
      {
        mrpt::obs::CObservationPointCloud obs;
        obs.sensorLabel = mapTopic;
        obs.pointcloud  = pm;
        obs.timestamp   = mu.timestamp;
        // Reuse code for point cloud observations: build a "fake" observation:
        internalOn(obs, false /*no tf*/, mu.reference_frame);
      }
      else
      {
        MRPT_LOG_WARN_STREAM("Voxel map '" << layerName << "' has no occupied voxels to publish.");
      }
    }
    // Not empty?
    else if (mu.map)
    {
      // Try to publish it via it's simple pointsmap representation:
      const auto* pts = mu.map->getAsSimplePointsMap();
      if (!pts)
      {
        MRPT_LOG_WARN_STREAM(
            "Do not know how to publish map layer '"
            << layerName << "' of type '" << mu.map->GetRuntimeClass()->className << "'");
      }
      else
      {
        mrpt::obs::CObservationPointCloud obs;
        obs.sensorLabel = mapTopic;
        obs.pointcloud  = std::make_shared<mrpt::maps::CSimplePointsMap>(*pts);
        obs.timestamp   = mu.timestamp;
        // Reuse code for point cloud observations: build a "fake" observation:
        internalOn(obs, false /*no tf*/, mu.reference_frame);
      }
    }

    // If we have georef info, publish it:
    if (mu.georeferencing.has_value())
    {
      const std::string georefTopic =
          (mu.method.empty() ? "slam"s : mu.method) + "/geo_ref_metadata"s;
      publishMetricMapGeoreferencingData(*mu.georeferencing, georefTopic);
    }

    // If it has metadata, publish it:
    if (mu.map_metadata.has_value())
    {
      const std::string metadataTopic = (mu.method.empty() ? "slam"s : mu.method) + "/metadata"s;

      auto lck = mrpt::lockHelper(rosPubsMtx_);

      // Publish it as transient local:
      auto mapQos      = rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable();
      auto pubMetadata = get_publisher<std_msgs::msg::String>(metadataTopic, mapQos);
      std_msgs::msg::String msg;
      msg.data = *mu.map_metadata;
      pubMetadata->publish(msg);
    }
  }
}

void BridgeROS2::publishMetricMapGeoreferencingData(
    const mola::Georeferencing& g, const std::string& georefTopic)
{
  MRPT_LOG_INFO_STREAM(
      "Publishing map georeferencing metadata: T_enu_to_map="
      << g.T_enu_to_map.asString()  //
      << " geo_coord.lat=" << g.geo_coord.lat.getAsString()  //
      << " geo_coord.lon=" << g.geo_coord.lon.getAsString()  //
      << " geo_coord.height=" << g.geo_coord.height  //
  );

  // Publish several georeference items:
  // 1) /tf's  ENU->MAP, ENU->UTM
  // 2) msg: mrpt_nav_interfaces::msg::GeoreferencingMetadata

  // 1.a) ENU -> MAP
  {
    const auto& T_enu_to_map = g.T_enu_to_map.mean;

    geometry_msgs::msg::TransformStamped tfStmp;

    tfStmp.transform      = tf2::toMsg(mrpt::ros2bridge::toROS_tfTransform(T_enu_to_map.asTPose()));
    tfStmp.child_frame_id = params_.georef_map_reference_frame;  // "map"
    tfStmp.header.frame_id = params_.georef_map_enu_frame;  // "enu"
    tfStmp.header.stamp    = myNow(mrpt::Clock::now());

    auto lckTfBc = mrpt::lockHelper(ros_tf_bc_mtx_);
    if (tf_static_bc_)
    {
      tf_static_bc_->sendTransform(tfStmp);
    }
  }

  // 1.b) ENU -> UTM
  mrpt::poses::CPose3D T_enu_to_utm;
  int                  utmZone = 0;
  char                 utmBand = 0;
  {
    mrpt::topography::TUTMCoords utmCoordsOfENU;
    mrpt::topography::GeodeticToUTM(g.geo_coord, utmCoordsOfENU, utmZone, utmBand);

    // T_enu_to_utm = - utmCoordsOfENU  (without rotation, both are "ENU")
    T_enu_to_utm = mrpt::poses::CPose3D::FromTranslation(-utmCoordsOfENU);

    geometry_msgs::msg::TransformStamped tfStmp;

    tfStmp.transform      = tf2::toMsg(mrpt::ros2bridge::toROS_tfTransform(T_enu_to_utm.asTPose()));
    tfStmp.child_frame_id = params_.georef_map_utm_frame;  // "utm"
    tfStmp.header.frame_id = params_.georef_map_enu_frame;  // "enu"
    tfStmp.header.stamp    = myNow(mrpt::Clock::now());

    auto lckTfBc = mrpt::lockHelper(ros_tf_bc_mtx_);
    if (tf_static_bc_)
    {
      tf_static_bc_->sendTransform(tfStmp);
    }
  }

  // 2) g.geo_coord => georefTopic
  auto lck = mrpt::lockHelper(rosPubsMtx_);

  // Publish georef info as transient local:
  auto mapQos = rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable();

  auto pubGeoRef =
      get_publisher<mrpt_nav_interfaces::msg::GeoreferencingMetadata>(georefTopic, mapQos);

  mrpt_nav_interfaces::msg::GeoreferencingMetadata geoRefMsg;
  geoRefMsg.valid = true;

  geoRefMsg.t_enu_to_map = mrpt::ros2bridge::toROS_Pose(g.T_enu_to_map);
  geoRefMsg.t_enu_to_utm = mrpt::ros2bridge::toROS_Pose(T_enu_to_utm);

  geoRefMsg.latitude  = g.geo_coord.lat.decimal_value;
  geoRefMsg.longitude = g.geo_coord.lon.decimal_value;
  geoRefMsg.height    = g.geo_coord.height;

  geoRefMsg.utm_zone = utmZone;
  geoRefMsg.utm_band = mrpt::format("%c", utmBand);

  pubGeoRef->publish(geoRefMsg);
}

void BridgeROS2::internalAnalyzeTopicsToSubscribe(const mrpt::containers::yaml& ds_subscribe)
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

    const auto topic_name          = topic["topic"].as<std::string>();
    const auto type                = topic["msg_type"].as<std::string>();
    const auto output_sensor_label = topic["output_sensor_label"].as<std::string>();

    MRPT_LOG_DEBUG_STREAM(
        "Creating ros2 subscriber for topic='" << topic_name << "' (" << type << ")");

    // Optional: fixed sensorPose (then ignores/don't need "tf" data):
    std::optional<mrpt::poses::CPose3D> fixedSensorPose;
    if (topic.has("fixed_sensor_pose") &&
        (!topic.has("use_fixed_sensor_pose") || topic["use_fixed_sensor_pose"].as<bool>()))
    {
      fixedSensorPose = mrpt::poses::CPose3D::FromString(
          "["s + topic["fixed_sensor_pose"].as<std::string>() + "]"s);
    }

    if (type == "PointCloud2")
    {
      subsPointCloud_.emplace_back(rosNode_->create_subscription<sensor_msgs::msg::PointCloud2>(
          topic_name, qos,
          [this, output_sensor_label, fixedSensorPose](const sensor_msgs::msg::PointCloud2& o)
          { this->callbackOnPointCloud2(o, output_sensor_label, fixedSensorPose); }));
    }
    else if (type == "LaserScan")
    {
      subsLaserScan_.emplace_back(rosNode_->create_subscription<sensor_msgs::msg::LaserScan>(
          topic_name, qos,
          [this, output_sensor_label, fixedSensorPose](const sensor_msgs::msg::LaserScan& o)
          { this->callbackOnLaserScan(o, output_sensor_label, fixedSensorPose); }));
    }
    else if (type == "Imu")
    {
      subsImu_.emplace_back(rosNode_->create_subscription<sensor_msgs::msg::Imu>(
          topic_name, qos,
          [this, output_sensor_label, fixedSensorPose](const sensor_msgs::msg::Imu& o)
          { this->callbackOnImu(o, output_sensor_label, fixedSensorPose); }));
    }
    else if (type == "NavSatFix")
    {
      subsGNSS_.emplace_back(rosNode_->create_subscription<sensor_msgs::msg::NavSatFix>(
          topic_name, qos,
          [this, output_sensor_label, fixedSensorPose](const sensor_msgs::msg::NavSatFix& o)
          { this->callbackOnNavSatFix(o, output_sensor_label, fixedSensorPose); }));
    }
    else if (type == "Odometry")
    {
      subsOdometry_.emplace_back(rosNode_->create_subscription<nav_msgs::msg::Odometry>(
          topic_name, qos,
          [this, output_sensor_label](const nav_msgs::msg::Odometry& o)
          { this->callbackOnOdometry(o, output_sensor_label); }));
    }
    else
    {
      THROW_EXCEPTION_FMT("Unhandled type=`%s` for topic=`%s`", type.c_str(), topic_name.c_str());
    }
  }
}

void BridgeROS2::publishStaticTFs()
{
  if (params_.base_footprint_frame.empty())
  {
    return;
  }

  const tf2::Transform transform =
      mrpt::ros2bridge::toROS_tfTransform(-params_.base_footprint_to_base_link_tf);

  geometry_msgs::msg::TransformStamped tfStmp;

  tfStmp.transform       = tf2::toMsg(transform);
  tfStmp.child_frame_id  = params_.base_footprint_frame;
  tfStmp.header.frame_id = params_.base_link_frame;
  tfStmp.header.stamp    = myNow(mrpt::Clock::now());

  auto lckTfBc = mrpt::lockHelper(ros_tf_bc_mtx_);
  if (tf_static_bc_)
  {
    tf_static_bc_->sendTransform(tfStmp);
  }
}

namespace
{
/// 'mola::LidarOdometry:lidar_odom' -> 'lidar_odom'
std::string module_name_to_valid_topic(const std::string& s)
{
  const auto lastPos = s.find_last_of(':');
  if (lastPos == std::string::npos)
  {
    return s;
  }

  return s.substr(lastPos + 1);
}
}  // namespace

void BridgeROS2::publishDiagnostics()
{
#if MOLA_VERSION_CHECK(1, 6, 2)
  using namespace std::string_literals;

  const auto qos = rclcpp::SystemDefaultsQoS();

  // Get all MOLA modules:
  auto listMods = this->findService<mola::ExecutableBase>();
  for (auto& m : listMods)
  {
    const std::string topicPrefix =
        "mola_diagnostics/"s + module_name_to_valid_topic(m->getModuleInstanceName()) + "/"s;

    const auto diagnosticsMsgs = m->module_move_out_diagnostics_messages();

    for (const auto& diag : diagnosticsMsgs)
    {
      const std::string topic = topicPrefix + diag.label;

      // TODO: No way to embed this in std_msgs (!): diag.timestamp;

      if (const auto* valString = std::any_cast<std::string>(&diag.value); valString)
      {
        auto                  pubDiag = get_publisher<std_msgs::msg::String>(topic, qos);
        std_msgs::msg::String msg;
        msg.data = *valString;
        pubDiag->publish(msg);
      }
      else if (const auto* valYaml = std::any_cast<mola::Yaml>(&diag.value); valYaml)
      {
        auto                  pubDiag = get_publisher<std_msgs::msg::String>(topic, qos);
        std_msgs::msg::String msg;
        std::stringstream     ss;
        mrpt::containers::YamlEmitOptions eo;
        eo.emitHeader = false;
        valYaml->printAsYAML(ss, eo);
        msg.data = ss.str();
        pubDiag->publish(msg);
      }
      else if (const auto* valDouble = std::any_cast<double>(&diag.value); valDouble)
      {
        auto                   pubDiag = get_publisher<std_msgs::msg::Float32>(topic, qos);
        std_msgs::msg::Float32 msg;
        msg.data = *valDouble;
        pubDiag->publish(msg);
      }
      else if (const auto* valFloat = std::any_cast<float>(&diag.value); valFloat)
      {
        auto                   pubDiag = get_publisher<std_msgs::msg::Float32>(topic, qos);
        std_msgs::msg::Float32 msg;
        msg.data = *valFloat;
        pubDiag->publish(msg);
      }
      else if (const auto* valInt = std::any_cast<int>(&diag.value); valInt)
      {
        auto                   pubDiag = get_publisher<std_msgs::msg::Float32>(topic, qos);
        std_msgs::msg::Float32 msg;
        msg.data = *valInt;
        pubDiag->publish(msg);
      }
      else
      {
        MRPT_LOG_THROTTLE_WARN_STREAM(
            10.0, "Do not know how to publish diagnostic message named '" << topic
                                                                          << "' of unknown type.");
      }

    }  // fof each diagnostics message
  }  // end for each module
#endif
}

void BridgeROS2::internalPublishGridMap(
    const mrpt::maps::COccupancyGridMap2D& m, const std::string& sMapTopicName,
    const std::string& sReferenceFrame, const mrpt::Clock::time_point& timestamp)
{
  using namespace std::string_literals;

  const std::string grid_topic          = sMapTopicName + "_gridmap"s;
  const std::string grid_metadata_topic = sMapTopicName + "_gridmap_metadata"s;

  // REP-2003: https://ros.org/reps/rep-2003.html#id5
  // - Maps:  reliable transient-local
  auto qos = rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable();

  auto pubGrid = get_publisher<nav_msgs::msg::OccupancyGrid>(grid_topic, qos);
  auto pubMeta = get_publisher<nav_msgs::msg::MapMetaData>(grid_metadata_topic, qos);

  std_msgs::msg::Header msg_header;
  msg_header.stamp    = myNow(timestamp);
  msg_header.frame_id = sReferenceFrame;

  {
    nav_msgs::msg::OccupancyGrid gridMsg;
    mrpt::ros2bridge::toROS(m, gridMsg, msg_header);
    pubGrid->publish(gridMsg);
    pubMeta->publish(gridMsg.info);
  }
}
