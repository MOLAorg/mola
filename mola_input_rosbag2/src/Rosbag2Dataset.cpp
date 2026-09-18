/*               _
 _ __ ___   ___ | | __ _
| '_ ` _ \ / _ \| |/ _` | Modular Optimization framework for
| | | | | | (_) | | (_| | Localization and mApping (MOLA)
|_| |_| |_|\___/|_|\__,_| https://github.com/MOLAorg/mola

 Copyright (C) 2018-2026 Jose Luis Blanco, University of Almeria,
                         and individual contributors.
 SPDX-License-Identifier: GPL-3.0
 See LICENSE for full license information.
*/

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
 * rosbag2rawlog (MRPT project), Hunter Laux, 2018, JLBC, 2018-2024.
 */

#include <mola_input_rosbag2/Rosbag2Dataset.h>
#include <mola_yaml/yaml_helpers.h>
#include <mrpt/containers/yaml.h>
#include <mrpt/core/Clock.h>
#include <mrpt/core/bits_math.h>
#include <mrpt/core/initializer.h>
#include <mrpt/obs/CActionRobotMovement3D.h>
#include <mrpt/obs/CObservation2DRangeScan.h>
#include <mrpt/obs/CObservation3DRangeScan.h>
#include <mrpt/obs/CObservationIMU.h>
#include <mrpt/obs/CObservationImage.h>
#include <mrpt/obs/CObservationOdometry.h>
#include <mrpt/obs/CObservationPointCloud.h>
#include <mrpt/obs/CObservationRobotPose.h>
#include <mrpt/obs/CObservationRotatingScan.h>
#include <mrpt/ros2bridge/gps.h>
#include <mrpt/ros2bridge/imu.h>
#include <mrpt/ros2bridge/laser_scan.h>
#include <mrpt/ros2bridge/point_cloud2.h>
#include <mrpt/ros2bridge/pose.h>
#include <mrpt/ros2bridge/time.h>
#include <mrpt/system/filesystem.h>
#include <mrpt/version.h>

#include <chrono>
#include <set>
#include <tf2/buffer_core.hpp>
#include <tf2/convert.hpp>
#include <tf2/exceptions.hpp>
#include <type_traits>

#if CV_BRIDGE_VERSION < 0x030400
#include <cv_bridge/cv_bridge.h>
#else
#include <cv_bridge/cv_bridge.hpp>
#endif

#if MRPT_ROS2_BRIDGE_VERSION >= 0x030400
#include <mrpt/ros2bridge/rosbag2_to_mrpt.h>
#endif

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/serialization.hpp>
#include <rosbag2_cpp/converter_options.hpp>
#include <rosbag2_cpp/readers/sequential_reader.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/compressed_image.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <std_msgs/msg/int32.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_msgs/msg/tf_message.hpp>

using namespace mola;

namespace
{
// The bag message receive-timestamp field was renamed from "time_stamp" to
// "recv_timestamp" across rosbag2_storage versions. Detect which one is
// available at compile time so the same source works for all ROS 2 distros.
template <typename T, typename = void>
struct HasTimeStampField : std::false_type
{
};

template <typename T>
struct HasTimeStampField<T, std::void_t<decltype(std::declval<T>().time_stamp)>> : std::true_type
{
};

template <typename T>
int64_t bagMessageRecvTimestampNs(const T& m)
{
  if constexpr (HasTimeStampField<T>::value)
  {
    return m.time_stamp;
  }
  else
  {
    return m.recv_timestamp;
  }
}
}  // namespace

// arguments: class_name, parent_class, class namespace
IMPLEMENTS_MRPT_OBJECT(Rosbag2Dataset, RawDataSourceBase, mola)

MRPT_INITIALIZER(do_register_Rosbag2Dataset)  // NOLINT(misc-use-anonymous-namespace)
{
  MOLA_REGISTER_MODULE(Rosbag2Dataset);
}

Rosbag2Dataset::Rosbag2Dataset()
{
  this->setLoggerName("Rosbag2Dataset");
  tfBuffer_ = std::make_shared<tf2::BufferCore>();
}

void Rosbag2Dataset::initialize_rds(const Yaml& c)
{
  using namespace std::string_literals;

  const std::map<std::string, std::string> mapTopic2Class = {
      {"sensor_msgs/msg/Imu", "CObservationIMU"},
      {"sensor_msgs/msg/Image", "CObservationImage"},
      {"sensor_msgs/msg/CompressedImage", "CObservationImage"},
      {"sensor_msgs/msg/PointCloud2", "CObservationPointCloud"},
      {"sensor_msgs/msg/LaserScan", "CObservation2DRangeScan"},
      {"sensor_msgs/msg/NavSatFix", "CObservationGPS_NavSatFix"},
      {"gps_msgs/msg/GPSFix", "CObservationGPS_GpsFix"},
      {"geometry_msgs/msg/PoseStamped", "CObservationRobotPose"},
      {"geometry_msgs/msg/PoseWithCovarianceStamped", "CObservationRobotPose"},
  };

  MRPT_START
  ProfilerEntry tle(profiler_, "initialize");

  // Mandatory parameters:
  ENSURE_YAML_ENTRY_EXISTS(c, "params");
  const auto cfg = c["params"];
  MRPT_LOG_DEBUG_STREAM("Initializing with these params:\n" << cfg);

  // rosbag_filename: accepts a scalar string or a YAML sequence of strings.
  // Empty entries in a sequence are silently skipped (convenient for optional
  // extra bags whose env var resolves to "").
  ENSURE_YAML_ENTRY_EXISTS(cfg, "rosbag_filename");
  {
    const auto& node = cfg["rosbag_filename"];
    if (node.isScalar())
    {
      auto s = node.as<std::string>();
      if (!s.empty())
      {
        rosbag_filenames_.push_back(s);
      }
    }
    else
    {
      ASSERT_(node.isSequence());
      for (const auto& entry : node.asSequence())
      {
        auto s = entry.as<std::string>();
        if (!s.empty())
        {
          rosbag_filenames_.push_back(s);
        }
      }
    }
  }
  ASSERTMSG_(!rosbag_filenames_.empty(), "'rosbag_filename' resolved to an empty list of paths");

  YAML_LOAD_MEMBER_OPT(time_warp_scale, double);
  // Optional user override for all bags; stored separately from the per-bag
  // auto-detected ids. Old YAML key kept for backwards compatibility.
  if (cfg.has("rosbag_storage_id"))
  {
    rosbag_storage_id_override_ = cfg["rosbag_storage_id"].as<std::string>();
  }
  YAML_LOAD_MEMBER_OPT(rosbag_serialization, std::string);
  YAML_LOAD_MEMBER_OPT(base_link_frame_id, std::string);
  YAML_LOAD_MEMBER_OPT(tf_topic, std::string);
  YAML_LOAD_MEMBER_OPT(tf_static_topic, std::string);

  ASSERTMSG_(!tf_topic_.empty(), "'tf_topic' must not be empty");
  ASSERTMSG_(!tf_static_topic_.empty(), "'tf_static_topic' must not be empty");
  ASSERTMSG_(tf_topic_ != tf_static_topic_, "'tf_topic' and 'tf_static_topic' must differ");

  YAML_LOAD_MEMBER_OPT(read_ahead_length, size_t);
  YAML_LOAD_MEMBER_OPT(max_duration_sec, double);
  paused_ = cfg.getOrDefault<bool>("start_paused", paused_);

  // Validate all bag paths, auto-detect storage ids, and count total messages.
  bagMessageCount_ = 0;
  per_bag_msg_counts_.clear();
  rosbag_storage_ids_.clear();
  std::map<std::string, rosbag2_storage::TopicMetadata> all_topics_by_name;

  // Time span covered by all input bags, to report the total dataset duration:
  using bag_time_point = std::chrono::time_point<std::chrono::high_resolution_clock>;
  std::optional<std::pair<bag_time_point, bag_time_point>> bagsTimeSpan;

  for (const auto& path : rosbag_filenames_)
  {
    const bool pathIsDir  = mrpt::system::directoryExists(path);
    const bool pathIsFile = !pathIsDir && mrpt::system::fileExists(path);
    ASSERTMSG_(
        pathIsFile || pathIsDir, "'"s + path + "' is neither an existing directory nor a file."s);

    const std::string storageId = detectStorageId(path, rosbag_storage_id_override_);
    rosbag_storage_ids_.push_back(storageId);

    // Open the bag just to read its message count, then let it close.
    {
      rosbag2_storage::StorageOptions so;
      so.uri        = path;
      so.storage_id = storageId;
      rosbag2_cpp::ConverterOptions co;
      co.input_serialization_format  = rosbag_serialization_;
      co.output_serialization_format = rosbag_serialization_;
      auto tmpReader                 = std::make_shared<rosbag2_cpp::readers::SequentialReader>();
      tmpReader->open(so, co);
      for (const auto& t : tmpReader->get_all_topics_and_types())
      {
        auto [it, inserted] = all_topics_by_name.emplace(t.name, t);
        ASSERTMSG_(
            inserted || it->second.type == t.type,
            "Topic '"s + t.name + "' has inconsistent types across input bags"s);
      }
      const auto&  md  = tmpReader->get_metadata();
      const size_t cnt = static_cast<size_t>(md.message_count);
      per_bag_msg_counts_.push_back(cnt);
      bagMessageCount_ += cnt;

      // Accumulate the total time span covered by all input bags:
      if (cnt > 0)
      {
        const auto tIni = md.starting_time;
        const auto tEnd = tIni + md.duration;
        if (!bagsTimeSpan)
        {
          bagsTimeSpan = {tIni, tEnd};
        }
        else
        {
          bagsTimeSpan->first  = std::min(bagsTimeSpan->first, tIni);
          bagsTimeSpan->second = std::max(bagsTimeSpan->second, tEnd);
        }
      }
      MRPT_LOG_INFO_STREAM(
          "Bag '" << path << "': " << cnt << " messages (storage: " << storageId << ")");
    }
  }

  if (rosbag_filenames_.size() > 1)
  {
    MRPT_LOG_INFO_STREAM(
        "Total messages across " << rosbag_filenames_.size() << " bags: " << bagMessageCount_);
  }

  if (bagsTimeSpan)
  {
    dataset_total_time_ =
        std::chrono::duration<double>(bagsTimeSpan->second - bagsTimeSpan->first).count();
  }

  // Open the first bag to read topic metadata and start replay.
  openBag(0);

  std::vector<rosbag2_storage::TopicMetadata> topics;
  topics.reserve(all_topics_by_name.size());
  for (const auto& [_, topic] : all_topics_by_name)
  {
    topics.push_back(topic);
  }

  MRPT_LOG_INFO_STREAM(
      "List of topics found across " << rosbag_filenames_.size() << " bag(s) (" << bagMessageCount_
                                     << " msgs total):");

  // Build map: topic name -> type:
  std::map<std::string, std::string> topic2type;

  for (const auto& t : topics)
  {
    topic2type[t.name] = t.type;

    MRPT_LOG_INFO_STREAM(" " << t.name << " (" << t.type << ")");
  }

  read_ahead_.clear();
  read_ahead_.resize(bagMessageCount_);
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

    for (const auto& t : topics)
    {
      auto itType = mapTopic2Class.find(t.type);
      if (itType == mapTopic2Class.end())
      {
        MRPT_LOG_INFO_FMT(
            "- Skipped %25s (%30s): no known mapping to MOLA", t.name.c_str(), t.type.c_str());
        continue;
      }

      mrpt::containers::yaml s = mrpt::containers::yaml::Map();

      s["topic"] = t.name;
      s["type"]  = itType->second;

      sensorsYaml.push_back(s);

      MRPT_LOG_INFO_FMT(
          "- ADDED   %25s (%30s): as %s", t.name.c_str(), t.type.c_str(), itType->second.c_str());
    }
  }

  // Start creating topic observers for /tf and all sensors.
  // Topic names are configurable (default /tf and /tf_static) so bags recorded
  // under a ROS namespace (e.g. /robot1/tf) can be read too.
  lookup_[tf_topic_].emplace_back([this](const rosbag2_storage::SerializedBagMessage& rosmsg)
                                  { return toTf<false>(rosmsg); });
  lookup_[tf_static_topic_].emplace_back([this](const rosbag2_storage::SerializedBagMessage& rosmsg)
                                         { return toTf<true>(rosmsg); });

  for (auto& sensorNode : sensorsYaml.asSequence())
  {
    const mrpt::containers::yaml sensor(sensorNode);
    const auto                   topic = sensor["topic"].as<std::string>();

    std::string sensorLabel = topic;
    if (sensor.has("sensorLabel"))
    {
      sensorLabel = sensor["sensorLabel"].as<std::string>();
    }

    // Map to MOLA class: auto or manual:
    std::string sensorType;

    if (sensor.has("type"))
    {
      sensorType = sensor["type"].as<std::string>();
    }
    else
    {
      bool topic_is_optional = false;
      if (sensor.has("is_optional") && sensor["is_optional"].as<bool>())
      {
        topic_is_optional = true;
      }

      if (topic2type.count(topic) == 0)
      {
        if (!topic_is_optional)
        {
          THROW_EXCEPTION_FMT(
              "'sensors' contains topic '%s' which is not found in the rosbag and is not marked as "
              "'is_optional'!",
              topic.c_str());
        }
      }
      else
      {
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

        MRPT_LOG_INFO_FMT(
            "Topic '%s' listed in 'sensors' with automatic mapping, determined to be '%s'",
            topic.c_str(), sensorType.c_str());
      }
    }

    // Optional: fixed sensorPose (then ignores/don't need "tf" data):
    std::optional<mrpt::poses::CPose3D> fixedSensorPose;
    if (sensor.has("fixed_sensor_pose") &&
        (!sensor.has("use_fixed_sensor_pose") || sensor["use_fixed_sensor_pose"].as<bool>()))
    {
      fixedSensorPose = mrpt::poses::CPose3D::FromString(
          "["s + sensor["fixed_sensor_pose"].as<std::string>() + "]"s);
    }

    // Optional: override this observation's timestamp with the bag's own
    // recv/storage time instead of the ROS message header stamp. Default
    // false. Useful for drivers that stamp messages with a monotonic/uptime
    // clock instead of wall-clock epoch time (seen in the wild for some IMU
    // drivers), which otherwise trips the "mis-timestamped sensors" time
    // reference reset (huge jump between this sensor and others).
    const bool useBagRecvTimeAsTimestamp = sensor.has("use_bag_recv_time_as_timestamp") &&
                                           sensor["use_bag_recv_time_as_timestamp"].as<bool>();

#if 0  // TODO ?
			else if (sensorType == "CObservation3DRangeScan")
			{
				bool rangeIsDepth = sensor.has("rangeIsDepth")
										? sensor["rangeIsDepth"].as<bool>()
										: true;
				auto callback = [=](const sensor_msgs::Image::Ptr& image,
									const sensor_msgs::CameraInfo::Ptr& info) {
					return toRangeImage(sensorName, image, info, rangeIsDepth);
				};
			}
#endif

    // TODO: Remove the #else branch once mrpt_ros2_bridge > 3.4.0 is generally available

    using rosbag2_storage::SerializedBagMessage;

    // Compressed images (sensor_msgs/msg/CompressedImage, e.g. an
    // "image/compressed" topic) need their own decoder: mrpt_ros_bridge's
    // rosbag2ToImage() only understands the raw, uncompressed
    // sensor_msgs/msg/Image wire format. Dispatch on the topic's actual
    // wire type, independently of the mrpt_ros_bridge version available.
    const bool isCompressedImageTopic =
        topic2type.count(topic) != 0 && topic2type.at(topic) == "sensor_msgs/msg/CompressedImage";

    if (sensorType == "CObservationImage" && isCompressedImageTopic)
    {
      auto callback = [this, sensorLabel, fixedSensorPose,
                       useBagRecvTimeAsTimestamp](const SerializedBagMessage& m) -> Obs
      {
        return catchExceptions(
            [this, sensorLabel, fixedSensorPose, useBagRecvTimeAsTimestamp, m]() -> Obs
            {
              Obs obs = toCompressedImage(sensorLabel, m, fixedSensorPose);
              if (useBagRecvTimeAsTimestamp)
              {
                const auto recvTimestamp = mrpt::Clock::fromDouble(
                    1e-9 * static_cast<double>(bagMessageRecvTimestampNs(m)));
                for (auto& o : obs)
                {
                  if (o)
                  {
                    o->timestamp = recvTimestamp;
                  }
                }
              }
              return obs;
            });
      };
      MRPT_LOG_INFO_STREAM("Installing callback for topic '" << topic << "' (compressed image)");
      lookup_[topic].emplace_back(callback);
      continue;
    }

#if MRPT_ROS2_BRIDGE_VERSION >= 0x030400
    // Map sensor type → rosbag2 conversion function
    using ConvFunc = std::function<Obs(
        std::string_view, const SerializedBagMessage&, tf2::BufferCore&, const std::string&,
        const std::optional<mrpt::poses::CPose3D>&)>;

    static const std::map<std::string, ConvFunc> converters = {
        {"CObservationPointCloud", &mrpt::ros2bridge::rosbag2ToPointCloud2},
        {"CObservationImage", &mrpt::ros2bridge::rosbag2ToImage},
        {"CObservation2DRangeScan", &mrpt::ros2bridge::rosbag2ToLidar2D},
        {"CObservationRotatingScan", &mrpt::ros2bridge::rosbag2ToRotatingScan},
        {"CObservationIMU", &mrpt::ros2bridge::rosbag2ToIMU},
        {
            "CObservationGPS_NavSatFix",
            static_cast<ConvFunc>(
                [](std::string_view                             sensor_label,
                   const rosbag2_storage::SerializedBagMessage& rosmsg, tf2::BufferCore& tfBuffer,
                   const std::string&                         base_link_frame,
                   const std::optional<mrpt::poses::CPose3D>& fixed_sensor_pose)
                {
                  return mrpt::ros2bridge::rosbag2ToGPS(
                      sensor_label, rosmsg, tfBuffer, base_link_frame, fixed_sensor_pose);
                }),
        },
        {
            "CObservationGPS_GpsFix",
            static_cast<ConvFunc>(
                [](std::string_view                             sensor_label,
                   const rosbag2_storage::SerializedBagMessage& rosmsg, tf2::BufferCore& tfBuffer,
                   const std::string&                         base_link_frame,
                   const std::optional<mrpt::poses::CPose3D>& fixed_sensor_pose) -> Obs
                {
#if MRPT_ROS2_BRIDGE_VERSION >= 0x030500
                  return mrpt::ros2bridge::rosbag2ToGPS(
                      sensor_label, rosmsg, tfBuffer, base_link_frame, fixed_sensor_pose, true);
#else
                  THROW_EXCEPTION("mrpt_ros_bridge >=3.5.0 required for GpsFix messages");
#endif
                }),
        },
    };

    if (auto it = converters.find(sensorType); it != converters.end())
    {
      auto convFn   = it->second;
      auto callback = [this, sensorLabel, fixedSensorPose, useBagRecvTimeAsTimestamp,
                       convFn](const SerializedBagMessage& m) -> Obs
      {
        return catchExceptions(
            [this, sensorLabel, fixedSensorPose, useBagRecvTimeAsTimestamp, convFn, m]() -> Obs
            {
              Obs obs = convFn(sensorLabel, m, *tfBuffer_, base_link_frame_id_, fixedSensorPose);
              if (useBagRecvTimeAsTimestamp)
              {
                const auto recvTimestamp = mrpt::Clock::fromDouble(
                    1e-9 * static_cast<double>(bagMessageRecvTimestampNs(m)));
                for (auto& o : obs)
                {
                  if (o)
                  {
                    o->timestamp = recvTimestamp;
                  }
                }
              }
              return obs;
            });
      };
      MRPT_LOG_INFO_STREAM("Installing callback for topic '" << topic << "'");
      lookup_[topic].emplace_back(callback);
    }

    // This one has a different signature has to be handled apart:
    if (sensorType == "CObservationOdometry")
    {
      auto callback = [this, sensorLabel, fixedSensorPose,
                       useBagRecvTimeAsTimestamp](const SerializedBagMessage& m) -> Obs
      {
        return catchExceptions(
            [sensorLabel, fixedSensorPose, useBagRecvTimeAsTimestamp, m]() -> Obs
            {
              Obs obs = mrpt::ros2bridge::rosbag2ToOdometry(sensorLabel, m);
              if (useBagRecvTimeAsTimestamp)
              {
                const auto recvTimestamp = mrpt::Clock::fromDouble(
                    1e-9 * static_cast<double>(bagMessageRecvTimestampNs(m)));
                for (auto& o : obs)
                {
                  if (o)
                  {
                    o->timestamp = recvTimestamp;
                  }
                }
              }
              return obs;
            });
      };
      MRPT_LOG_INFO_STREAM("Installing callback for topic '" << topic << "'");
      lookup_[topic].emplace_back(callback);
    }

    // Also a different signature: it needs the topic's ROS message type, since
    // several of them map to this one MRPT class.
    if (sensorType == "CObservationRobotPose")
    {
      const std::string rosMsgType = topic2type.count(topic) ? topic2type.at(topic) : std::string();
      auto              callback   = [this, sensorLabel, fixedSensorPose,
                       rosMsgType](const SerializedBagMessage& m) -> Obs
      {
        return catchExceptions(
            [this, sensorLabel, m, rosMsgType, fixedSensorPose]()
            { return toRobotPose(sensorLabel, m, rosMsgType, fixedSensorPose); });
      };
      MRPT_LOG_INFO_STREAM("Installing callback for topic '" << topic << "'");
      lookup_[topic].emplace_back(callback);
    }
#else
    // To be removed:
    if (sensorType == "CObservationPointCloud")
    {
      auto callback = [=](const rosbag2_storage::SerializedBagMessage& m)
      { return catchExceptions([=]() { return toPointCloud2(sensorLabel, m, fixedSensorPose); }); };
      lookup_[topic].emplace_back(callback);
    }
    else if (sensorType == "CObservationImage")
    {
      auto callback = [=](const rosbag2_storage::SerializedBagMessage& m)
      { return catchExceptions([=]() { return toImage(sensorLabel, m, fixedSensorPose); }); };
      lookup_[topic].emplace_back(callback);
    }
    else if (sensorType == "CObservation2DRangeScan")
    {
      auto callback = [=](const rosbag2_storage::SerializedBagMessage& m)
      { return catchExceptions([=]() { return toLidar2D(sensorLabel, m, fixedSensorPose); }); };

      lookup_[topic].emplace_back(callback);
    }
    else if (sensorType == "CObservationRotatingScan")
    {
      auto callback = [=](const rosbag2_storage::SerializedBagMessage& m) {
        return catchExceptions([=]() { return toRotatingScan(sensorLabel, m, fixedSensorPose); });
      };
      lookup_[topic].emplace_back(callback);
    }
    else if (sensorType == "CObservationIMU")
    {
      auto callback = [=](const rosbag2_storage::SerializedBagMessage& m)
      { return catchExceptions([=]() { return toIMU(sensorLabel, m, fixedSensorPose); }); };
      lookup_[topic].emplace_back(callback);
    }
    else if (sensorType == "CObservationGPS_NavSatFix")
    {
      auto callback = [=](const rosbag2_storage::SerializedBagMessage& m)
      { return catchExceptions([=]() { return toGPS(sensorLabel, m, fixedSensorPose); }); };
      lookup_[topic].emplace_back(callback);
    }
    else if (sensorType == "CObservationOdometry")
    {
      auto callback = [=](const rosbag2_storage::SerializedBagMessage& m)
      { return catchExceptions([=]() { return toOdometry(sensorLabel, m); }); };
      lookup_[topic].emplace_back(callback);
    }
    else if (sensorType == "CObservationRobotPose")
    {
      const std::string rosMsgType = topic2type.count(topic) ? topic2type.at(topic) : std::string();
      auto              callback   = [this, sensorLabel, fixedSensorPose,
                       rosMsgType](const rosbag2_storage::SerializedBagMessage& m)
      {
        return catchExceptions(
            [this, sensorLabel, m, rosMsgType, fixedSensorPose]()
            { return toRobotPose(sensorLabel, m, rosMsgType, fixedSensorPose); });
      };
      lookup_[topic].emplace_back(callback);
    }
#endif

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
  ProfilerEntry tle(profiler_, "spinOnce");

  const auto tNow = mrpt::Clock::now();

  // Starting time:
  if (!last_play_wallclock_time_)
  {
    last_play_wallclock_time_ = tNow;
  }

  // get current replay time:
  auto         lckUIVars       = mrpt::lockHelper(dataset_ui_mtx_);
  const double time_warp_scale = time_warp_scale_;
  const bool   paused          = paused_;
  const auto   teleport_here   = teleport_here_;
  teleport_here_.reset();
  lckUIVars.unlock();

  double dt = mrpt::system::timeDifference(*last_play_wallclock_time_, tNow) * time_warp_scale;
  last_play_wallclock_time_ = tNow;

  // const double t0 = mrpt::Clock::toDouble(rawlog_begin_time_);

  if (!rosbag_begin_time_ && bagMessageCount_ > 0)
  {
    doReadAhead(0, true /* skip read ahead buffer */);
    rosbag_begin_time_ = read_ahead_.at(0)->timestamp;
  }

  // override by an special teleport order?
  if (teleport_here.has_value() && *teleport_here < bagMessageCount_)
  {
    if (*teleport_here > rosbag_next_idx_write_)
    {
      MRPT_LOG_INFO_STREAM("Request to fast-forward ('teleport') to timestep: " << *teleport_here);

      rosbag_next_idx_ = *teleport_here;
      doReadAhead(rosbag_next_idx_, true /* skip read ahead buffer */);

      // this will force a reset with the first valid timestamp.
      last_dataset_time_ = 0;
    }
    else
    {
      MRPT_LOG_WARN_STREAM(
          "IGNORING order to go backwards in time to index="
          << *teleport_here << " due to limitation of serialized rosbag2 reader.");
    }
  }
  else
  {
    if (paused)
    {
      return;
    }
    // move forward replayed dataset time:
    last_dataset_time_ += dt;
  }

  if (rosbag_next_idx_ >= read_ahead_.size())
  {
    onDatasetPlaybackEnds();  // notify base class

    MRPT_LOG_THROTTLE_INFO(
        10.0,
        "End of dataset reached! Nothing else to publish (CTRL+C to "
        "quit)");
    return;
  }

  if (max_duration_sec_ > 0 && last_dataset_time_ > max_duration_sec_)
  {
    onDatasetPlaybackEnds();  // notify base class

    MRPT_LOG_THROTTLE_INFO_FMT(
        10.0,
        "max_duration_sec (%.1f s) reached! Nothing else to publish (CTRL+C "
        "to quit)",
        max_duration_sec_);
    return;
  }

  MRPT_LOG_THROTTLE_INFO_FMT(
      5.0, "Dataset replay progress: %lu / %lu  (%4.02f%%)",
      static_cast<unsigned long>(rosbag_next_idx_), static_cast<unsigned long>(bagMessageCount_),
      (100.0 * rosbag_next_idx_) / bagMessageCount_);

  // Publish observations up to current time:
  for (;;)
  {
    if (rosbag_next_idx_ >= rosbag_next_idx_write_)
    {
      doReadAhead(rosbag_next_idx_);
    }

    // EOF?
    if (rosbag_next_idx_ >= read_ahead_.size())
    {
      break;
    }

    // current dataset entry:
    auto& de = read_ahead_.at(rosbag_next_idx_);
    ASSERT_(de.has_value());

    // Already past the time?
    // First rawlog timestamp?
    if (auto& de_tim = de->timestamp; de_tim)
    {
      if (!rosbag_begin_time_)
      {
        rosbag_begin_time_ = de_tim.value();
      }

      double thisTim = timeDifference(*rosbag_begin_time_, de_tim.value());

      // mechanism to detect mis-timestamped datasets:
      // e.g. good sensors mixed with LiDARs with timestamps starting
      //      in UNIX epoch.
      if (std::abs(thisTim - last_dataset_time_) > 1e9)
      {
        rosbag_begin_time_ = de_tim.value();
        thisTim            = .0;
        last_dataset_time_ = thisTim;

        MRPT_LOG_THROTTLE_WARN(
            2.0,
            "Apparently mis-timestamped sensors: resetting time "
            "reference. Please, fix your sensor timestamps.");
      }

      // Reset time after a "teleport"?
      if (last_dataset_time_ == 0)
      {
        last_dataset_time_ = thisTim;
      }

      // end of playback for now?
      if (last_dataset_time_ < thisTim)
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
              "Starting streaming of '" << obs->sensorLabel << "' ("
                                        << obs->GetRuntimeClass()->className
                                        << ") from the rosbag");
        }

        MRPT_LOG_DEBUG_STREAM(
            "Publishing " << obs->GetRuntimeClass()->className
                          << " sensorLabel: " << obs->sensorLabel << " for t=" << last_dataset_time_
                          << " observation timestamp="
                          << mrpt::system::dateTimeLocalToString(obs->timestamp));
      }
    }

    // Move on:
    rosbag_next_idx_++;
  }

  {
    auto lck = mrpt::lockHelper(dataset_ui_mtx_);

    last_used_tim_index_ = rosbag_next_idx_;
    ui_dataset_time_     = last_dataset_time_;
  }

  MRPT_END
}

std::string Rosbag2Dataset::detectStorageId(
    const std::string& path, const std::string& user_override)
{
  using namespace std::string_literals;

  if (!user_override.empty())
  {
    return user_override;
  }

  const bool isDir = mrpt::system::directoryExists(path);

  if (!isDir)
  {
    auto ext = mrpt::system::extractFileExtension(path);
    if (ext == "mcap")
    {
      return "mcap";
    }
    else if (ext == "db3")
    {
      return "sqlite3";
    }
    else
    {
      THROW_EXCEPTION_FMT(
          "Argument 'rosbag_storage_id' was not provided and could not determine the rosbag2 "
          "format from unknown extension of file '%s'",
          path.c_str());
    }
  }

  const std::string metadata_yaml = mrpt::system::pathJoin({path, "metadata.yaml"});
  ASSERT_FILE_EXISTS_(metadata_yaml);
  const auto metadata = mrpt::containers::yaml::FromFile(metadata_yaml);
  ASSERT_(metadata.has("rosbag2_bagfile_information"));
  ASSERT_(metadata["rosbag2_bagfile_information"].has("storage_identifier"));
  return metadata["rosbag2_bagfile_information"]["storage_identifier"].as<std::string>();
}

void Rosbag2Dataset::openBag(size_t bag_idx)
{
  using namespace std::string_literals;

  ASSERT_LT_(bag_idx, rosbag_filenames_.size());

  current_bag_idx_ = bag_idx;

  rosbag2_storage::StorageOptions storage_options;
  storage_options.uri        = rosbag_filenames_[bag_idx];
  storage_options.storage_id = rosbag_storage_ids_[bag_idx];

  rosbag2_cpp::ConverterOptions converter_options;
  converter_options.input_serialization_format  = rosbag_serialization_;
  converter_options.output_serialization_format = rosbag_serialization_;

  MRPT_LOG_INFO_STREAM(
      "Opening bag " << (bag_idx + 1) << "/" << rosbag_filenames_.size() << ": "
                     << storage_options.uri);

  reader_ = std::make_shared<rosbag2_cpp::readers::SequentialReader>();
  reader_->open(storage_options, converter_options);
}

void Rosbag2Dataset::doReadAhead(const std::optional<size_t>& requestedIndex, bool skipBufferAhead)
{
  MRPT_START

  ASSERT_(initialized_);

  // ensure we have observation data at the desired read point, plus a few
  // more:
  const auto startIdx = rosbag_next_idx_write_;

  ASSERT_GT_(read_ahead_length_, 0);

  // End of read segment:
  size_t endIdx = 0;
  if (requestedIndex)
  {
    if (skipBufferAhead)
    {
      endIdx = *requestedIndex;
    }
    else
    {
      endIdx = *requestedIndex + read_ahead_length_;
    }
  }
  else
  {
    endIdx = rosbag_next_idx_ + read_ahead_length_;
  }

  mrpt::saturate<size_t>(endIdx, 0, read_ahead_.size() - 1);

  for (size_t idx = startIdx; idx <= endIdx; idx++)
  {
    unload_queue_.push_back(idx);  // mark as recently accessed

    if (read_ahead_.at(idx).has_value())
    {
      continue;  // already read:
    }

    // serialized data
    ASSERT_EQUAL_(rosbag_next_idx_write_, idx);
    rosbag_next_idx_write_++;

    // Switch to the next bag when the current one is exhausted (loop to skip empty bags).
    while (!reader_->has_next() && current_bag_idx_ + 1 < rosbag_filenames_.size())
    {
      openBag(current_bag_idx_ + 1);
    }
    ASSERT_(reader_->has_next());

    auto serialized_message = reader_->read_next();

    if (skipBufferAhead && idx != endIdx)
    {
      continue;
    }

    SF::Ptr sf = to_mrpt(*serialized_message);
    ASSERT_(sf);

    DatasetEntry& de = read_ahead_.at(idx).emplace();

    de.obs = sf;

    if (!sf->empty())
    {
      de.timestamp = sf->getObservationByIndex(0)->timestamp;
    }
  }

  // and also, unload() very old observations.
  autoUnloadOldEntries();

  MRPT_END
}

// See docs in base class:
size_t Rosbag2Dataset::datasetSize() const
{
  ASSERTMSG_(initialized_, "You must call initialize() first");

  return bagMessageCount_;
}

mrpt::obs::CSensoryFrame::Ptr Rosbag2Dataset::datasetGetObservations(size_t timestep) const
{
  ASSERTMSG_(initialized_, "You must call initialize() first");

  {
    auto lck             = mrpt::lockHelper(dataset_ui_mtx_);
    last_used_tim_index_ = timestep;
  }

  auto& me = const_cast<Rosbag2Dataset&>(*this);

  me.doReadAhead(timestep);

  ASSERT_(read_ahead_.at(timestep).has_value());

  return read_ahead_.at(timestep)->obs;
}

std::optional<mola::TransformTree> Rosbag2Dataset::transform_tree(
    const std::string& root, const std::optional<mrpt::Clock::time_point>& timestamp) const
{
  // No external locking is needed here: tf2::BufferCore guards its own
  // internals, so this walk may run concurrently with the thread feeding /tf.
  if (!tfBuffer_ || !tfBuffer_->_frameExists(root))
  {
    return {};
  }

  // Build the parent -> children adjacency of the whole buffer first, so the
  // subtree below 'root' can then be walked depth-first. Each node is emitted
  // before its own children are queued, which is what gives the "parents
  // before children" order the interface promises.
  std::vector<std::string> allFrames;
  tfBuffer_->_getFrameStrings(allFrames);

  const tf2::TimePoint queryTime =
      timestamp ? tf2::TimePoint(timestamp->time_since_epoch()) : tf2::TimePoint();

  std::map<std::string, std::vector<std::string>> children;
  for (const auto& f : allFrames)
  {
    std::string parent;
    if (tfBuffer_->_getParent(f, queryTime, parent) ||
        tfBuffer_->_getParent(f, tf2::TimePoint(), parent))
    {
      children[parent].push_back(f);
    }
  }

  mola::TransformTree tree;
  tree.root      = root;
  tree.timestamp = timestamp.value_or(mrpt::Clock::now());
  tree.nodes.push_back(mola::TransformTreeNode{root, {}, mrpt::poses::CPose3D::Identity()});

  // 'visited' guards against a cyclic parent chain: tf2 reassigns a frame's
  // parent on every setTransform(), so malformed input can produce one, and
  // the walk would otherwise never terminate.
  std::set<std::string>    visited = {root};
  std::vector<std::string> pending = {root};
  while (!pending.empty())
  {
    const std::string frame = pending.back();
    pending.pop_back();

    const auto itChildren = children.find(frame);
    if (itChildren == children.end())
    {
      continue;
    }

    for (const auto& child : itChildren->second)
    {
      mrpt::poses::CPose3D childInRoot;
      try
      {
        // Prefer the requested time, but fall back to the latest available
        // transform: /tf is streamed as the bag plays, so a consumer asking
        // about "now" can easily be slightly ahead of the buffered data.
        geometry_msgs::msg::TransformStamped tfMsg;
        try
        {
          tfMsg = tfBuffer_->lookupTransform(root, child, queryTime);
        }
        catch (const tf2::TransformException&)
        {
          tfMsg = tfBuffer_->lookupTransform(root, child, tf2::TimePoint());
        }

        tf2::Transform t;
        tf2::fromMsg(tfMsg.transform, t);
        childInRoot = mrpt::ros2bridge::fromROS(t);
      }
      catch (const tf2::TransformException&)
      {
        // A frame with no usable transform at this time is skipped, together
        // with its own subtree (it has no resolvable pose to draw it at).
        continue;
      }

      if (!visited.insert(child).second)
      {
        continue;
      }

      tree.nodes.push_back(mola::TransformTreeNode{child, frame, childInRoot});
      pending.push_back(child);
    }
  }

  return tree;
}

// TODO: Remove once this package is well available
#if MRPT_ROS2_BRIDGE_VERSION < 0x030400
bool Rosbag2Dataset::findOutSensorPose(
    mrpt::poses::CPose3D& des, const std::string& frame, const std::string& referenceFrame,
    const std::optional<mrpt::poses::CPose3D>& fixedSensorPose, const std::string_view label)
{
  if (fixedSensorPose)
  {
    des = fixedSensorPose.value();
    return true;
  }

  try
  {
    geometry_msgs::msg::TransformStamped ref_to_trgFrame =
        tfBuffer_->lookupTransform(referenceFrame, frame, {} /*latest value*/);

    tf2::Transform tf;
    tf2::fromMsg(ref_to_trgFrame.transform, tf);
    des = mrpt::ros2bridge::fromROS(tf);

    MRPT_LOG_DEBUG_FMT(
        "[findOutSensorPose] Found pose %s -> %s: %s", referenceFrame.c_str(), frame.c_str(),
        des.asString().c_str());

    return true;
  }
  catch (const tf2::TransformException& ex)
  {
    MRPT_LOG_ERROR_STREAM(
        "findOutSensorPose (label='" << label << "', " << frame << "<-" << referenceFrame
                                     << "): " << ex.what());
    return false;
  }
}

Rosbag2Dataset::Obs Rosbag2Dataset::toPointCloud2(
    std::string_view label, const rosbag2_storage::SerializedBagMessage& rosmsg,
    const std::optional<mrpt::poses::CPose3D>& fixedSensorPose)
{
  rclcpp::SerializedMessage                                   serMsg(*rosmsg.serialized_data);
  static rclcpp::Serialization<sensor_msgs::msg::PointCloud2> serializer;

  sensor_msgs::msg::PointCloud2 pts;
  serializer.deserialize_message(&serMsg, &pts);

  auto ptsObs         = mrpt::obs::CObservationPointCloud::Create();
  ptsObs->sensorLabel = label;
  ptsObs->timestamp   = mrpt::ros2bridge::fromROS(pts.header.stamp);

  bool sensorPoseOK = findOutSensorPose(
      ptsObs->sensorPose, pts.header.frame_id, base_link_frame_id_, fixedSensorPose, label);
  ASSERT_(sensorPoseOK);

  // Convert points:
  std::set<std::string> fields = mrpt::ros2bridge::extractFields(pts);

  // We need X Y Z:
  if (0 == fields.count("x") || 0 == fields.count("y") || 0 == fields.count("z"))
  {
    return {};
  }

  // Generic map:
  if (fields.count("ring") != 0 || fields.count("time") != 0 || fields.count("timestamp") != 0 ||
      fields.count("t") != 0 || fields.count("intensity") != 0)
  {
    auto mrptPts       = mrpt::maps::CGenericPointsMap::Create();
    ptsObs->pointcloud = mrptPts;

    if (!mrpt::ros2bridge::fromROS(pts, *mrptPts))
    {
      THROW_EXCEPTION("Could not convert pointcloud from ROS to CGenericPointsMap");
    }

    // Fix timestamps for Livox driver:
    // It uses doubles for timestamps, but they are actually nanoseconds!
    auto* ts =
        mrptPts->getPointsBufferRef_float_field(mrpt::maps::CPointsMap::POINT_FIELD_TIMESTAMP);
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

    // converted ok:
    return {ptsObs};
  }

  // Simple XYZ map
  {
    auto mrptPts       = mrpt::maps::CSimplePointsMap::Create();
    ptsObs->pointcloud = mrptPts;

    if (!mrpt::ros2bridge::fromROS(pts, *mrptPts))
    {
      THROW_EXCEPTION("Could not convert pointcloud from ROS to CSimplePointsMap");
    }
  }

  return {ptsObs};
}

Rosbag2Dataset::Obs Rosbag2Dataset::toLidar2D(
    std::string_view label, const rosbag2_storage::SerializedBagMessage& rosmsg,
    const std::optional<mrpt::poses::CPose3D>& fixedSensorPose)
{
  rclcpp::SerializedMessage                                 serMsg(*rosmsg.serialized_data);
  static rclcpp::Serialization<sensor_msgs::msg::LaserScan> serializer;

  sensor_msgs::msg::LaserScan scan;
  serializer.deserialize_message(&serMsg, &scan);

  auto scanObs = mrpt::obs::CObservation2DRangeScan::Create();

  // Extract sensor pose from tf frames, if enabled:
  mrpt::poses::CPose3D sensorPose;
  mrpt::ros2bridge::fromROS(scan, sensorPose, *scanObs);

  scanObs->sensorLabel = label;
  scanObs->timestamp   = mrpt::ros2bridge::fromROS(scan.header.stamp);

  bool sensorPoseOK = findOutSensorPose(
      scanObs->sensorPose, scan.header.frame_id, base_link_frame_id_, fixedSensorPose, label);
  ASSERT_(sensorPoseOK);

  return {scanObs};
}

Rosbag2Dataset::Obs Rosbag2Dataset::toRotatingScan(
    std::string_view label, const rosbag2_storage::SerializedBagMessage& rosmsg,
    const std::optional<mrpt::poses::CPose3D>& fixedSensorPose)
{
  rclcpp::SerializedMessage                                   serMsg(*rosmsg.serialized_data);
  static rclcpp::Serialization<sensor_msgs::msg::PointCloud2> serializer;

  sensor_msgs::msg::PointCloud2 pts;
  serializer.deserialize_message(&serMsg, &pts);

  // Convert points:
  std::set<std::string> fields = mrpt::ros2bridge::extractFields(pts);

  // We need X Y Z:
  if (!fields.count("x") || !fields.count("y") || !fields.count("z") || !fields.count("ring"))
  {
    return {};
  }

  // As a structured 2D range images, if we have ring numbers:
  auto                       obsRotScan = mrpt::obs::CObservationRotatingScan::Create();
  const mrpt::poses::CPose3D sensorPose;

  if (!mrpt::ros2bridge::fromROS(pts, *obsRotScan, sensorPose))
  {
    THROW_EXCEPTION(
        "Could not convert pointcloud from ROS to "
        "CObservationRotatingScan. Trying another format.");
  }

  obsRotScan->sensorLabel = label;
  obsRotScan->timestamp   = mrpt::ros2bridge::fromROS(pts.header.stamp);

  bool sensorPoseOK = findOutSensorPose(
      obsRotScan->sensorPose, pts.header.frame_id, base_link_frame_id_, fixedSensorPose, label);
  ASSERT_(sensorPoseOK);

  return {obsRotScan};
}

Rosbag2Dataset::Obs Rosbag2Dataset::toIMU(
    std::string_view label, const rosbag2_storage::SerializedBagMessage& rosmsg,
    const std::optional<mrpt::poses::CPose3D>& fixedSensorPose)
{
  rclcpp::SerializedMessage                           serMsg(*rosmsg.serialized_data);
  static rclcpp::Serialization<sensor_msgs::msg::Imu> serializer;

  sensor_msgs::msg::Imu imu;
  serializer.deserialize_message(&serMsg, &imu);

  auto imuObs = mrpt::obs::CObservationIMU::Create();

  imuObs->sensorLabel = label;
  imuObs->timestamp   = mrpt::ros2bridge::fromROS(imu.header.stamp);

  // Convert data:
  mrpt::ros2bridge::fromROS(imu, *imuObs);

  bool sensorPoseOK = findOutSensorPose(
      imuObs->sensorPose, imu.header.frame_id, base_link_frame_id_, fixedSensorPose, label);
  ASSERT_(sensorPoseOK);

  return {imuObs};
}

Rosbag2Dataset::Obs Rosbag2Dataset::toGPS(
    std::string_view label, const rosbag2_storage::SerializedBagMessage& rosmsg,
    const std::optional<mrpt::poses::CPose3D>& fixedSensorPose)
{
  rclcpp::SerializedMessage                                 serMsg(*rosmsg.serialized_data);
  static rclcpp::Serialization<sensor_msgs::msg::NavSatFix> serializer;

  sensor_msgs::msg::NavSatFix gps;
  serializer.deserialize_message(&serMsg, &gps);

  auto gpsObs = mrpt::obs::CObservationGPS::Create();

  gpsObs->sensorLabel = label;
  gpsObs->timestamp   = mrpt::ros2bridge::fromROS(gps.header.stamp);

  // Convert data:
  mrpt::ros2bridge::fromROS(gps, *gpsObs);

  bool sensorPoseOK = findOutSensorPose(
      gpsObs->sensorPose, gps.header.frame_id, base_link_frame_id_, fixedSensorPose, label);
  ASSERT_(sensorPoseOK);

  return {gpsObs};
}

Rosbag2Dataset::Obs Rosbag2Dataset::toOdometry(
    std::string_view label, const rosbag2_storage::SerializedBagMessage& rosmsg)
{
  rclcpp::SerializedMessage                             serMsg(*rosmsg.serialized_data);
  static rclcpp::Serialization<nav_msgs::msg::Odometry> serializer;

  nav_msgs::msg::Odometry odo;
  serializer.deserialize_message(&serMsg, &odo);

  auto mrptObs = mrpt::obs::CObservationOdometry::Create();

  mrptObs->sensorLabel = label;
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
    std::string_view label, const rosbag2_storage::SerializedBagMessage& rosmsg,
    const std::optional<mrpt::poses::CPose3D>& fixedSensorPose)
{
  rclcpp::SerializedMessage                             serMsg(*rosmsg.serialized_data);
  static rclcpp::Serialization<sensor_msgs::msg::Image> serializer;

  auto image = std::make_shared<sensor_msgs::msg::Image>();
  serializer.deserialize_message(&serMsg, image.get());

  auto imgObs = mrpt::obs::CObservationImage::Create();

  imgObs->sensorLabel = label;
  imgObs->timestamp   = mrpt::ros2bridge::fromROS(image->header.stamp);

  auto cv_ptr = cv_bridge::toCvShare(image);

  imgObs->image = mrpt::img::CImage(cv_ptr->image, mrpt::img::DEEP_COPY);

  bool sensorPoseOK = findOutSensorPose(
      imgObs->cameraPose, image->header.frame_id, base_link_frame_id_, fixedSensorPose, label);
  ASSERT_(sensorPoseOK);

  return {imgObs};
}
#endif

namespace
{
/// A source that leaves `pose.covariance` all zeros is not claiming a perfect
/// measurement, it is not filling the field in. Substitute something usable so
/// downstream fusion does not read it as infinite confidence.
void fillInDefaultPoseCovariance(mrpt::poses::CPose3DPDFGaussian& p)
{
  if (p.cov != mrpt::math::CMatrixDouble66::Zero()) return;

  const double sigmaXYZ = 0.10;  // [m]
  const double sigmaAng = mrpt::DEG2RAD(2.0);  // [rad]
  for (int k = 0; k < 3; k++) p.cov(k, k) = mrpt::square(sigmaXYZ);
  for (int k = 3; k < 6; k++) p.cov(k, k) = mrpt::square(sigmaAng);
}
}  // namespace

Rosbag2Dataset::Obs Rosbag2Dataset::toRobotPose(
    std::string_view label, const rosbag2_storage::SerializedBagMessage& rosmsg,
    const std::string& rosMsgType, const std::optional<mrpt::poses::CPose3D>& fixedSensorPose)
{
  rclcpp::SerializedMessage serMsg(*rosmsg.serialized_data);

  auto mrptObs         = mrpt::obs::CObservationRobotPose::Create();
  mrptObs->sensorLabel = label;
  // Unlike CObservationOdometry, this type can carry a sensor pose, so a
  // source reported for a frame other than base_link remains usable here:
  if (fixedSensorPose.has_value()) mrptObs->sensorPose = *fixedSensorPose;

  if (rosMsgType == "nav_msgs/msg/Odometry")
  {
    static rclcpp::Serialization<nav_msgs::msg::Odometry> ser;
    nav_msgs::msg::Odometry                               msg;
    ser.deserialize_message(&serMsg, &msg);
    mrptObs->timestamp = mrpt::ros2bridge::fromROS(msg.header.stamp);
    mrptObs->pose      = mrpt::ros2bridge::fromROS(msg.pose);
  }
  else if (rosMsgType == "geometry_msgs/msg/PoseWithCovarianceStamped")
  {
    static rclcpp::Serialization<geometry_msgs::msg::PoseWithCovarianceStamped> ser;
    geometry_msgs::msg::PoseWithCovarianceStamped                               msg;
    ser.deserialize_message(&serMsg, &msg);
    mrptObs->timestamp = mrpt::ros2bridge::fromROS(msg.header.stamp);
    mrptObs->pose      = mrpt::ros2bridge::fromROS(msg.pose);
  }
  else if (rosMsgType == "geometry_msgs/msg/PoseStamped")
  {
    static rclcpp::Serialization<geometry_msgs::msg::PoseStamped> ser;
    geometry_msgs::msg::PoseStamped                               msg;
    ser.deserialize_message(&serMsg, &msg);
    mrptObs->timestamp = mrpt::ros2bridge::fromROS(msg.header.stamp);
    mrptObs->pose.mean = mrpt::ros2bridge::fromROS(msg.pose);
    // geometry_msgs/PoseStamped carries no covariance at all.
  }
  else
  {
    THROW_EXCEPTION_FMT(
        "Topic for sensorLabel '%s' was declared as 'CObservationRobotPose' but its ROS type is "
        "'%s', which is none of nav_msgs/msg/Odometry, "
        "geometry_msgs/msg/PoseWithCovarianceStamped or geometry_msgs/msg/PoseStamped.",
        std::string(label).c_str(), rosMsgType.c_str());
  }

  fillInDefaultPoseCovariance(mrptObs->pose);

  return {mrptObs};
}

Rosbag2Dataset::Obs Rosbag2Dataset::toCompressedImage(
    std::string_view label, const rosbag2_storage::SerializedBagMessage& rosmsg,
    const std::optional<mrpt::poses::CPose3D>& fixedSensorPose)
{
  rclcpp::SerializedMessage                                       serMsg(*rosmsg.serialized_data);
  static rclcpp::Serialization<sensor_msgs::msg::CompressedImage> serializer;

  sensor_msgs::msg::CompressedImage image;
  serializer.deserialize_message(&serMsg, &image);

  auto imgObs = mrpt::obs::CObservationImage::Create();

  imgObs->sensorLabel = label;
  imgObs->timestamp   = mrpt::ros2bridge::fromROS(image.header.stamp);

  auto cv_ptr = cv_bridge::toCvCopy(image, "bgr8");

  // MRPT 3.x's CImage no longer wraps cv::Mat directly (removed to drop the
  // OpenCV dependency), so copy the decoded pixels via the raw-buffer loader
  // instead; swapRedBlue=true converts cv_bridge's BGR order to MRPT's RGB.
  ASSERT_(cv_ptr->image.isContinuous());
  imgObs->image.loadFromMemoryBuffer(
      cv_ptr->image.cols, cv_ptr->image.rows, mrpt::img::CH_RGB, cv_ptr->image.data,
      /*swapRedBlue=*/true);

  if (fixedSensorPose)
  {
    imgObs->cameraPose = *fixedSensorPose;
  }
  else
  {
    try
    {
      geometry_msgs::msg::TransformStamped ref_to_trgFrame =
          tfBuffer_->lookupTransform(base_link_frame_id_, image.header.frame_id, {});

      tf2::Transform tf;
      tf2::fromMsg(ref_to_trgFrame.transform, tf);
      imgObs->cameraPose = mrpt::ros2bridge::fromROS(tf);
    }
    catch (const tf2::TransformException& ex)
    {
      THROW_EXCEPTION_FMT(
          "toCompressedImage (label='%s'): could not find sensor pose '%s' -> '%s': %s",
          std::string(label).c_str(), base_link_frame_id_.c_str(), image.header.frame_id.c_str(),
          ex.what());
    }
  }

  return {imgObs};
}

// TODO: When removed the code above, port this one too:

template <bool isStatic>
Rosbag2Dataset::Obs Rosbag2Dataset::toTf(const rosbag2_storage::SerializedBagMessage& rosmsg)
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

Rosbag2Dataset::SF::Ptr Rosbag2Dataset::to_mrpt(const rosbag2_storage::SerializedBagMessage& rosmsg)
{
  auto rets = Rosbag2Dataset::SF::Create();

  auto topic = rosmsg.topic_name;

  if (auto search = lookup_.find(topic); search != lookup_.end())
  {
    for (const auto& callback : search->second)
    {
      auto obs = callback(rosmsg);

      for (const auto& o : obs)
      {  // insert observation:
        rets->insert(o);
      }
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

Rosbag2Dataset::Obs Rosbag2Dataset::catchExceptions(const std::function<Obs()>& f)
{
  try
  {
    return f();
  }
  catch (const std::exception& e)
  {
    MRPT_LOG_ERROR_STREAM(
        "Exception while processing topic message (ignore if the error "
        "stops later on, e.g. missing /tf):\n"
        << e.what());
    return {};
  }
}

void Rosbag2Dataset::autoUnloadOldEntries() const
{
  const size_t MAX_UNLOAD_LEN = std::max<size_t>(10, 2 * read_ahead_length_);

  // unload() very old observations.
  while (unload_queue_.size() > MAX_UNLOAD_LEN)
  {
    const auto idx = unload_queue_.front();
    unload_queue_.erase(unload_queue_.begin());

    // Free memory in read-ahead buffer:
    read_ahead_.at(idx).reset();
  }
}
