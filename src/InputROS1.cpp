/* -------------------------------------------------------------------------
 *   A Modular Optimization framework for Localization and mApping  (MOLA)
 * Copyright (C) 2018-2021 Jose Luis Blanco, University of Almeria
 * See LICENSE for license information.
 * ------------------------------------------------------------------------- */
/**
 * @file   InputROS1.cpp
 * @brief  RawDataSource from ROS1 topics
 * @author Jose Luis Blanco Claraco
 * @date   Aug 12, 2019
 */

/** \defgroup mola_input_ros1_grp mola_input_ros1_grp.
 * RawDataSource for datasets in MRPT rawlog format
 *
 */

#include <mola-input-ros1/InputROS1.h>
#include <mola-yaml/yaml_helpers.h>
#include <mrpt/containers/yaml.h>
#include <mrpt/core/initializer.h>
#include <mrpt/maps/CPointsMapXYZI.h>
#include <mrpt/maps/CSimplePointsMap.h>
#include <mrpt/obs/CObservationPointCloud.h>
#include <mrpt/ros1bridge/point_cloud2.h>
#include <mrpt/ros1bridge/time.h>
#include <mrpt/system/filesystem.h>

using namespace mola;

// arguments: class_name, parent_class, class namespace
IMPLEMENTS_MRPT_OBJECT(InputROS1, RawDataSourceBase, mola)

MRPT_INITIALIZER(do_register_InputROS1) { MOLA_REGISTER_MODULE(InputROS1); }

InputROS1::InputROS1() = default;

void InputROS1::initialize(const std::string& cfg_block)
{
    using namespace std::string_literals;

    MRPT_START
    ProfilerEntry tle(profiler_, "initialize");

    // Mandatory parameters:
    auto c = mrpt::containers::yaml::FromText(cfg_block);

    ENSURE_YAML_ENTRY_EXISTS(c, "params");
    auto cfg = c["params"];
    MRPT_LOG_DEBUG_STREAM("Initializing with these params:\n" << cfg);

    // Init ROS subsystem
    ros::M_string remappings;
    ros::init(remappings, "InputROS1", ros::init_options::NoSigintHandler);

    /* NodeHandle is the main access point to communications with the ROS
     * system. The first NodeHandle constructed will fully initialize this node,
     * and the last NodeHandle destructed will close down the node.
     */
    rosnode_ = std::make_unique<ros::NodeHandle>();

    // Subscribe to topics as described by MOLA YAML parameters:
    auto ds_subscribe = cfg["subscribe"];
    if (!ds_subscribe)
    {
        throw std::runtime_error(
            "No topic found for subscription under YAML entry `subscribe`. It "
            "is certainly pointless invoking this MOLA module without any "
            "topic, thus this is understood as a fatal error and will abort.");
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

        if (type == "PointCloud2")
        {
            subcribers_.emplace_back(
                rosnode_->subscribe<sensor_msgs::PointCloud2>(
                    topic_name, queue_size,
                    boost::bind(
                        &InputROS1::callbackOnPointCloud2, this, _1,
                        output_sensor_label)));
        }
        else
        {
            THROW_EXCEPTION_FMT(
                "Unhandled type=`%s` for topic=`%s`", type.c_str(),
                topic_name.c_str());
        }
    }

    MRPT_END
}  // end initialize()

void InputROS1::spinOnce()
{
    using mrpt::system::timeDifference;

    MRPT_START
    ProfilerEntry tleg(profiler_, "spinOnce");

    if (!ros::ok())
    {
        MRPT_LOG_THROTTLE_ERROR(
            5.0 /*seconds*/, "ROS1 is in error state (missing roscore?)");
        return;
    }

    // Process a single round of callbacks:
    {
        ProfilerEntry tle(profiler_, "spinOnce.ros::spinOne()");

        ros::spinOnce();
    }

    CObservation::Ptr obs;

    if (!obs) return;

    this->sendObservationsToFrontEnds(obs);

    MRPT_LOG_DEBUG_STREAM(
        "Publishing " << obs->GetRuntimeClass()->className << " sensorLabel: "
                      << obs->sensorLabel << " observation timestamp="
                      << mrpt::system::dateTimeLocalToString(obs->timestamp));

    MRPT_END
}

void InputROS1::callbackOnPointCloud2(
    const sensor_msgs::PointCloud2::ConstPtr& o,
    const std::string&                        outSensorLabel)
{
    MRPT_START
    ProfilerEntry tle(profiler_, "callbackOnPointCloud2");

    const std::set<std::string> fields = mrpt::ros1bridge::extractFields(*o);

    CObservation::Ptr obs;

    if (fields.count("intensity"))
    {
        auto p = mrpt::maps::CPointsMapXYZI::Create();
        if (!mrpt::ros1bridge::fromROS(*o, *p))
            throw std::runtime_error("Error converting ros->mrpt(?)");

        auto obs_pc         = mrpt::obs::CObservationPointCloud::Create();
        obs_pc->timestamp   = mrpt::ros1bridge::fromROS(o->header.stamp);
        obs_pc->sensorLabel = outSensorLabel;
        obs_pc->pointcloud  = p;

        obs = obs_pc;
    }

    // Send:
    if (obs) this->sendObservationsToFrontEnds(obs);

    MRPT_END
}
