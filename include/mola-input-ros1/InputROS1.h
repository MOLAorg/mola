/* -------------------------------------------------------------------------
 *   A Modular Optimization framework for Localization and mApping  (MOLA)
 * Copyright (C) 2018-2019 Jose Luis Blanco, University of Almeria
 * See LICENSE for license information.
 * ------------------------------------------------------------------------- */
/**
 * @file   InputROS1.h
 * @brief  RawDataSource from ROS1 topics
 * @author Jose Luis Blanco Claraco
 * @date   Aug 12, 2019
 */
#pragma once

#include <mola-kernel/interfaces/RawDataSourceBase.h>
#include <mrpt/serialization/CArchive.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>

namespace mola
{
/**
 *
 * \ingroup mola_input_ros1_grp */
class InputROS1 : public RawDataSourceBase
{
    DEFINE_MRPT_OBJECT(InputROS1)

   public:
    InputROS1();
    ~InputROS1() override = default;

    // See docs in base class
    void initialize(const std::string& cfg_block) override;
    void spinOnce() override;

   private:
    std::unique_ptr<ros::NodeHandle> rosnode_;
    std::vector<ros::Subscriber>     subcribers_;

    void callbackOnPointCloud2(
        const sensor_msgs::PointCloud2::ConstPtr& o,
        const std::string&                        outSensorLabel);
};

}  // namespace mola
