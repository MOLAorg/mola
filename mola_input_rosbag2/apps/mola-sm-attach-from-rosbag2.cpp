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
 * @file   mola-sm-attach-from-rosbag2.cpp
 * @brief  Adds observations from a ROS 2 bag (e.g. camera images) to the
 *         keyframes of an existing simplemap, the closest in time to each.
 * @author Jose Luis Blanco Claraco
 * @date   2026
 */

#include <mola_input_rosbag2/Rosbag2Dataset.h>
#include <mola_kernel/utils/simplemap_attach_observations.h>
#include <mrpt/containers/yaml.h>
#include <mrpt/core/exceptions.h>
#include <mrpt/core/format.h>

#include <CLI/CLI.hpp>
#include <iostream>
#include <stdexcept>
#include <string>
#include <vector>

int main(int argc, char** argv)
{
  try
  {
    CLI::App app(
        "mola-sm-attach-from-rosbag2: adds observations from a ROS 2 bag (e.g. camera images) "
        "to the keyframes of a simplemap. Each observation goes to the keyframe closest in time "
        "(within --max-age), and each keyframe keeps the closest of them, so no observation is "
        "stored twice. Images are written to <output>_Images/.");

    std::string                  inputFile;
    std::string                  outputFile;
    std::string                  rosbag;
    std::vector<std::string>     topics;
    mola::SimplemapAttachOptions opts;
    std::string                  baseLink      = "base_link";
    std::string                  tfTopic       = "/tf";
    std::string                  tfStaticTopic = "/tf_static";

    app.add_option("-i,--input", inputFile, "Input .simplemap")->required();
    app.add_option("-o,--output", outputFile, "Output .simplemap (may be the input one)")
        ->required();
    app.add_option("--rosbag2", rosbag, "Input bag (directory or .mcap/.db3 file)")->required();
    app.add_option(
           "--topic", topics,
           "Topic to read, as TOPIC or TOPIC:LABEL (default label: the topic name). Repeatable, "
           "with a different label for each topic.")
        ->required();
    app.add_option("--max-age", opts.max_age, "Max time offset to the keyframe [s]")
        ->capture_default_str();
    app.add_option("--image-format", opts.image_format, "jpg, png or inline")
        ->capture_default_str();
    app.add_option("--jpeg-quality", opts.jpeg_quality, "JPEG quality [1-100]")
        ->capture_default_str();
    std::string fixedSensorPose;
    app.add_option(
        "--fixed-sensor-pose", fixedSensorPose,
        "Sensor pose on the vehicle as 'x y z yaw_deg pitch_deg roll_deg', for all the topics, "
        "instead of looking it up in /tf (needed for bags without /tf).");
    app.add_option("--base-link-frame-id", baseLink, "Vehicle frame, for the sensor poses")
        ->capture_default_str()
        ->envname("MOLA_TF_BASE_LINK");
    app.add_option("--tf-topic", tfTopic)->capture_default_str()->envname("MOLA_TF_TOPIC");
    app.add_option("--tf-static-topic", tfStaticTopic)
        ->capture_default_str()
        ->envname("MOLA_TF_STATIC_TOPIC");

    CLI11_PARSE(app, argc, argv);

    // The dataset configuration, built as YAML nodes: values from the command
    // line are taken literally, never expanded as ${...} or $(...).
    mrpt::containers::yaml sensors = mrpt::containers::yaml::Sequence();
    for (const auto& t : topics)
    {
      const auto        sep   = t.find(':');
      const std::string topic = t.substr(0, sep);
      const std::string label = sep == std::string::npos ? topic : t.substr(sep + 1);
      if (!opts.sensor_labels.insert(label).second)
      {
        throw std::invalid_argument(
            "Label '" + label + "' is used by more than one --topic: give each one its own label");
      }
      mrpt::containers::yaml s = mrpt::containers::yaml::Map();
      s["topic"]               = topic;
      s["sensorLabel"]         = label;
      if (!fixedSensorPose.empty())
      {
        s["fixed_sensor_pose"]     = fixedSensorPose;
        s["use_fixed_sensor_pose"] = true;
      }
      sensors.push_back(s);
    }
    mrpt::containers::yaml params = mrpt::containers::yaml::Map();
    params["rosbag_filename"]     = rosbag;
    params["base_link_frame_id"]  = baseLink;
    params["tf_topic"]            = tfTopic;
    params["tf_static_topic"]     = tfStaticTopic;
    params["sensors"]             = sensors;
    mrpt::containers::yaml cfg    = mrpt::containers::yaml::Map();
    cfg["params"]                 = params;

    auto ds = std::make_shared<mola::Rosbag2Dataset>();
    ds->setMinLoggingLevel(mrpt::system::LVL_WARN);
    ds->initialize(cfg);

    const auto r = mola::simplemap_attach_observations(
        inputFile, outputFile, *ds, opts,
        [](double f)
        { std::cout << mrpt::format("\rReading: %5.01f%%", 100.0 * f) << std::flush; });

    std::cout << "\n";
    for (const auto& [label, n] : r.attached)
    {
      std::cout << "'" << label << "': " << n << " keyframes of " << r.keyframes << "\n";
    }
    std::cout << "Observations read: " << r.observations_read << "\nSaved: " << outputFile << "\n";
    return 0;
  }
  catch (const std::exception& e)
  {
    std::cerr << "Error: " << mrpt::exception_to_str(e) << "\n";
    return 1;
  }
}
