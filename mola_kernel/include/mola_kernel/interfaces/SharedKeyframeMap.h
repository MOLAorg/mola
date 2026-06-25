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
 * @file   SharedKeyframeMap.h
 * @brief  Virtual interface for a central-map keyframe-insertion sink.
 * @author Jose Luis Blanco Claraco
 * @date   2026
 */
#pragma once

#include <mrpt/core/Clock.h>
#include <mrpt/obs/CSensoryFrame.h>
#include <mrpt/poses/CPose3DPDFGaussian.h>

#include <cstdint>
#include <optional>
#include <string>

namespace mola
{
/** Mixin interface for a central-map backend (e.g. `mola_mapper_3d`) that
 *  accepts keyframe-insertion requests from front ends (LIO/VIO), decoupled
 *  from the front end's own internal local map / odometry frame.
 *
 *  A front end detects this sink the same way it detects a `NavStateFilter`
 *  (`ExecutableBase::findService<SharedKeyframeMap>()`) and, when present,
 *  pushes keyframes using its own existing keyframing/sparsity criteria, in
 *  addition to (or instead of) writing its own `.simplemap`.
 *
 *  Implementations are required to be thread-safe.
 *
 *  \ingroup mola_kernel_interfaces_grp
 */
class SharedKeyframeMap
{
 public:
  using KeyFrameID = uint64_t;

  SharedKeyframeMap()                                    = default;
  SharedKeyframeMap(const SharedKeyframeMap&)            = default;
  SharedKeyframeMap& operator=(const SharedKeyframeMap&) = default;
  SharedKeyframeMap(SharedKeyframeMap&&)                 = default;
  SharedKeyframeMap& operator=(SharedKeyframeMap&&)      = default;
  virtual ~SharedKeyframeMap();

  /** A request from a front end to insert one keyframe into the central map. */
  struct KeyframeInsertRequest
  {
    KeyframeInsertRequest() = default;

    /** Time of the keyframe. */
    mrpt::Clock::time_point timestamp;

    /** Identifies the requesting odometry source (e.g. "odom_lidar"). One
     *  `{odom_i}` frame is created per distinct `source_frame_id`. Use a
     *  DEDICATED name here, distinct from any `frame_id` the same front end
     *  also uses for dense, every-sample `NavStateFilter::fuse_pose()` calls
     *  (e.g. for short-term prediction): mixing the two on the same frame lets
     *  this sink's anchor-once tie collide with the dense path's already-
     *  present tie on the same (possibly relocalization-seeded) keyframe,
     *  which has been observed to ill-condition the graph in practice. */
    std::string source_frame_id;

    /** Pose of the keyframe (vehicle/base_link) with respect to
     *  `{source_frame_id}`, i.e. the front end's own odometry estimate at
     *  `timestamp`. The receiving backend is expected to chain consecutive
     *  requests from the same source via their *relative* motion (tight,
     *  reflecting the front end's local accuracy), using the absolute value
     *  only to anchor the first keyframe of each source: this lets the
     *  backend's own global corrections (gravity, GNSS, loop closure)
     *  override the front end's accumulated absolute drift. The covariance
     *  is informational only: a conforming backend uses its own configured
     *  noise for these factors, not this covariance (real front ends can
     *  report pathologically tiny values, e.g. a relocalization seed). */
    mrpt::poses::CPose3DPDFGaussian pose_in_source;

    /** Raw observations / annotations for this keyframe (point clouds,
     *  images, IMU windows, etc.), merged into the central map's keyframe
     *  storage. */
    mrpt::obs::CSensoryFrame observations;

    /** Front-end-reported quality in [0,1] (0=worst, 1=best). May be used to
     *  scale the noise of the relative-pose factor chaining this keyframe to
     *  the previous one from the same source. */
    double quality = 1.0;
  };

  /** Requests inserting one keyframe into the central map.
   *  \return The central-map keyframe id, or `std::nullopt` if the request
   *  was rejected.
   */
  virtual std::optional<KeyFrameID> requestInsertKeyframe(const KeyframeInsertRequest& req) = 0;
};

}  // namespace mola
