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
 * @file   TransformTreeSource.h
 * @brief  Virtual interface for modules exposing a tree of coordinate frames.
 * @author Jose Luis Blanco Claraco
 * @date   2026
 */
#pragma once

#include <mrpt/core/Clock.h>
#include <mrpt/poses/CPose3D.h>

#include <optional>
#include <string>
#include <vector>

namespace mola
{
/** One coordinate frame within a TransformTree.
 *
 *  Declares no constructor, not even a defaulted one: since C++20 any
 *  user-declared constructor makes a struct a non-aggregate, which breaks the
 *  brace initialization its producers use. The implicit default constructor is
 *  equivalent, and leaving it implicit keeps this compiling under both C++17
 *  and C++20.
 */
struct TransformTreeNode
{
  /** This frame's name, e.g. "LF_FOOT". */
  std::string frame;

  /** Name of this frame's parent. Empty for the tree root only. */
  std::string parent;

  /** Pose of this frame with respect to the tree root. */
  mrpt::poses::CPose3D pose_in_root;
};

/** A snapshot of all coordinate frames hanging below a given root frame,
 *  with their poses already resolved with respect to that root.
 *
 *  Declares no constructor either, for the same reason as TransformTreeNode
 *  above. */
struct TransformTree
{
  /** The frame all poses in `nodes` are relative to. */
  std::string root;

  /** The time the poses were looked up at. */
  mrpt::Clock::time_point timestamp;

  /** The root itself is included, as the first entry, with an identity pose
   *  and an empty `parent`. Parents always appear before their children. */
  std::vector<TransformTreeNode> nodes;
};

/** Mixin interface for modules that track a tree of coordinate frames (ROS
 *  "/tf" and "/tf_static", or an equivalent), and can expose snapshots of it
 *  to other MOLA modules: dataset readers, live sensor bridges, etc.
 *
 *  Consumers detect it the same way they detect a `NavStateFilter`
 *  (`ExecutableBase::findService<TransformTreeSource>()`). Note that this
 *  interface is deliberately free of any ROS/tf2 type, so `mola_kernel` stays
 *  independent of ROS.
 *
 *  Implementations are required to be thread-safe.
 *
 *  \ingroup mola_kernel_interfaces_grp
 */
class TransformTreeSource
{
 public:
  TransformTreeSource()                                      = default;
  TransformTreeSource(const TransformTreeSource&)            = default;
  TransformTreeSource& operator=(const TransformTreeSource&) = default;
  TransformTreeSource(TransformTreeSource&&)                 = default;
  TransformTreeSource& operator=(TransformTreeSource&&)      = default;
  virtual ~TransformTreeSource();

  /** Returns all frames reachable *below* `root` (that is, the subtree it is
   *  the root of), with their poses resolved with respect to it.
   *
   *  Filtering happens here, in the source, since it is what owns the
   *  underlying transform buffer: a consumer interested in a robot's joints
   *  never has to receive, nor walk, the frames of unrelated subtrees.
   *
   *  \param root      The frame to use as the subtree root, e.g. "base_link".
   *  \param timestamp The time to look the transforms up at. If not provided,
   *                   or if no data is buffered for it, the latest available
   *                   transform of each frame is used instead.
   *  \return The subtree, or `std::nullopt` if `root` is not a known frame.
   */
  virtual std::optional<TransformTree> transform_tree(
      const std::string&                            root,
      const std::optional<mrpt::Clock::time_point>& timestamp = std::nullopt) const = 0;

  /** Returns the frame this source considers the robot's body frame (its
   *  `base_link_frame_id` parameter, or equivalent), so consumers can default
   *  to it instead of hardcoding a name. Empty if the source has none. */
  virtual std::string transform_tree_default_root() const { return {}; }
};

}  // namespace mola
