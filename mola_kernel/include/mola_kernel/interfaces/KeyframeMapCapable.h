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
 * @file   KeyframeMapCapable.h
 * @brief  Virtual interface for metric-map classes that hold keyframes with mutable SE(3) poses.
 * @author Jose Luis Blanco Claraco
 * @date   2026
 */
#pragma once

#include <mrpt/poses/CPose3D.h>

#include <cstdint>
#include <map>
#include <optional>

namespace mola
{
/** Mixin interface for keyframe-based metric maps that need to expose
 *  per-KF pose plumbing to higher layers (e.g. LiDAR odometry online
 *  gravity tilt correction).
 *
 *  A "keyframe map" is a metric map whose internal representation is a
 *  finite, dynamic set of SE(3)-posed sub-clouds (keyframes), each
 *  identified by a stable monotonic `KeyFrameID`.
 *
 *  All methods are required to be thread-safe with respect to the
 *  implementing class' own internal state.
 *
 *  \ingroup mola_kernel_interfaces_grp
 */
class KeyframeMapCapable
{
 public:
  using KeyFrameID = uint64_t;

  KeyframeMapCapable()                                     = default;
  KeyframeMapCapable(const KeyframeMapCapable&)            = default;
  KeyframeMapCapable& operator=(const KeyframeMapCapable&) = default;
  KeyframeMapCapable(KeyframeMapCapable&&)                 = default;
  KeyframeMapCapable& operator=(KeyframeMapCapable&&)      = default;
  virtual ~KeyframeMapCapable();

  /** Returns a snapshot of all currently-active keyframe poses, keyed by
   *  KF id. The returned map is a deep copy and is safe to hold
   *  across subsequent map mutations.
   */
  [[nodiscard]] virtual std::map<KeyFrameID, mrpt::poses::CPose3D> keyframePoses() const = 0;

  /** Returns the smallest currently-active KF id, or nullopt if the map
   *  is empty. Used as the natural pivot for online tilt correction
   *  (oldest active KF stays put).
   */
  [[nodiscard]] virtual std::optional<KeyFrameID> oldestActiveKeyframeID() const = 0;

  /** Overwrites the pose of one keyframe. No-op if `id` is not present.
   *  Any internal caches dependent on KF poses must be invalidated by
   *  the implementation.
   */
  virtual void setKeyframePose(KeyFrameID id, const mrpt::poses::CPose3D& new_pose) = 0;

  /** Applies a pivot-based rigid SE(3) transform to all currently-active KFs.
   *
   *  Implementations are encouraged to delegate to their existing
   *  `transform_map_left_multiply()` after computing:
   *    T_correct = T_pivot + delta_at_pivot + (-T_pivot)
   *  No-op if `pivot_id` is not present in the active set.
   */
  virtual void applyPivotTransform(
      KeyFrameID pivot_id, const mrpt::poses::CPose3D& delta_at_pivot) = 0;
};

}  // namespace mola
