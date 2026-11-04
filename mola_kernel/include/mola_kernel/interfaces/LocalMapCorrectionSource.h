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
 * @file   LocalMapCorrectionSource.h
 * @brief  Virtual interface for modules that correct the keyframes of a front end.
 * @author Jose Luis Blanco Claraco
 * @date   2026
 */
#pragma once

#include <mola_kernel/interfaces/SharedKeyframeMap.h>
#include <mrpt/core/Clock.h>
#include <mrpt/obs/CSensoryFrame.h>
#include <mrpt/poses/CPose3D.h>

#include <cstddef>
#include <cstdint>
#include <functional>
#include <map>
#include <mutex>
#include <optional>
#include <string>
#include <vector>

namespace mola
{
/** Notice that the poses of some keyframes changed (e.g. after a loop closure),
 *  so a front end whose local map was built from them may rebuild it. It only
 *  says that something changed: the front end then asks for the keyframes with
 *  LocalMapCorrectionSource::local_map_keyframes(). */
struct LocalMapCorrection
{
  LocalMapCorrection() = default;

  /** Time of the newest keyframe included in the correction. */
  mrpt::Clock::time_point timestamp;

  /** Loop closures added since the previous correction. */
  std::size_t new_loop_closures = 0;
};

/** One keyframe given back to a front end, with its current pose estimate. */
struct LocalMapKeyframe
{
  LocalMapKeyframe() = default;

  SharedKeyframeMap::KeyFrameID id = 0;
  mrpt::Clock::time_point       timestamp;

  /** The `source_frame_id` of the front end that inserted it. */
  std::string source_frame_id;

  /** Pose of the keyframe (vehicle frame) relative to the reference keyframe
   *  of the query. */
  mrpt::poses::CPose3D pose_from_reference;

  /** Its observations, as inserted (SharedKeyframeMap::KeyframeInsertRequest). */
  mrpt::obs::CSensoryFrame observations;
};

/** Virtual interface for modules that estimate the poses of the keyframes
 *  inserted by front ends (e.g. a global mapper with loop closures), so a front
 *  end can rebuild its local map with the corrected poses, and reuse what was
 *  mapped in earlier visits of the same place.
 *
 *  Consumers find providers with `ExecutableBase::findService<LocalMapCorrectionSource>()`,
 *  subscribe to corrections, and, when one arrives, query the keyframes around
 *  one of their own keyframes (the reference) with local_map_keyframes(), from
 *  any thread. Poses are relative to the reference, so the front end can anchor
 *  them at its own estimate of the reference, and its own pose does not jump.
 *  Callbacks run in the provider thread, one at a time, and must return quickly
 *  (e.g. only wake up a worker thread).
 *
 *  Providers implement local_map_keyframes_impl(), thread-safe, and call
 *  advertise_local_map_correction() after each correction, without holding the
 *  lock that local_map_keyframes_impl() takes.
 *
 *  Downstream code can detect this interface with
 *  `__has_include(<mola_kernel/interfaces/LocalMapCorrectionSource.h>)`.
 *
 * \ingroup mola_kernel_interfaces_grp
 */
class LocalMapCorrectionSource
{
 public:
  LocalMapCorrectionSource() = default;
  virtual ~LocalMapCorrectionSource();

  LocalMapCorrectionSource(const LocalMapCorrectionSource&)            = delete;
  LocalMapCorrectionSource& operator=(const LocalMapCorrectionSource&) = delete;
  LocalMapCorrectionSource(LocalMapCorrectionSource&&)                 = delete;
  LocalMapCorrectionSource& operator=(LocalMapCorrectionSource&&)      = delete;

  /** The keyframes whose current position is within `radius` [m] of keyframe
   *  `reference` (included), from any front end, oldest first. Nothing if
   *  `reference` is unknown to this provider. */
  [[nodiscard]] std::optional<std::vector<LocalMapKeyframe>> local_map_keyframes(
      SharedKeyframeMap::KeyFrameID reference, double radius) const
  {
    return local_map_keyframes_impl(reference, radius);
  }

  using correction_callback_t = std::function<void(const LocalMapCorrection&)>;
  using subscription_id_t     = std::uint64_t;

  /** Registers a callback invoked after every correction.
   *  \return an id for unsubscribe_from_local_map_corrections().
   *  \exception std::invalid_argument if the callback is empty. */
  subscription_id_t subscribe_to_local_map_corrections(const correction_callback_t& callback);

  /** Removes a callback. Once this returns, the callback is neither running nor
   *  started again: it waits for a correction being advertised in another
   *  thread, so do not call it while holding a lock that the callback takes.
   *  It can be called from within the callback itself. Unknown ids are ignored. */
  void unsubscribe_from_local_map_corrections(subscription_id_t id);

 protected:
  [[nodiscard]] bool any_local_map_correction_subscriber() const;

  /** To be called by providers after each correction. An exception in one
   *  callback does not prevent the others from running. */
  void advertise_local_map_correction(const LocalMapCorrection& c);

 private:
  /** Implemented by providers: see local_map_keyframes(). */
  [[nodiscard]] virtual std::optional<std::vector<LocalMapKeyframe>> local_map_keyframes_impl(
      SharedKeyframeMap::KeyFrameID reference, double radius) const = 0;

  std::map<subscription_id_t, correction_callback_t> subs_;
  subscription_id_t                                  nextId_ = 1;
  mutable std::mutex                                 subsMtx_;
  /// Held while advertising, so unsubscribing waits for callbacks in progress.
  std::recursive_mutex dispatchMtx_;
};

}  // namespace mola
