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
 * @file   MatchingDistanceProfileCompat.h
 * @brief  mp2p_icp::MatchingDistanceProfile, or a flat stand-in for older mp2p_icp
 * @author Jose Luis Blanco Claraco
 * @date   Aug 11, 2026
 */
#pragma once

// This package is developed in lockstep with mp2p_icp, but rosdep (and hence
// every CI job) resolves mp2p_icp to the last *released* binary package, which
// can predate this source tree.
//
// MatchingDistanceProfile.h is an entirely new header, so its mere presence is
// an exact test for the API and __has_include() alone settles it. (Contrast
// with MOLA_MM_HAS_ROTATE_VIEW_HEADER in KeyframePointCloudMap.cpp, which needs
// a second probe only because that feature was added to a header that already
// existed.) Everything downstream then keys off mp2p_icp's own
// MP2P_ICP_HAS_MATCHING_DISTANCE_PROFILE, so there is a single macro to reason
// about rather than a mola-side mirror of it.
#if defined(__has_include)
#if __has_include(<mp2p_icp/MatchingDistanceProfile.h>)
#include <mp2p_icp/MatchingDistanceProfile.h>
#endif
#endif

namespace mola
{
#if defined(MP2P_ICP_HAS_MATCHING_DISTANCE_PROFILE)

/** The real thing, when mp2p_icp is recent enough to provide it. */
using MatchingDistanceProfile = mp2p_icp::MatchingDistanceProfile;

#else

/** Flat-only stand-in used when building against an mp2p_icp release that
 *  predates mp2p_icp::MatchingDistanceProfile.
 *
 *  It exposes the same (small) surface the map implementations use, and is
 *  constrained to the flat case by construction: there is no way to build a
 *  range-adaptive profile, because the matcher on the other side of the
 *  interface cannot ask for one either. This is what keeps the search
 *  implementations themselves free of preprocessor branches.
 */
struct MatchingDistanceProfile
{
  MatchingDistanceProfile() = default;

  /** Implicit on purpose, matching the real type. */
  MatchingDistanceProfile(float flatThreshold) : near(flatThreshold) {}

  float near = 0.40f;
  float far  = 0.40f;

  [[nodiscard]] bool isFlat() const { return true; }
  [[nodiscard]] bool needsRange() const { return false; }

  [[nodiscard]] float operator()(float /*range*/) const { return near; }
};

#endif

}  // namespace mola
