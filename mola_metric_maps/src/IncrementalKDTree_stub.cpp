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
 * @file   IncrementalKDTree_stub.cpp
 * @brief  Fallback for builds whose nanoflann lacks the incremental k-d tree
 * @author Jose Luis Blanco Claraco
 * @date   Jul 25, 2026
 */

// Built instead of IncrementalKDTree.cpp when the available nanoflann predates
// the incremental (self-balancing) index, added in 1.10.0. mola::IncrementalPointCloud
// is still declared, compiled and registered in the class factory, so that YAML
// configurations naming it fail with an explanatory message instead of MRPT's
// generic "no such registered CMetricMap class", which reads like a typo or a
// missing plugin.

#include <stdexcept>
#include <string>

#include "IncrementalKDTree.h"

#if !defined(MOLA_NANOFLANN_FOUND_VERSION)
#define MOLA_NANOFLANN_FOUND_VERSION "not found"
#endif

namespace mola::internal
{
IncrementalKDTree::~IncrementalKDTree() = default;

std::unique_ptr<IncrementalKDTree> IncrementalKDTree::Create([[maybe_unused]] const Params& p)
{
  throw std::runtime_error(
      "mola::IncrementalPointCloud is not available in this build: it requires "
      "nanoflann >= 1.10.0 (the release that introduced the incremental k-d tree "
      "index), but this workspace was built against nanoflann '" MOLA_NANOFLANN_FOUND_VERSION
      "'. Install a newer nanoflann and rebuild mola_metric_maps, or use "
      "mola::KeyframePointCloudMap instead.");
}

}  // namespace mola::internal
