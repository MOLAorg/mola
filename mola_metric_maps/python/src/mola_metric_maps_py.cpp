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
 * @file   mola_metric_maps_py.cpp
 * @brief  Python extension module `mola_metric_maps._core`
 * @author Jose Luis Blanco Claraco
 * @date   Nov 12, 2026
 */

#include <mola_metric_maps/PointIndexType.h>
#include <mola_metric_maps/ViewDirectionFilter.h>

#include "py_common.h"

namespace mola_metric_maps_py
{
void init_voxel_maps(py::module_& m);
void init_keyframe_map(py::module_& m);
void init_incremental_point_cloud(py::module_& m);
void init_occgrid(py::module_& m);
}  // namespace mola_metric_maps_py

namespace
{
// Optional features, which depend on the nanoflann version found at build time:
constexpr bool has_incremental_kdtree()
{
#if defined(MOLA_METRIC_MAPS_HAS_INCREMENTAL_POINT_CLOUD)
  return true;
#else
  return false;
#endif
}

constexpr bool has_incremental_kdtree_bake()
{
#if defined(MOLA_METRIC_MAPS_HAS_INCREMENTAL_KDTREE_BAKE)
  return true;
#else
  return false;
#endif
}
}  // namespace

PYBIND11_MODULE(_core, m)
{
  namespace py = pybind11;
  using namespace mola_metric_maps_py;

  m.doc() =
      "Python bindings of the mola_metric_maps C++ library (use the `mola_metric_maps` package)";
  m.attr("__version__") = MOLA_METRIC_MAPS_VERSION_STRING;

  m.attr("HAS_INCREMENTAL_KDTREE")      = has_incremental_kdtree();
  m.attr("HAS_INCREMENTAL_KDTREE_BAKE") = has_incremental_kdtree_bake();

  // Types of other modules used in signatures, and the bases of the map
  // classes, must be registered first, by their own Python modules: this one
  // shares their type registry.
  for (const char* mod :
       {"mrpt.rtti", "mrpt.serialization", "mrpt.config", "mrpt.math", "mrpt.img", "mrpt.poses",
        "mrpt.viz", "mrpt.obs", "mrpt.maps", "mp2p_icp"})
  {
    py::module_::import(mod);
  }

  py::enum_<mola::ViewDirectionFilter>(
      m, "ViewDirectionFilter",
      "How the cov-to-cov matching of KeyframePointCloudMap and IncrementalPointCloud uses the "
      "per-point view directions (view_x/y/z fields) to reject pairs.")
      .value("None_", mola::ViewDirectionFilter::None, "No filtering.")
      .value(
          "MaxAngle", mola::ViewDirectionFilter::MaxAngle,
          "Reject pairs whose view directions are more than max_view_angle_deg apart.")
      .value(
          "SurfaceSide", mola::ViewDirectionFilter::SurfaceSide,
          "Reject pairs seeing a flat surface from opposite sides.");

  py::enum_<mola::PointIndexType>(m, "PointIndexType", "Spatial index of IncrementalPointCloud.")
      .value(
          "KDTree", mola::PointIndexType::KDTree,
          "Incremental, self-balancing k-d tree (nanoflann).")
      .value(
          "VoxelHash", mola::PointIndexType::VoxelHash,
          "Hashed grid of voxels, with O(1) insertion and removal.");

  init_voxel_maps(m);
  init_keyframe_map(m);
  init_incremental_point_cloud(m);
  init_occgrid(m);
}
