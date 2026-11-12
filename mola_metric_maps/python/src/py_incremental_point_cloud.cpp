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
 * @file   py_incremental_point_cloud.cpp
 * @brief  Python bindings: IncrementalPointCloud
 * @author Jose Luis Blanco Claraco
 * @date   Nov 12, 2026
 */

#include <mola_metric_maps/IncrementalPointCloud.h>
#include <mrpt/core/format.h>
#include <mrpt/poses/CPose2D.h>

#include "py_common.h"

namespace mola_metric_maps_py
{
void init_incremental_point_cloud(py::module_& m)
{
  using T = mola::IncrementalPointCloud;

  py::class_<T, mrpt::maps::CGenericPointsMap, std::shared_ptr<T>> c(
      m, "IncrementalPointCloud",
      "A sliding-window point cloud for LiDAR odometry, with an incremental spatial index "
      "(mola::IncrementalPointCloud). It is an mrpt.maps.CGenericPointsMap, but point indices "
      "are storage slots: size() and the inherited NumPy conversions include freed slots (NaN) "
      "and removed points not reclaimed yet. Use livePointCount() and liveCompactedCopy() (or "
      "compact()) to work with the live points only.");

  using Cr = T::TCreationOptions;
  bind_options<Cr>(
      c, "TCreationOptions", "IncrementalPointCloud.TCreationOptions",
      "Options of the spatial index, the sliding window and the per-point covariances.")
      .def_readwrite(
          "remove_points_farther_than", &Cr::remove_points_farther_than,
          "If not 0, on each insertion keep only the points in a cube of this half side [m] "
          "around the robot.")
      .def_readwrite(
          "min_distance_between_points", &Cr::min_distance_between_points,
          "If not 0, inserted points closer than this [m] to one in the map are dropped.")
      .def_readwrite("index_type", &Cr::index_type, "Spatial index type (PointIndexType).")
      .def_readwrite(
          "index_voxel_size", &Cr::index_voxel_size, "Voxel size [m] of the VoxelHash index.")
      .def_readwrite(
          "async_rebuild", &Cr::async_rebuild,
          "KDTree index only: rebuild the tree in a background thread.")
      .def_readwrite(
          "alpha_balance", &Cr::alpha_balance, "KDTree index only: balance rebuild threshold.")
      .def_readwrite(
          "alpha_deleted", &Cr::alpha_deleted, "KDTree index only: deleted points threshold.")
      .def_readwrite(
          "reserve_points", &Cr::reserve_points,
          "If not 0, preallocate storage for this many points.")
      .def_readwrite(
          "k_correspondences_for_cov", &Cr::k_correspondences_for_cov,
          "Number of neighbors to estimate each point covariance.")
      .def_readwrite(
          "min_correspondences_for_cov", &Cr::min_correspondences_for_cov,
          "Below this number of neighbors, an isotropic covariance is used.")
      .def_readwrite(
          "max_distance_for_cov", &Cr::max_distance_for_cov,
          "Maximum distance [m] of the neighbors used for a covariance.")
      .def_readwrite(
          "min_neighbors_to_cache_cov", &Cr::min_neighbors_to_cache_cov,
          "Minimum neighbors for a covariance to be cached (0: k_correspondences_for_cov).")
      .def_readwrite(
          "max_plane_deviation_for_cov", &Cr::max_plane_deviation_for_cov,
          "Maximum distance [m] of a neighbor to the fitted plane for the plane regularization "
          "to apply (0: no test).")
      .def_readwrite(
          "plane_regularization_lambda", &Cr::plane_regularization_lambda,
          "Variance along the normal of the regularized covariances (<=0: no regularization).")
      .def_readwrite(
          "use_view_direction_filter", &Cr::use_view_direction_filter,
          "Master switch of the view-direction filter of cov-to-cov pairs.")
      .def_readwrite(
          "view_direction_filter", &Cr::view_direction_filter,
          "How the view-direction filter decides (ViewDirectionFilter).")
      .def_readwrite(
          "max_view_angle_deg", &Cr::max_view_angle_deg,
          "Maximum angle [deg] between view directions, for ViewDirectionFilter.MaxAngle.")
      .def_readwrite(
          "serialize_kdtree", &Cr::serialize_kdtree,
          "KDTree index only: store the tree in the serialized map.");

  c.def(py::init<>(), "Creates an empty map.")
      .def(
          "livePointCount", &T::livePointCount,
          "Number of points in the map, without the freed storage slots counted by size().")
      .def(
          "recyclableSlotCount", &T::recyclableSlotCount,
          "Number of storage slots ready to be reused by the next insertion.")
      .def(
          "covarianceComputations", &T::covarianceComputations,
          "Number of per-point covariance estimations so far (diagnostics).")
      .def(
          "cachedCovarianceCount", &T::cachedCovarianceCount,
          "Number of per-point covariances in the cache.")
      .def(
          "compact",
          [](T& self)
          {
            py::gil_scoped_release nogil;
            self.compact();
          },
          "Drops the freed storage slots, so size() equals livePointCount(), and rebuilds the "
          "index. Invalidates the indices returned by the nn_*() methods.")
      .def(
          "keepOnlyPointsNear",
          [](T& self, const py::object& center, double halfSide)
          { self.keepOnlyPointsNear(to_point3df(center), halfSide); },
          "center"_a, "half_side"_a,
          "Removes the points outside a cube of half side `half_side` [m] around `center`.")
      .def(
          "liveCompactedCopy", &T::liveCompactedCopy,
          "A new mrpt.maps.CGenericPointsMap with the live points only, with all their fields.")
      .def(
          "changeCoordinatesReference",
          [](T& self, const mrpt::poses::CPose3D& b)
          {
            py::gil_scoped_release nogil;
            self.changeCoordinatesReference(b);
          },
          "pose"_a, "Transforms all points by a pose (mrpt.poses.CPose3D) and rebuilds the index.")
      .def(
          "changeCoordinatesReference",
          [](T& self, const mrpt::poses::CPose2D& b)
          {
            py::gil_scoped_release nogil;
            self.changeCoordinatesReference(b);
          },
          "pose"_a, "Transforms all points by a pose (mrpt.poses.CPose2D) and rebuilds the index.")
      .def(
          "__repr__",
          [](const T& self)
          {
            return mrpt::format(
                "IncrementalPointCloud(points=%zu, storage=%zu)", self.livePointCount(),
                self.size());
          });

  def_options_capable<T>(c);
  def_insert_point_cloud<T>(c);
  // Replaces the nn_*() of CPointsMap, to accept [x, y, z] queries as the
  // other map classes. 2D queries are not supported by this class.
  def_nn_search<T>(c);
  def_copy_and_pickle<T>(c);
}

}  // namespace mola_metric_maps_py
