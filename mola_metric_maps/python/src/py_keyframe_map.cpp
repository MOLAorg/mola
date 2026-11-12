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
 * @file   py_keyframe_map.cpp
 * @brief  Python bindings: KeyframePointCloudMap
 * @author Jose Luis Blanco Claraco
 * @date   Nov 12, 2026
 */

#include <mola_metric_maps/KeyframePointCloudMap.h>
#include <mrpt/core/format.h>
#include <mrpt/math/TBoundingBox.h>
#include <pybind11/functional.h>

#include "py_common.h"

namespace mola_metric_maps_py
{
void init_keyframe_map(py::module_& m)
{
  using T = mola::KeyframePointCloudMap;

  py::class_<T, mrpt::maps::CMetricMap, std::shared_ptr<T>> c(
      m, "KeyframePointCloudMap",
      "A map of keyframes: point clouds in their own local frame, each with its global pose, "
      "so a loop closure only updates poses (mola::KeyframePointCloudMap). Each inserted point "
      "cloud observation becomes a new keyframe at the robot pose.");

  using Cr = T::TCreationOptions;
  bind_options<Cr>(
      c, "TCreationOptions", "KeyframePointCloudMap.TCreationOptions",
      "Options of the keyframe search and the per-point covariances.")
      .def_readwrite(
          "max_search_keyframes", &Cr::max_search_keyframes,
          "Maximum number of keyframes to search for neighbors.")
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
          "max_plane_deviation_for_cov", &Cr::max_plane_deviation_for_cov,
          "Maximum distance [m] of a neighbor to the fitted plane for the plane regularization "
          "to apply (0: no test).")
      .def_readwrite(
          "plane_regularization_lambda", &Cr::plane_regularization_lambda,
          "Variance along the normal of the regularized covariances (<=0: no regularization).")
      .def_readwrite(
          "rotation_distance_weight", &Cr::rotation_distance_weight,
          "Weight [m/rad] of the angular distance when ranking keyframes by proximity.")
      .def_readwrite(
          "num_diverse_keyframes", &Cr::num_diverse_keyframes,
          "Keyframe search slots reserved for angularly diverse or farther keyframes.")
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
          "serialize_kdtrees", &Cr::serialize_kdtrees,
          "Store the per-keyframe k-d trees in the serialized map.")
      .def_readwrite(
          "serialize_covariances", &Cr::serialize_covariances,
          "Store the per-point covariances in the serialized map.")
      .def_readwrite(
          "approximate_cov", &Cr::approximate_cov,
          "Cov-to-cov matching against each active keyframe instead of a merged submap.")
      .def_readwrite(
          "density_penalty_min_points", &Cr::density_penalty_min_points,
          "Keyframes with fewer points are penalized in the proximity ranking (0: disabled).")
      .def_readwrite(
          "density_penalty_max_m", &Cr::density_penalty_max_m,
          "Maximum proximity penalty [m], for an empty keyframe.");

  bind_options<T::TInsertionOptions>(
      c, "TInsertionOptions", "KeyframePointCloudMap.TInsertionOptions",
      "Options for inserting observations.")
      .def_readwrite(
          "remove_frames_farther_than", &T::TInsertionOptions::remove_frames_farther_than,
          "If > 0, keyframes farther than this distance [m] from the robot are removed on each "
          "insertion.");

  bind_options<T::TLikelihoodOptions>(
      c, "TLikelihoodOptions", "KeyframePointCloudMap.TLikelihoodOptions",
      "Options for computing the likelihood of observations (none yet).");

  using Ren = T::TRenderOptions;
  bind_options<Ren>(
      c, "TRenderOptions", "KeyframePointCloudMap.TRenderOptions", "Options for rendering.")
      .def_readwrite("point_size", &Ren::point_size, "Point size [pixels].")
      .def_readwrite(
          "color", &Ren::color, "Color of points (mrpt.img.TColorf), if colormap is cmNONE.")
      .def_readwrite("colormap", &Ren::colormap, "Colormap (mrpt.img.TColormap).")
      .def_readwrite(
          "recolorByPointField", &Ren::recolorByPointField, "Point field used for the colormap.")
      .def_readwrite(
          "max_points_per_kf", &Ren::max_points_per_kf, "Maximum points to render per keyframe.")
      .def_readwrite(
          "max_overall_points", &Ren::max_overall_points, "Maximum points to render overall.")
      .def_readwrite(
          "keyframes_axes_length", &Ren::keyframes_axes_length,
          "If > 0, draw the axes of each keyframe, of this length [m].")
      .def_readwrite(
          "show_covariances", &Ren::show_covariances,
          "Draw the per-point covariances as ellipsoids.")
      .def_readwrite(
          "cov_color", &Ren::cov_color, "Color of the covariance ellipsoids (mrpt.img.TColorf).")
      .def_readwrite(
          "show_cov_decimation", &Ren::show_cov_decimation,
          "Draw the covariance of one out of N points.");

  using Rp = T::RegroupParams;
  py::class_<Rp>(c, "RegroupParams", "Parameters of regroupKeyframes(), see the C++ documentation.")
      .def(py::init<>(), "Default parameters.")
      .def_readwrite(
          "voxel_size", &Rp::voxel_size,
          "Voxel size [m] to compute cloud overlaps (<=0: automatic).")
      .def_readwrite(
          "edge_overlap", &Rp::edge_overlap,
          "Minimum overlap [0,1] of two keyframes to be merged into the same group.")
      .def_readwrite(
          "core_fraction", &Rp::core_fraction,
          "Inner-core fraction (0,1] of the extent of a group; the rest overlaps with others.")
      .def_readwrite(
          "extent_factor", &Rp::extent_factor,
          "Maximum extent of a group, as a multiple of the sensing radius of its seed.")
      .def_readwrite(
          "merge_decimate_voxel", &Rp::merge_decimate_voxel,
          "If > 0, voxel size [m] to decimate the merged clouds.")
      .def_readwrite(
          "island_merge_fraction", &Rp::island_merge_fraction,
          "Groups smaller than this fraction of the largest one are merged into a neighbor.")
      .def_readwrite("unify_all", &Rp::unify_all, "Merge all keyframes into a single one.")
      .def(
          "__repr__",
          [](const Rp& p)
          {
            return mrpt::format(
                "KeyframePointCloudMap.RegroupParams(voxel_size=%g, edge_overlap=%g, "
                "core_fraction=%g, extent_factor=%g, merge_decimate_voxel=%g, "
                "island_merge_fraction=%g, unify_all=%s)",
                p.voxel_size, p.edge_overlap, p.core_fraction, p.extent_factor,
                p.merge_decimate_voxel, p.island_merge_fraction, p.unify_all ? "True" : "False");
          });

  c.def(py::init<>(), "Creates an empty map.")
      .def(
          "keyframePoses", &T::keyframePoses,
          "A dict {keyframe ID: mrpt.poses.CPose3D} with the global poses of all keyframes.")
      .def(
          "getKeyframePosesAsNumpy",
          [](const T& self)
          {
            const auto            poses = self.keyframePoses();
            const auto            n     = static_cast<py::ssize_t>(poses.size());
            py::array_t<uint64_t> ids(std::vector<py::ssize_t>{n});
            py::array_t<double>   mats(std::vector<py::ssize_t>{n, 4, 4});
            auto                  id = ids.mutable_unchecked<1>();
            auto                  M  = mats.mutable_unchecked<3>();
            py::ssize_t           i  = 0;
            for (const auto& [kfId, pose] : poses)
            {
              id(i)        = kfId;
              const auto H = pose.getHomogeneousMatrix();
              for (py::ssize_t r = 0; r < 4; r++)
              {
                for (py::ssize_t col = 0; col < 4; col++)
                {
                  M(i, r, col) = H(r, col);
                }
              }
              i++;
            }
            return py::make_tuple(ids, mats);
          },
          "The keyframe IDs and poses, as a tuple (ids: uint64 array, poses: (N,4,4) float64 "
          "array of homogeneous matrices), sorted by ID.")
      .def(
          "setKeyframePose",
          [](T& self, uint64_t id, const py::object& pose)
          { self.setKeyframePose(id, to_pose3d(pose)); },
          "id"_a, "pose"_a,
          "Changes the global pose of a keyframe (mrpt.poses.CPose3D, a 4x4 matrix...). Does "
          "nothing if it does not exist.")
      .def(
          "lastInsertedKeyFrameID", &T::lastInsertedKeyFrameID,
          "ID of the last inserted keyframe, or None if none was inserted since the map was "
          "created or cleared.")
      .def(
          "nextFreeKeyFrameID", &T::nextFreeKeyFrameID_public,
          "The ID that the next inserted keyframe will get.")
      .def(
          "drainEvictedKeyFrameIDs", &T::drainEvictedKeyFrameIDs,
          "Returns and clears the list of keyframe IDs removed since the last call.")
      .def("point_count", &T::point_count, "Total number of points in all keyframes.")
      .def(
          "getPointsAsNumpy",
          [](const T& self)
          {
            const auto* pts = self.getAsSimplePointsMap();
            if (!pts)
            {
              return py::array_t<float>(std::vector<py::ssize_t>{0, 3});
            }
            const auto&        xs = pts->getPointsBufferRef_x();
            const auto&        ys = pts->getPointsBufferRef_y();
            const auto&        zs = pts->getPointsBufferRef_z();
            const auto         n  = static_cast<py::ssize_t>(pts->size());
            py::array_t<float> arr(std::vector<py::ssize_t>{n, 3});
            auto               a = arr.mutable_unchecked<2>();
            for (py::ssize_t i = 0; i < n; i++)
            {
              a(i, 0) = xs[i];
              a(i, 1) = ys[i];
              a(i, 2) = zs[i];
            }
            return arr;
          },
          "The points of all keyframes, in the global frame, as an (N,3) float32 array. As "
          "the point cloud view of the map (e.g. mp2p_icp.layer_to_numpy()), they are decimated "
          "as set by renderOptions.max_points_per_kf and max_overall_points (0: no limit).")
      .def(
          "icp_get_prepared_as_global",
          [](const T& self, const py::object& pose,
             const std::optional<mrpt::math::TBoundingBoxf>& roi)
          {
            const auto             p = to_pose3d(pose);
            py::gil_scoped_release nogil;
            self.icp_get_prepared_as_global(p, roi);
          },
          "pose"_a, "local_map_roi"_a = py::none(),
          "Selects the keyframes near a pose (and an optional mrpt.math.TBoundingBoxf around "
          "it) as the active submap for ICP. Needed before the nn_*() queries.")
      .def("icp_cleanup", &T::icp_cleanup, "Releases the active submap of ICP.")
      .def(
          "transform_map_left_multiply",
          [](T& self, const py::object& pose)
          { self.transform_map_left_multiply(to_pose3d(pose)); },
          "pose"_a, "Left-multiplies all keyframe poses by a pose.")
      .def(
          "merge_with",
          [](T& self, const T& other, const py::object& relativePose)
          { self.merge_with(other, to_optional_pose3d(relativePose)); },
          "other"_a, "relative_pose"_a = py::none(),
          "Adds the keyframes of another map, optionally transformed by a relative pose.")
      .def(
          "regroupKeyframes",
          [](const T& self, const Rp& params, const std::function<void(const std::string&)>& log)
          {
            // The callback wrapper takes the GIL by itself, from any thread:
            py::gil_scoped_release nogil;
            return self.regroupKeyframes(params, log);
          },
          "params"_a = Rp(), "log"_a = py::none(),
          "Builds a NEW map whose keyframes are overlapping groups of the keyframes of this one, "
          "so localization needs one active keyframe at a time (as the mm-kf-regroup tool). "
          "`log`, if given, is called with progress messages.")
      .def_readwrite(
          "insertionOptions", &T::insertionOptions, "Insertion options (TInsertionOptions).")
      .def_readwrite(
          "likelihoodOptions", &T::likelihoodOptions, "Likelihood options (TLikelihoodOptions).")
      .def_readwrite("renderOptions", &T::renderOptions, "Render options (TRenderOptions).")
      .def(
          "__repr__",
          [](const T& self) {
            return mrpt::format(
                "KeyframePointCloudMap(keyframes=%zu)", self.keyframePoses().size());
          });

  def_options_capable<T>(c);
  def_insert_point_cloud<T>(c);
  def_nn_search<T>(c);
  def_copy_and_pickle<T>(c);
}

}  // namespace mola_metric_maps_py
