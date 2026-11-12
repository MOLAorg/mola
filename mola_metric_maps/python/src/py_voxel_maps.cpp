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
 * @file   py_voxel_maps.cpp
 * @brief  Python bindings: voxel maps (hashed, sparse, sparse trees, NDT, TSDF)
 * @author Jose Luis Blanco Claraco
 * @date   Nov 12, 2026
 */

#include <mola_metric_maps/HashedVoxelPointCloud.h>
#include <mola_metric_maps/NDT.h>
#include <mola_metric_maps/SparseTreesPointCloud.h>
#include <mola_metric_maps/SparseVoxelPointCloud.h>
#include <mola_metric_maps/TSDF.h>

#include <limits>

#include "py_common.h"

namespace mola_metric_maps_py
{
namespace
{
using mrpt::config::CLoadableOptions;
using mrpt::maps::CMetricMap;
using mrpt::math::TPoint3Df;

/** NumPy copy of a flat vector, with the given shape */
template <typename T>
py::array_t<T> to_array(const std::vector<T>& v, const std::vector<py::ssize_t>& shape)
{
  py::array_t<T> arr(shape);
  std::copy(v.begin(), v.end(), arr.mutable_data());
  return arr;
}

/** Collects the per-voxel data of a map, as columns of NumPy arrays */
struct VoxelColumns
{
  std::vector<int32_t>  indices;
  std::vector<float>    centers;
  std::vector<uint32_t> num_points;

  void add(const mola::index3d_t<int32_t>& idx, const TPoint3Df& center, size_t n)
  {
    indices.insert(indices.end(), {idx.cx, idx.cy, idx.cz});
    centers.insert(centers.end(), {center.x, center.y, center.z});
    num_points.push_back(static_cast<uint32_t>(n));
  }

  [[nodiscard]] py::ssize_t size() const { return static_cast<py::ssize_t>(num_points.size()); }

  [[nodiscard]] py::dict as_dict() const
  {
    py::dict d;
    d["indices"]    = to_array(indices, {size(), 3});
    d["centers"]    = to_array(centers, {size(), 3});
    d["num_points"] = to_array(num_points, {size()});
    return d;
  }
};

/** An (N,3) array with all the points visited by visitAllPoints() */
template <class T>
py::array_t<float> all_points(const T& map)
{
  std::vector<float> xyz;
  map.visitAllPoints([&](const TPoint3Df& p) { xyz.insert(xyz.end(), {p.x, p.y, p.z}); });
  return to_array(xyz, {static_cast<py::ssize_t>(xyz.size() / 3), 3});
}

// -----------------------------------------------------------------------------
// HashedVoxelPointCloud
// -----------------------------------------------------------------------------
void init_hashed_voxel_point_cloud(py::module_& m)
{
  using T = mola::HashedVoxelPointCloud;

  py::class_<T, CMetricMap, std::shared_ptr<T>> c(
      m, "HashedVoxelPointCloud",
      "A point cloud stored as a sparse hash map of cubic voxels, each with up to 32 points "
      "(mola::HashedVoxelPointCloud).");

  bind_options<T::TCreationOptions>(
      c, "TCreationOptions", "HashedVoxelPointCloud.TCreationOptions",
      "Options that define the map structure.")
      .def_readwrite("voxel_size", &T::TCreationOptions::voxel_size, "Voxel size [m].");

  bind_options<T::TInsertionOptions>(
      c, "TInsertionOptions", "HashedVoxelPointCloud.TInsertionOptions",
      "Options for inserting observations.")
      .def_readwrite(
          "max_points_per_voxel", &T::TInsertionOptions::max_points_per_voxel,
          "Maximum number of points per voxel (0: no limit other than 32).")
      .def_readwrite(
          "remove_voxels_farther_than", &T::TInsertionOptions::remove_voxels_farther_than,
          "If not 0, voxels farther than this distance [m] from the robot are removed on each "
          "insertion.")
      .def_readwrite(
          "min_distance_between_points", &T::TInsertionOptions::min_distance_between_points,
          "If not 0, points closer than this [m] to another one in its voxel are not inserted.");

  bind_options<T::TLikelihoodOptions>(
      c, "TLikelihoodOptions", "HashedVoxelPointCloud.TLikelihoodOptions",
      "Options for computing the likelihood of observations.")
      .def_readwrite(
          "sigma_dist", &T::TLikelihoodOptions::sigma_dist,
          "Divisor of the squared point-to-map distance in the log-likelihood [m^2].")
      .def_readwrite(
          "max_corr_distance", &T::TLikelihoodOptions::max_corr_distance,
          "Maximum distance [m] of a point to the map, to bound its likelihood.")
      .def_readwrite("decimation", &T::TLikelihoodOptions::decimation, "Use one out of N points.");

  bind_options<T::TRenderOptions>(
      c, "TRenderOptions", "HashedVoxelPointCloud.TRenderOptions", "Options for rendering.")
      .def_readwrite("point_size", &T::TRenderOptions::point_size, "Point size [pixels].")
      .def_readwrite(
          "color", &T::TRenderOptions::color,
          "Color of points (mrpt.img.TColorf), if colormap is cmNONE.")
      .def_readwrite("colormap", &T::TRenderOptions::colormap, "Colormap (mrpt.img.TColormap).")
      .def_readwrite(
          "recolorByPointField", &T::TRenderOptions::recolorByPointField,
          "Point field used for the colormap.");

  c.def(
       py::init<float>(), "voxel_size"_a = 1.0f,
       "Creates an empty map, with voxels of this size [m].")
      .def(
          "setVoxelProperties", &T::setVoxelProperties, "voxel_size"_a,
          "Changes the voxel size [m], CLEARING the map.")
      .def(
          "insertPoint", [](T& self, const py::object& pt) { self.insertPoint(to_point3df(pt)); },
          "point"_a, "Inserts one point, in map coordinates (mrpt.math.TPoint3D or [x, y, z]).")
      .def(
          "voxelCount", [](const T& self) { return self.voxels().size(); },
          "Number of voxels in the map.")
      .def(
          "getPointsAsNumpy", [](const T& self) { return all_points(self); },
          "All points in the map, as an (N,3) float32 array.")
      .def(
          "getVoxelsAsNumpy",
          [](const T& self)
          {
            const float  half = 0.5f * self.creationOptions.voxel_size;
            VoxelColumns cols;
            self.visitAllVoxels(
                [&](const T::global_index3d_t& idx, const T::VoxelData& v)
                {
                  const auto corner = self.globalIdxToCoord(idx);
                  cols.add(idx, {corner.x + half, corner.y + half, corner.z + half}, v.size());
                });
            return cols.as_dict();
          },
          "The voxels, as a dict of arrays: 'indices' ((M,3) int32), 'centers' ((M,3) "
          "float32) and 'num_points' (uint32).")
      .def(
          "saveToTextFile", &T::saveToTextFile, "file"_a,
          "Saves all points to a text file, one 'x y z' per line. Returns False on error.")
      .def_readwrite(
          "insertionOptions", &T::insertionOptions, "Insertion options (TInsertionOptions).")
      .def_readwrite(
          "likelihoodOptions", &T::likelihoodOptions, "Likelihood options (TLikelihoodOptions).")
      .def_readwrite("renderOptions", &T::renderOptions, "Render options (TRenderOptions).")
      .def(
          "__repr__",
          [](const T& self)
          {
            return mrpt::format(
                "HashedVoxelPointCloud(voxel_size=%g, voxels=%zu)",
                static_cast<double>(self.creationOptions.voxel_size), self.voxels().size());
          });

  def_options_capable<T>(c);
  def_insert_point_cloud<T>(c);
  def_nn_search<T>(c);
  def_copy_and_pickle<T>(c);
}

// -----------------------------------------------------------------------------
// SparseVoxelPointCloud
// -----------------------------------------------------------------------------
void init_sparse_voxel_point_cloud(py::module_& m)
{
  using T = mola::SparseVoxelPointCloud;

  py::class_<T, CMetricMap, std::shared_ptr<T>> c(
      m, "SparseVoxelPointCloud",
      "A point cloud in a two-level voxel map: a sparse map of dense blocks of 32x32x32 voxels, "
      "each with up to 16 points and their mean (mola::SparseVoxelPointCloud).");

  bind_options<T::TCreationOptions>(
      c, "TCreationOptions", "SparseVoxelPointCloud.TCreationOptions",
      "Options that define the map structure.")
      .def_readwrite("voxel_size", &T::TCreationOptions::voxel_size, "Voxel size [m].");

  bind_options<T::TInsertionOptions>(
      c, "TInsertionOptions", "SparseVoxelPointCloud.TInsertionOptions",
      "Options for inserting observations.")
      .def_readwrite(
          "max_points_per_voxel", &T::TInsertionOptions::max_points_per_voxel,
          "Maximum number of points per voxel (0: no limit other than 16).");

  bind_options<T::TLikelihoodOptions>(
      c, "TLikelihoodOptions", "SparseVoxelPointCloud.TLikelihoodOptions",
      "Options for computing the likelihood of observations.")
      .def_readwrite(
          "sigma_dist", &T::TLikelihoodOptions::sigma_dist,
          "Divisor of the squared point-to-map distance in the log-likelihood [m^2].")
      .def_readwrite(
          "max_corr_distance", &T::TLikelihoodOptions::max_corr_distance,
          "Maximum distance [m] of a point to the map, to bound its likelihood.")
      .def_readwrite("decimation", &T::TLikelihoodOptions::decimation, "Use one out of N points.")
      .def_readwrite(
          "match_mean", &T::TLikelihoodOptions::match_mean,
          "Match against the voxel means instead of their points, in the likelihood and the "
          "nn_*() queries.");

  bind_options<T::TRenderOptions>(
      c, "TRenderOptions", "SparseVoxelPointCloud.TRenderOptions", "Options for rendering.")
      .def_readwrite("point_size", &T::TRenderOptions::point_size, "Point size [pixels].")
      .def_readwrite(
          "show_mean_only", &T::TRenderOptions::show_mean_only,
          "Render only the mean point of each voxel.")
      .def_readwrite(
          "show_inner_grid_boxes", &T::TRenderOptions::show_inner_grid_boxes,
          "Render the boxes of the dense blocks.")
      .def_readwrite(
          "color", &T::TRenderOptions::color,
          "Color of points (mrpt.img.TColorf), if colormap is cmNONE.")
      .def_readwrite("colormap", &T::TRenderOptions::colormap, "Colormap (mrpt.img.TColormap).")
      .def_readwrite(
          "recolorByPointField", &T::TRenderOptions::recolorByPointField,
          "Point field used for the colormap.");

  c.def(
       py::init<float>(), "voxel_size"_a = 0.20f,
       "Creates an empty map, with voxels of this size [m].")
      .def(
          "setVoxelProperties", &T::setVoxelProperties, "voxel_size"_a,
          "Changes the voxel size [m], CLEARING the map.")
      .def(
          "insertPoint", [](T& self, const py::object& pt) { self.insertPoint(to_point3df(pt)); },
          "point"_a, "Inserts one point, in map coordinates (mrpt.math.TPoint3D or [x, y, z]).")
      .def(
          "getPointsAsNumpy", [](const T& self) { return all_points(self); },
          "All points in the map, as an (N,3) float32 array.")
      .def(
          "getVoxelsAsNumpy",
          [](const T& self)
          {
            VoxelColumns       cols;
            std::vector<float> means;
            self.visitAllVoxels(
                [&](const T::outer_index3d_t& outer, const T::inner_plain_index_t plain,
                    const T::VoxelData& v, const T::InnerGrid&)
                {
                  if (v.size() == 0)
                  {
                    return;
                  }
                  const auto                inner = T::plain2i(plain);
                  const T::global_index3d_t idx(
                      outer.cx + static_cast<int32_t>(inner.cx),
                      outer.cy + static_cast<int32_t>(inner.cy),
                      outer.cz + static_cast<int32_t>(inner.cz));
                  cols.add(idx, self.globalIdxToCoord(idx), v.size());
                  const auto& mean = v.mean();
                  means.insert(means.end(), {mean.x, mean.y, mean.z});
                });
            auto d     = cols.as_dict();
            d["means"] = to_array(means, {cols.size(), 3});
            return d;
          },
          "The non-empty voxels, as a dict of arrays: 'indices' ((M,3) int32), 'centers' ((M,3) "
          "float32), 'num_points' (uint32) and 'means' ((M,3) float32, mean of their points).")
      .def(
          "saveToTextFile", &T::saveToTextFile, "file"_a,
          "Saves all points to a text file, one 'x y z' per line. Returns False on error.")
      .def_readwrite(
          "insertionOptions", &T::insertionOptions, "Insertion options (TInsertionOptions).")
      .def_readwrite(
          "likelihoodOptions", &T::likelihoodOptions, "Likelihood options (TLikelihoodOptions).")
      .def_readwrite("renderOptions", &T::renderOptions, "Render options (TRenderOptions).")
      .def(
          "__repr__",
          [](const T& self)
          {
            return mrpt::format(
                "SparseVoxelPointCloud(voxel_size=%g, blocks=%zu)",
                static_cast<double>(self.creationOptions.voxel_size), self.grids().size());
          });

  def_options_capable<T>(c);
  def_insert_point_cloud<T>(c);
  def_nn_search<T>(c);
  def_copy_and_pickle<T>(c);
}

// -----------------------------------------------------------------------------
// SparseTreesPointCloud
// -----------------------------------------------------------------------------
void init_sparse_trees_point_cloud(py::module_& m)
{
  using T = mola::SparseTreesPointCloud;

  py::class_<T, CMetricMap, std::shared_ptr<T>> c(
      m, "SparseTreesPointCloud",
      "A point cloud split into a sparse grid of blocks, each a point cloud with its own k-d "
      "tree (mola::SparseTreesPointCloud). Experimental.");

  bind_options<T::TCreationOptions>(
      c, "TCreationOptions", "SparseTreesPointCloud.TCreationOptions",
      "Options that define the map structure.")
      .def_readwrite("grid_size", &T::TCreationOptions::grid_size, "Size of the blocks [m].");

  bind_options<T::TInsertionOptions>(
      c, "TInsertionOptions", "SparseTreesPointCloud.TInsertionOptions",
      "Options for inserting observations.")
      .def_readwrite(
          "minimum_points_clearance", &T::TInsertionOptions::minimum_points_clearance,
          "Points closer than this [m] to an existing one are not inserted.")
      .def_readwrite(
          "remove_submaps_farther_than", &T::TInsertionOptions::remove_submaps_farther_than,
          "If not 0, blocks farther than this distance [m] from the robot are removed on each "
          "insertion.");

  bind_options<T::TLikelihoodOptions>(
      c, "TLikelihoodOptions", "SparseTreesPointCloud.TLikelihoodOptions",
      "Options for computing the likelihood of observations.")
      .def_readwrite(
          "sigma_dist", &T::TLikelihoodOptions::sigma_dist,
          "Divisor of the squared point-to-map distance in the log-likelihood [m^2].")
      .def_readwrite(
          "max_corr_distance", &T::TLikelihoodOptions::max_corr_distance,
          "Maximum distance [m] of a point to the map, to bound its likelihood.")
      .def_readwrite("decimation", &T::TLikelihoodOptions::decimation, "Use one out of N points.");

  bind_options<T::TRenderOptions>(
      c, "TRenderOptions", "SparseTreesPointCloud.TRenderOptions", "Options for rendering.")
      .def_readwrite("point_size", &T::TRenderOptions::point_size, "Point size [pixels].")
      .def_readwrite(
          "show_inner_grid_boxes", &T::TRenderOptions::show_inner_grid_boxes,
          "Render the boxes of the blocks.")
      .def_readwrite(
          "color", &T::TRenderOptions::color,
          "Color of points (mrpt.img.TColorf), if colormap is cmNONE.")
      .def_readwrite("colormap", &T::TRenderOptions::colormap, "Colormap (mrpt.img.TColormap).")
      .def_readwrite(
          "recolorByPointField", &T::TRenderOptions::recolorByPointField,
          "Point field used for the colormap.");

  c.def(
       py::init<float>(), "grid_size"_a = 10.0f,
       "Creates an empty map, with blocks of this size [m].")
      .def(
          "setGridProperties", &T::setGridProperties, "grid_size"_a,
          "Changes the size of the blocks [m], CLEARING the map.")
      .def(
          "insertPoint", [](T& self, const py::object& pt) { self.insertPoint(to_point3df(pt)); },
          "point"_a, "Inserts one point, in map coordinates (mrpt.math.TPoint3D or [x, y, z]).")
      .def(
          "eraseGridsFartherThan",
          [](T& self, const py::object& pt, float distance)
          { self.eraseGridsFartherThan(to_point3df(pt), distance); },
          "point"_a, "distance"_a, "Removes the blocks entirely farther than `distance` [m].")
      .def(
          "getPointsAsNumpy", [](const T& self) { return all_points(self); },
          "All points in the map, as an (N,3) float32 array.")
      .def(
          "getGridsAsNumpy",
          [](const T& self)
          {
            const float  half = 0.5f * self.creationOptions.grid_size;
            VoxelColumns cols;
            self.visitAllGrids(
                [&](const T::outer_index3d_t& idx, const T::GridData& g)
                {
                  const auto corner = self.outerIdxToCoord(idx);
                  cols.add(
                      idx, {corner.x + half, corner.y + half, corner.z + half}, g.points().size());
                });
            return cols.as_dict();
          },
          "The blocks, as a dict of arrays: 'indices' ((M,3) int32), 'centers' ((M,3) float32) "
          "and 'num_points' (uint32).")
      .def(
          "saveToTextFile", &T::saveToTextFile, "file"_a,
          "Saves all points to a text file, one 'x y z' per line. Returns False on error.")
      .def_readwrite(
          "insertionOptions", &T::insertionOptions, "Insertion options (TInsertionOptions).")
      .def_readwrite(
          "likelihoodOptions", &T::likelihoodOptions, "Likelihood options (TLikelihoodOptions).")
      .def_readwrite("renderOptions", &T::renderOptions, "Render options (TRenderOptions).")
      .def(
          "__repr__",
          [](const T& self)
          {
            return mrpt::format(
                "SparseTreesPointCloud(grid_size=%g, blocks=%zu)",
                static_cast<double>(self.creationOptions.grid_size), self.grids().size());
          });

  def_options_capable<T>(c);
  def_insert_point_cloud<T>(c);
  def_nn_search<T>(c);
  def_copy_and_pickle<T>(c);
}

// -----------------------------------------------------------------------------
// NDT
// -----------------------------------------------------------------------------
void init_ndt(py::module_& m)
{
  using T = mola::NDT;

  py::class_<T, CMetricMap, std::shared_ptr<T>> c(
      m, "NDT",
      "3D Normal Distributions Transform map: voxels with up to 16 points and the Gaussian "
      "fitted to them, which may be a plane (mola::NDT).");

  bind_options<T::TCreationOptions>(
      c, "TCreationOptions", "NDT.TCreationOptions", "Options that define the map structure.")
      .def_readwrite("voxel_size", &T::TCreationOptions::voxel_size, "Voxel size [m].");

  bind_options<T::TInsertionOptions>(
      c, "TInsertionOptions", "NDT.TInsertionOptions", "Options for inserting observations.")
      .def_readwrite(
          "max_points_per_voxel", &T::TInsertionOptions::max_points_per_voxel,
          "Maximum number of points per voxel (0: no limit other than 16).")
      .def_readwrite(
          "remove_voxels_farther_than", &T::TInsertionOptions::remove_voxels_farther_than,
          "If not 0, voxels farther than this distance [m] from the robot are removed on each "
          "insertion.")
      .def_readwrite(
          "min_distance_between_points", &T::TInsertionOptions::min_distance_between_points,
          "If not 0, points closer than this [m] to another one in its voxel are not inserted.")
      .def_readwrite(
          "max_eigen_ratio_for_planes", &T::TInsertionOptions::max_eigen_ratio_for_planes,
          "A voxel is a plane if its smallest eigenvalue is below this ratio of the second one.");

  bind_options<T::TLikelihoodOptions>(
      c, "TLikelihoodOptions", "NDT.TLikelihoodOptions",
      "Options for computing the likelihood of observations (not used yet).")
      .def_readwrite(
          "sigma_dist", &T::TLikelihoodOptions::sigma_dist,
          "Divisor of the squared point-to-map distance in the log-likelihood [m^2].")
      .def_readwrite(
          "max_corr_distance", &T::TLikelihoodOptions::max_corr_distance,
          "Maximum distance [m] of a point to the map, to bound its likelihood.")
      .def_readwrite("decimation", &T::TLikelihoodOptions::decimation, "Use one out of N points.");

  bind_options<T::TRenderOptions>(
      c, "TRenderOptions", "NDT.TRenderOptions", "Options for rendering.")
      .def_readwrite("points_visible", &T::TRenderOptions::points_visible, "Render the points.")
      .def_readwrite("point_size", &T::TRenderOptions::point_size, "Point size [pixels].")
      .def_readwrite(
          "points_color", &T::TRenderOptions::points_color,
          "Color of points (mrpt.img.TColorf), if points_colormap is cmNONE.")
      .def_readwrite("planes_visible", &T::TRenderOptions::planes_visible, "Render the planes.")
      .def_readwrite(
          "planes_color", &T::TRenderOptions::planes_color,
          "Color of planes (mrpt.img.TColorf), if planes_colormap is cmNONE.")
      .def_readwrite(
          "normals_visible", &T::TRenderOptions::normals_visible, "Render the plane normals.")
      .def_readwrite(
          "normals_color", &T::TRenderOptions::normals_color,
          "Color of normals (mrpt.img.TColorf).")
      .def_readwrite(
          "points_colormap", &T::TRenderOptions::points_colormap,
          "Colormap of points (mrpt.img.TColormap).")
      .def_readwrite(
          "planes_colormap", &T::TRenderOptions::planes_colormap,
          "Colormap of planes (mrpt.img.TColormap).")
      .def_readwrite(
          "recolorByPointField", &T::TRenderOptions::recolorByPointField,
          "Point field used for the colormap.");

  c.def(
       py::init<float>(), "voxel_size"_a = 5.0f,
       "Creates an empty map, with voxels of this size [m].")
      .def(
          "setVoxelProperties", &T::setVoxelProperties, "voxel_size"_a,
          "Changes the voxel size [m], CLEARING the map.")
      .def(
          "insertPoint",
          [](T& self, const py::object& pt, const py::object& sensor)
          { self.insertPoint(to_point3df(pt), to_point3df(sensor)); },
          "point"_a, "sensor"_a,
          "Inserts one point, and the sensor position it was seen from (used to orient the "
          "plane normals), both in map coordinates (mrpt.math.TPoint3D or [x, y, z]).")
      .def(
          "voxelCount", [](const T& self) { return self.voxels().size(); },
          "Number of voxels in the map.")
      .def(
          "getPointsAsNumpy", [](const T& self) { return all_points(self); },
          "All points in the map, as an (N,3) float32 array.")
      .def(
          "getVoxelsAsNumpy",
          [](const T& self)
          {
            const float        half = 0.5f * self.creationOptions.voxel_size;
            const float        nan  = std::numeric_limits<float>::quiet_NaN();
            VoxelColumns       cols;
            std::vector<bool>  hasNdt;
            std::vector<bool>  isPlane;
            std::vector<float> means;
            std::vector<float> eigVals;
            std::vector<float> normals;
            self.visitAllVoxels(
                [&](const T::global_index3d_t& idx, const T::VoxelData& v)
                {
                  const auto corner = self.globalIdxToCoord(idx);
                  cols.add(idx, {corner.x + half, corner.y + half, corner.z + half}, v.size());
                  const auto& ndt = v.ndt();
                  hasNdt.push_back(ndt.has_value());
                  isPlane.push_back(ndt.has_value() && self.ndt_is_plane(*ndt));
                  if (!ndt)
                  {
                    means.insert(means.end(), {nan, nan, nan});
                    eigVals.insert(eigVals.end(), {nan, nan, nan});
                    normals.insert(normals.end(), {nan, nan, nan});
                    return;
                  }
                  const auto& mu = ndt->meanCov.mean;
                  const auto& n  = ndt->eigVectors[0];
                  means.insert(
                      means.end(), {static_cast<float>(mu.x()), static_cast<float>(mu.y()),
                                    static_cast<float>(mu.z())});
                  eigVals.insert(
                      eigVals.end(),
                      {static_cast<float>(ndt->eigVals[0]), static_cast<float>(ndt->eigVals[1]),
                       static_cast<float>(ndt->eigVals[2])});
                  normals.insert(
                      normals.end(),
                      {static_cast<float>(n.x), static_cast<float>(n.y), static_cast<float>(n.z)});
                });
            auto              d = cols.as_dict();
            py::array_t<bool> hasNdtArr(std::vector<py::ssize_t>{cols.size()});
            py::array_t<bool> isPlaneArr(std::vector<py::ssize_t>{cols.size()});
            for (py::ssize_t i = 0; i < cols.size(); i++)
            {
              hasNdtArr.mutable_at(i)  = hasNdt[i];
              isPlaneArr.mutable_at(i) = isPlane[i];
            }
            d["has_ndt"]     = hasNdtArr;
            d["is_plane"]    = isPlaneArr;
            d["means"]       = to_array(means, {cols.size(), 3});
            d["eigenvalues"] = to_array(eigVals, {cols.size(), 3});
            d["normals"]     = to_array(normals, {cols.size(), 3});
            return d;
          },
          "The voxels, as a dict of arrays: 'indices' ((M,3) int32), 'centers' ((M,3) "
          "float32), 'num_points' (uint32), 'has_ndt' (bool: enough points to fit a Gaussian), "
          "'is_plane' (bool), and from the Gaussian, NaN if there is none: 'means' ((M,3) "
          "float32), 'eigenvalues' ((M,3) float32, ascending) and 'normals' ((M,3) float32, "
          "eigenvector of the smallest eigenvalue, pointing to the sensor).")
      .def(
          "saveToTextFile", &T::saveToTextFile, "file"_a,
          "Saves all points to a text file, one 'x y z' per line. Returns False on error.")
      .def_readwrite(
          "insertionOptions", &T::insertionOptions, "Insertion options (TInsertionOptions).")
      .def_readwrite(
          "likelihoodOptions", &T::likelihoodOptions, "Likelihood options (TLikelihoodOptions).")
      .def_readwrite("renderOptions", &T::renderOptions, "Render options (TRenderOptions).")
      .def(
          "__repr__",
          [](const T& self)
          {
            return mrpt::format(
                "NDT(voxel_size=%g, voxels=%zu)",
                static_cast<double>(self.creationOptions.voxel_size), self.voxels().size());
          });

  def_options_capable<T>(c);
  def_insert_point_cloud<T>(c);
  def_nn_search<T>(c);
  def_pt2pl_search<T>(c);
  def_copy_and_pickle<T>(c);
}

// -----------------------------------------------------------------------------
// TSDF
// -----------------------------------------------------------------------------
void init_tsdf(py::module_& m)
{
  using T = mola::TSDF;

  py::class_<T, CMetricMap, std::shared_ptr<T>> c(
      m, "TSDF",
      "Truncated signed distance field, stored as a voxel hash map (mola::TSDF). Each point "
      "inserted with its sensor position updates the voxels around its ray.");

  bind_options<T::TCreationOptions>(
      c, "TCreationOptions", "TSDF.TCreationOptions", "Options that define the map structure.")
      .def_readwrite("voxel_size", &T::TCreationOptions::voxel_size, "Voxel size [m].");

  using Ins = T::TInsertionOptions;
  bind_options<Ins>(
      c, "TInsertionOptions", "TSDF.TInsertionOptions", "Options for inserting observations.")
      .def_readwrite(
          "truncation_distance", &Ins::truncation_distance,
          "Half-width [m] of the band updated around surfaces, also the maximum distance at "
          "which a query finds a surface. If <=0, truncation_voxels times the voxel size.")
      .def_readwrite(
          "truncation_voxels", &Ins::truncation_voxels,
          "Truncation distance in voxels, if truncation_distance <= 0.")
      .def_readwrite(
          "ray_tube_voxels", &Ins::ray_tube_voxels,
          "Radius, in voxels, of the cylinder around each ray that is updated.")
      .def_readwrite(
          "tube_sigma_voxels", &Ins::tube_sigma_voxels,
          "Sigma, in voxels, of the Gaussian profile across the ray tube (<=0: flat).")
      .def_readwrite("max_weight", &Ins::max_weight, "Weight cap of a voxel.")
      .def_readwrite(
          "min_weight_for_query", &Ins::min_weight_for_query,
          "Minimum weight of a voxel for it to take part in a query.")
      .def_readwrite(
          "remove_voxels_farther_than", &Ins::remove_voxels_farther_than,
          "If not 0, voxels farther than this distance [m] from the sensor are removed.")
      .def_readwrite(
          "max_voxels", &Ins::max_voxels, "If not 0, maximum number of voxels in the map.")
      .def_readwrite(
          "weight_by_range", &Ins::weight_by_range, "Weight samples by (weight_range_ref/range)^2.")
      .def_readwrite(
          "weight_range_ref", &Ins::weight_range_ref, "Reference range [m] for weight_by_range.")
      .def_readwrite(
          "weight_by_incidence", &Ins::weight_by_incidence,
          "Weight samples by the cosine of their incidence angle.")
      .def_readwrite(
          "bootstrap_with_points", &Ins::bootstrap_with_points,
          "Answer queries from the points of the first scans until the field can answer.")
      .def_readwrite(
          "bootstrap_min_scans", &Ins::bootstrap_min_scans,
          "Scans to insert before the field is tested.")
      .def_readwrite(
          "bootstrap_max_scans", &Ins::bootstrap_max_scans, "Maximum length of the warm-up.")
      .def_readwrite(
          "bootstrap_field_ready_fraction", &Ins::bootstrap_field_ready_fraction,
          "Fraction of the last scan the field must answer to end the warm-up.")
      .def_readwrite("bootstrap_knn", &Ins::bootstrap_knn, "Neighbors for the warm-up plane fits.")
      .def_readwrite(
          "bootstrap_max_plane_deviation", &Ins::bootstrap_max_plane_deviation,
          "Maximum distance [m] of a neighbor to a warm-up plane (<=0: no limit).");

  bind_options<T::TRenderOptions>(
      c, "TRenderOptions", "TSDF.TRenderOptions", "Options for rendering.")
      .def_readwrite("point_size", &T::TRenderOptions::point_size, "Point size [pixels].")
      .def_readwrite(
          "points_color", &T::TRenderOptions::points_color,
          "Color of the surface (mrpt.img.TColorf), if colormap is cmNONE, and of the mesh.")
      .def_readwrite("colormap", &T::TRenderOptions::colormap, "Colormap (mrpt.img.TColormap).")
      .def_readwrite(
          "recolorize_by", &T::TRenderOptions::recolorize_by,
          "Quantity used for the colormap: 'z' or 'weight'.")
      .def_readwrite(
          "render_as_mesh", &T::TRenderOptions::render_as_mesh,
          "Render the surface as a mesh instead of points (slower).");

  c.def(
       py::init<float>(), "voxel_size"_a = 0.30f,
       "Creates an empty map, with voxels of this size [m].")
      .def(
          "setVoxelProperties", &T::setVoxelProperties, "voxel_size"_a,
          "Changes the voxel size [m], CLEARING the map.")
      .def("voxelSize", &T::voxelSize, "Voxel size [m].")
      .def("size", &T::size, "Number of voxels.")
      .def("__len__", &T::size, "Number of voxels.")
      .def(
          "isBootstrapping", &T::isBootstrapping,
          "True while queries are answered from the warm-up point cloud, not from the field.")
      .def(
          "lastFieldReadyFraction", &T::lastFieldReadyFraction,
          "Fraction of the last inserted scan for which the field answered a query.")
      .def(
          "insertPoint",
          [](T& self, const py::object& pt, const py::object& sensor)
          { self.insertPoint(to_point3df(pt), to_point3df(sensor)); },
          "point"_a, "sensor"_a,
          "Fuses one measured point, and the origin of its ray, both in map coordinates "
          "(mrpt.math.TPoint3D or [x, y, z]).")
      .def(
          "queryField",
          [](const T& self, const py::object& pt) -> py::object
          {
            const auto q = self.queryField(to_point3df(pt));
            if (!q.valid)
            {
              return py::none();
            }
            py::array_t<float> g(std::vector<py::ssize_t>{3});
            g.mutable_at(0) = q.gradient.x;
            g.mutable_at(1) = q.gradient.y;
            g.mutable_at(2) = q.gradient.z;
            return py::make_tuple(q.dist, g);
          },
          "point"_a,
          "Trilinear interpolation of the field at a point. Returns (signed distance [m], "
          "unnormalized gradient as a float32 array), or None where the field is not defined.")
      .def(
          "queryFieldBatch",
          [](const T& self, const py::object& points)
          {
            const auto  arr = to_points_array(points);
            const auto  q   = arr.unchecked<2>();
            const auto  n   = q.shape(0);
            const float nan = std::numeric_limits<float>::quiet_NaN();

            py::array_t<bool>  valid(std::vector<py::ssize_t>{n});
            py::array_t<float> dist(std::vector<py::ssize_t>{n});
            py::array_t<float> grad(std::vector<py::ssize_t>{n, 3});
            auto               v = valid.mutable_unchecked<1>();
            auto               d = dist.mutable_unchecked<1>();
            auto               g = grad.mutable_unchecked<2>();
            {
              py::gil_scoped_release nogil;
              for (py::ssize_t i = 0; i < n; i++)
              {
                const auto r = self.queryField({q(i, 0), q(i, 1), q(i, 2)});
                v(i)         = r.valid;
                d(i)         = r.valid ? r.dist : nan;
                g(i, 0)      = r.valid ? r.gradient.x : nan;
                g(i, 1)      = r.valid ? r.gradient.y : nan;
                g(i, 2)      = r.valid ? r.gradient.z : nan;
              }
            }
            py::dict ret;
            ret["valid"]    = valid;
            ret["dist"]     = dist;
            ret["gradient"] = grad;
            return ret;
          },
          "points"_a,
          "queryField() for each row of an (N,3) array. Returns a dict with 'valid' (bool), "
          "'dist' (float32) and 'gradient' ((N,3) float32), NaN where not valid. Releases the "
          "GIL.")
      .def(
          "getPointsAsNumpy",
          [](const T& self)
          {
            std::vector<float> xyz;
            self.visitZeroCrossings(
                [&](const TPoint3Df& p, float) {
                  xyz.insert(xyz.end(), {p.x, p.y, p.z});
                });
            return to_array(xyz, {static_cast<py::ssize_t>(xyz.size() / 3), 3});
          },
          "The surface (zero level set of the field, sampled on the voxel lattice edges), as "
          "an (N,3) float32 array.")
      .def(
          "getZeroCrossingsAsNumpy",
          [](const T& self)
          {
            std::vector<float> xyz;
            std::vector<float> weights;
            self.visitZeroCrossings(
                [&](const TPoint3Df& p, float w)
                {
                  xyz.insert(xyz.end(), {p.x, p.y, p.z});
                  weights.push_back(w);
                });
            const auto n = static_cast<py::ssize_t>(weights.size());
            return py::make_tuple(to_array(xyz, {n, 3}), to_array(weights, {n}));
          },
          "Like getPointsAsNumpy(), returning a tuple (points: (N,3) float32, weights: float32 "
          "array with the smaller weight of the two voxels of each crossing).")
      .def(
          "getVoxelsAsNumpy",
          [](const T& self)
          {
            VoxelColumns       cols;
            std::vector<float> dist;
            std::vector<float> weight;
            self.visitAllVoxels(
                [&](const T::global_index3d_t& idx, const T::VoxelData& v)
                {
                  cols.add(idx, self.globalIdxToCenter(idx), 0);
                  dist.push_back(v.dist);
                  weight.push_back(v.weight);
                });
            py::dict d;
            d["indices"] = to_array(cols.indices, {cols.size(), 3});
            d["centers"] = to_array(cols.centers, {cols.size(), 3});
            d["dist"]    = to_array(dist, {cols.size()});
            d["weight"]  = to_array(weight, {cols.size()});
            return d;
          },
          "The voxels, as a dict of arrays: 'indices' ((M,3) int32), 'centers' ((M,3) "
          "float32), 'dist' (float32, signed distance [m], positive towards the sensor) and "
          "'weight' (float32).")
      .def_readwrite(
          "insertionOptions", &T::insertionOptions, "Insertion options (TInsertionOptions).")
      .def_readwrite("renderOptions", &T::renderOptions, "Render options (TRenderOptions).")
      .def(
          "__repr__",
          [](const T& self)
          {
            return mrpt::format(
                "TSDF(voxel_size=%g, voxels=%zu%s)", static_cast<double>(self.voxelSize()),
                self.size(), self.isBootstrapping() ? ", warming up" : "");
          });

  def_options_capable<T>(c);
  def_insert_point_cloud<T>(c);
  def_pt2pl_search<T>(c);
  def_copy_and_pickle<T>(c);
}

}  // namespace

void init_voxel_maps(py::module_& m)
{
  init_hashed_voxel_point_cloud(m);
  init_sparse_voxel_point_cloud(m);
  init_sparse_trees_point_cloud(m);
  init_ndt(m);
  init_tsdf(m);
}

}  // namespace mola_metric_maps_py
