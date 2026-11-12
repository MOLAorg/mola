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
 * @file   py_common.h
 * @brief  Helpers shared by the Python bindings of mola_metric_maps
 * @author Jose Luis Blanco Claraco
 * @date   Nov 12, 2026
 */
#pragma once

#include <mp2p_icp/NearestPlaneCapable.h>
#include <mp2p_icp/plane_patch.h>
#include <mrpt/config/CConfigFileBase.h>
#include <mrpt/config/CLoadableOptions.h>
#include <mrpt/io/CMemoryStream.h>
#include <mrpt/maps/CMetricMap.h>
#include <mrpt/maps/CPointsMap.h>
#include <mrpt/maps/NearestNeighborsCapable.h>
#include <mrpt/math/TPoint3D.h>
#include <mrpt/poses/CPose3D.h>
#include <mrpt/serialization/CArchive.h>
#include <pybind11/numpy.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

#include <functional>
#include <map>
#include <memory>
#include <optional>
#include <sstream>
#include <string>
#include <type_traits>
#include <vector>

namespace mola_metric_maps_py
{
namespace py = pybind11;
using namespace pybind11::literals;

using FloatArray = py::array_t<float, py::array::c_style | py::array::forcecast>;

/** Converts an mrpt.math.TPoint3D, TPoint3Df, or a sequence of 3 numbers */
mrpt::math::TPoint3Df to_point3df(const py::object& obj);

/** Converts None (identity), mrpt.poses.CPose3D, mrpt.math.TPose3D, a 4x4
 *  homogeneous matrix, or a sequence [x, y, z, yaw, pitch, roll] (meters,
 *  radians). */
mrpt::poses::CPose3D to_pose3d(const py::object& obj);

/** Like to_pose3d(), but None becomes std::nullopt */
std::optional<mrpt::poses::CPose3D> to_optional_pose3d(const py::object& obj);

/** An (N,3) float32 array from a list of points */
py::array_t<float> points_to_numpy(const std::vector<mrpt::math::TPoint3Df>& pts);

/** An (N,3) float32 array, validating the shape of the input */
FloatArray to_points_array(const py::object& obj);

/** A new point cloud from: an (N,3) array, a dict {field: 1-D array} (as
 *  mp2p_icp.layer_from_numpy()), or an mrpt.maps.CPointsMap (copied). */
mrpt::maps::CPointsMap::Ptr to_point_cloud(const py::object& obj);

/** Inserts a point cloud observation into a map, as insertObservation() with
 *  an mrpt::obs::CObservationPointCloud. Releases the GIL. */
bool insert_point_cloud(
    mrpt::maps::CMetricMap& map, const py::object& points, const py::object& robotPose);

/** Serializes an object (class name included) into Python bytes */
py::bytes to_bytes(const mrpt::serialization::CSerializable& o);

/** The reverse of to_bytes(), into an existing object of the same class */
void from_bytes(mrpt::serialization::CSerializable& o, const py::bytes& data);

/** Closest point to each row of an (N,3) array, as a dict of NumPy arrays */
py::dict nn_single_search_batch(
    const mrpt::maps::NearestNeighborsCapable& nn, const py::object& queries);

/** "Name(key=value, ...)" from the text dump of a set of options */
std::string options_repr(const std::string& name, const mrpt::config::CLoadableOptions& o);

/** @name Common bindings of several map classes
 *  @{ */

/** Declares a CLoadableOptions-derived struct, nested in `scope`, with a
 *  default constructor, __repr__ and __str__. Fields are added by the caller. */
template <class Opts>
py::class_<Opts, mrpt::config::CLoadableOptions, std::shared_ptr<Opts>> bind_options(
    py::handle scope, const char* name, const std::string& qualName, const char* doc)
{
  py::class_<Opts, mrpt::config::CLoadableOptions, std::shared_ptr<Opts>> c(scope, name, doc);
  c.def(py::init<>(), "Default options.")
      .def(
          "__repr__", [qualName](const Opts& o) { return options_repr(qualName, o); },
          "Options as Name(field=value, ...).")
      .def(
          "__str__",
          [](const Opts& o)
          {
            std::stringstream ss;
            o.dumpToTextStream(ss);
            return ss.str();
          },
          "Options as human-readable text.");
  return c;
}

/** A deep copy of a map, through serialization if it is not copyable */
template <class T>
std::shared_ptr<T> deep_copy(const T& self)
{
  if constexpr (std::is_copy_constructible_v<T>)
  {
    return std::make_shared<T>(self);
  }
  else
  {
    auto ret = std::make_shared<T>();
    from_bytes(*ret, to_bytes(self));
    return ret;
  }
}

/** Pickling (through MRPT serialization), copy and deepcopy */
template <class T, class PyClass>
void def_copy_and_pickle(PyClass& c)
{
  c.def(
       "__copy__", [](const T& self) { return deep_copy(self); }, "A deep copy of the map.")
      .def(
          "__deepcopy__", [](const T& self, const py::object&) { return deep_copy(self); },
          "memo"_a, "A deep copy of the map.")
      .def(py::pickle(
          [](const T& self) { return to_bytes(self); },
          [](const py::bytes& data)
          {
            auto ret = std::make_shared<T>();
            from_bytes(*ret, data);
            return ret;
          }));
}

/** insertPointCloud() of all CMetricMap classes */
template <class T, class PyClass>
void def_insert_point_cloud(PyClass& c)
{
  c.def(
      "insertPointCloud",
      [](T& self, const py::object& points, const py::object& robotPose)
      { return insert_point_cloud(self, points, robotPose); },
      "points"_a, "robotPose"_a = py::none(),
      "Inserts a point cloud, as insertObservation() with an mrpt.obs.CObservationPointCloud. "
      "`points` (an (N,3) array, a dict {field name: 1-D array} as in "
      "mp2p_icp.layer_from_numpy(), or an mrpt.maps.CPointsMap, which is copied) are given in "
      "the robot frame, and the robot is at `robotPose` in the map: None (identity), "
      "mrpt.poses.CPose3D, mrpt.math.TPose3D, a 4x4 matrix or [x, y, z, yaw, pitch, roll]. "
      "Maps that trace rays (TSDF, NDT) take the robot position as their origin. Insertion "
      "options and map-specific behavior apply, e.g. removal of far contents. Returns True if "
      "the map was updated.");
}

/** nn_*() queries of mrpt::maps::NearestNeighborsCapable, plus a NumPy batch
 *  query. The results have the same format as for mrpt.maps.CPointsMap. */
template <class T, class PyClass>
void def_nn_search(PyClass& c)
{
  using mrpt::maps::NearestNeighborsCapable;
  using mrpt::math::TPoint3Df;

  c.def(
       "nn_single_search",
       [](const T& self, const py::object& query) -> py::object
       {
         const auto& nn = static_cast<const NearestNeighborsCapable&>(self);
         TPoint3Df   result;
         float       distSqr = 0;
         uint64_t    idx     = 0;
         if (!nn.nn_single_search(to_point3df(query), result, distSqr, idx))
         {
           return py::none();
         }
         return py::make_tuple(idx, mrpt::math::TPoint3D(result), distSqr);
       },
       "query"_a,
       "Closest point to a 3D query point (mrpt.math.TPoint3D or [x, y, z]). Returns (index or "
       "ID, point, squared distance), or None if none was found.")
      .def(
          "nn_multiple_search",
          [](const T& self, const py::object& query, size_t N)
          {
            const auto&            nn = static_cast<const NearestNeighborsCapable&>(self);
            std::vector<TPoint3Df> pts;
            std::vector<float>     distsSqr;
            std::vector<uint64_t>  indices;
            nn.nn_multiple_search(to_point3df(query), N, pts, distsSqr, indices);
            return py::make_tuple(
                py::array_t<uint64_t>(
                    std::vector<py::ssize_t>{static_cast<py::ssize_t>(indices.size())},
                    indices.data()),
                points_to_numpy(pts),
                py::array_t<float>(
                    std::vector<py::ssize_t>{static_cast<py::ssize_t>(distsSqr.size())},
                    distsSqr.data()));
          },
          "query"_a, "N"_a,
          "The N closest points to a 3D query point, sorted by distance. Returns (indices or "
          "IDs: uint64 array, points: (N,3) float32 array, squared distances: float32 array).")
      .def(
          "nn_radius_search",
          [](const T& self, const py::object& query, float search_radius_sqr, size_t maxPoints)
          {
            const auto&            nn = static_cast<const NearestNeighborsCapable&>(self);
            std::vector<TPoint3Df> pts;
            std::vector<float>     distsSqr;
            std::vector<uint64_t>  indices;
            nn.nn_radius_search(
                to_point3df(query), search_radius_sqr, pts, distsSqr, indices, maxPoints);
            return py::make_tuple(
                py::array_t<uint64_t>(
                    std::vector<py::ssize_t>{static_cast<py::ssize_t>(indices.size())},
                    indices.data()),
                points_to_numpy(pts),
                py::array_t<float>(
                    std::vector<py::ssize_t>{static_cast<py::ssize_t>(distsSqr.size())},
                    distsSqr.data()));
          },
          "query"_a, "search_radius_sqr"_a, "maxPoints"_a = 0,
          "The points within a distance of a 3D query point. The radius is given squared; "
          "maxPoints=0 means no limit. Returns (indices or IDs: uint64 array, points: (N,3) "
          "float32 array, squared distances: float32 array).")
      .def(
          "nn_single_search_batch",
          [](const T& self, const py::object& queries) {
            return nn_single_search_batch(
                static_cast<const NearestNeighborsCapable&>(self), queries);
          },
          "queries"_a,
          "Closest point to each row of an (N,3) array of query points. Returns a dict with "
          "'found' (bool), 'indices' (uint64), 'points' ((N,3) float32, NaN if not found) and "
          "'dist_sqr' (float32, inf if not found). Releases the GIL.");
}

/** nn_search_pt2pl() of mp2p_icp::NearestPlaneCapable */
template <class T, class PyClass>
void def_pt2pl_search(PyClass& c)
{
  c.def(
      "nn_search_pt2pl",
      [](const T& self, const py::object& point, float max_search_distance) -> py::object
      {
        const auto& np = static_cast<const mp2p_icp::NearestPlaneCapable&>(self);
        const auto  r  = np.nn_search_pt2pl(to_point3df(point), max_search_distance);
        if (!r.pairing)
        {
          return py::none();
        }
        return py::make_tuple(r.pairing->pl_global, r.distance);
      },
      "point"_a, "max_search_distance"_a,
      "Closest plane to a 3D point (mrpt.math.TPoint3D or [x, y, z]), within a search "
      "distance. Returns (mp2p_icp.PlanePatch, absolute point-to-plane distance), or None.");
}

/** creationOptions (as a copy), trySetCreationOptions() and optionsByName() of
 *  the classes implementing mola::OptionsCapable */
template <class T, class PyClass>
void def_options_capable(PyClass& c)
{
  using CreationOptions = typename T::TCreationOptions;

  c.def_property(
       "creationOptions",
       [](const T& self) { return std::make_shared<CreationOptions>(self.creationOptions); },
       [](T& self, const CreationOptions& o)
       {
         if (!self.trySetCreationOptions(o))
         {
           throw py::value_error(
               "These creation options cannot be applied to a map that already holds data: "
               "clear() it first.");
         }
       },
       "Options that define the map structure (TCreationOptions). Reading it returns a COPY: "
       "modify it and assign it back, which applies it as trySetCreationOptions() does, raising "
       "ValueError if it is refused.")
      .def(
          "trySetCreationOptions",
          [](T& self, const CreationOptions& o) { return self.trySetCreationOptions(o); },
          "options"_a,
          "Applies new creation options. Returns False, leaving the map untouched, if they "
          "cannot be applied without discarding its contents (e.g. a different voxel size).")
      .def(
          "trySetCreationOptions",
          [](T& self, const mrpt::config::CConfigFileBase& cfg, const std::string& section)
          { return self.trySetCreationOptions(cfg, section); },
          "cfg"_a, "section"_a,
          "Like trySetCreationOptions(options), reading them from a section of a configuration "
          "file (mrpt.config.CConfigFileBase).")
      .def(
          "optionsByName",
          [](T& self)
          {
            std::map<std::string, mrpt::config::CLoadableOptions*> ret = self.optionsByName();
            return ret;
          },
          py::return_value_policy::reference_internal,
          "A dict {name: options} with all the option groups of the map, e.g. "
          "'insertionOptions'. They are the live objects, except that 'creationOptions' must "
          "not be modified in place: use trySetCreationOptions().");
}

/** @} */

}  // namespace mola_metric_maps_py
