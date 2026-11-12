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
 * @file   py_common.cpp
 * @brief  Helpers shared by the Python bindings of mola_metric_maps
 * @author Jose Luis Blanco Claraco
 * @date   Nov 12, 2026
 */

#include "py_common.h"

#include <mrpt/core/Clock.h>
#include <mrpt/maps/CSimplePointsMap.h>
#include <mrpt/math/CMatrixFixed.h>
#include <mrpt/math/TPose3D.h>
#include <mrpt/obs/CObservationPointCloud.h>
#include <mrpt/system/string_utils.h>

#include <limits>
#include <sstream>

namespace mola_metric_maps_py
{
mrpt::math::TPoint3Df to_point3df(const py::object& obj)
{
  if (py::isinstance<mrpt::math::TPoint3D>(obj))
  {
    const auto p = obj.cast<mrpt::math::TPoint3D>();
    return {static_cast<float>(p.x), static_cast<float>(p.y), static_cast<float>(p.z)};
  }
  if (py::isinstance<mrpt::math::TPoint3Df>(obj))
  {
    return obj.cast<mrpt::math::TPoint3Df>();
  }
  const auto arr = FloatArray::ensure(obj);
  if (!arr || arr.ndim() != 1 || arr.shape(0) != 3)
  {
    throw py::type_error("Expected a 3D point: mrpt.math.TPoint3D, or a sequence [x, y, z]");
  }
  return {arr.at(0), arr.at(1), arr.at(2)};
}

mrpt::poses::CPose3D to_pose3d(const py::object& obj)
{
  if (obj.is_none())
  {
    return {};
  }
  if (py::isinstance<mrpt::poses::CPose3D>(obj))
  {
    return obj.cast<mrpt::poses::CPose3D>();
  }
  if (py::isinstance<mrpt::math::TPose3D>(obj))
  {
    return mrpt::poses::CPose3D(obj.cast<mrpt::math::TPose3D>());
  }
  const auto arr = py::array_t<double, py::array::c_style | py::array::forcecast>::ensure(obj);
  if (arr && arr.ndim() == 2 && arr.shape(0) == 4 && arr.shape(1) == 4)
  {
    mrpt::math::CMatrixDouble44 M;
    for (int r = 0; r < 4; r++)
    {
      for (int c = 0; c < 4; c++)
      {
        M(r, c) = arr.at(r, c);
      }
    }
    return mrpt::poses::CPose3D(M);
  }
  if (arr && arr.ndim() == 1 && arr.shape(0) == 6)
  {
    return mrpt::poses::CPose3D(arr.at(0), arr.at(1), arr.at(2), arr.at(3), arr.at(4), arr.at(5));
  }
  throw py::type_error(
      "Cannot convert into a pose: expected None, mrpt.poses.CPose3D, mrpt.math.TPose3D, a 4x4 "
      "matrix, or [x, y, z, yaw, pitch, roll]");
}

std::optional<mrpt::poses::CPose3D> to_optional_pose3d(const py::object& obj)
{
  if (obj.is_none())
  {
    return std::nullopt;
  }
  return to_pose3d(obj);
}

py::array_t<float> points_to_numpy(const std::vector<mrpt::math::TPoint3Df>& pts)
{
  const auto         n = static_cast<py::ssize_t>(pts.size());
  py::array_t<float> arr(std::vector<py::ssize_t>{n, 3});
  auto               a = arr.mutable_unchecked<2>();
  for (py::ssize_t i = 0; i < n; i++)
  {
    a(i, 0) = pts[i].x;
    a(i, 1) = pts[i].y;
    a(i, 2) = pts[i].z;
  }
  return arr;
}

FloatArray to_points_array(const py::object& obj)
{
  const auto arr = FloatArray::ensure(obj);
  if (!arr || arr.ndim() != 2 || arr.shape(1) != 3)
  {
    throw py::value_error("Expected an array of shape (N,3) with point coordinates");
  }
  return arr;
}

mrpt::maps::CPointsMap::Ptr to_point_cloud(const py::object& obj)
{
  if (py::isinstance<mrpt::maps::CPointsMap>(obj))
  {
    // A copy, since some maps keep a reference to the inserted cloud:
    const auto& pc = obj.cast<const mrpt::maps::CPointsMap&>();
    return std::dynamic_pointer_cast<mrpt::maps::CPointsMap>(pc.duplicateGetSmartPtr());
  }
  if (py::hasattr(obj, "items"))
  {
    return py::module_::import("mp2p_icp")
        .attr("layer_from_numpy")(obj)
        .cast<mrpt::maps::CPointsMap::Ptr>();
  }
  const auto arr = to_points_array(obj);
  const auto r   = arr.unchecked<2>();
  auto       pc  = mrpt::maps::CSimplePointsMap::Create();
  pc->reserve(static_cast<size_t>(r.shape(0)));
  for (py::ssize_t i = 0; i < r.shape(0); i++)
  {
    pc->insertPointFast(r(i, 0), r(i, 1), r(i, 2));
  }
  pc->mark_as_modified();
  return pc;
}

bool insert_point_cloud(
    mrpt::maps::CMetricMap& map, const py::object& points, const py::object& robotPose)
{
  auto obs        = mrpt::obs::CObservationPointCloud::Create();
  obs->pointcloud = to_point_cloud(points);
  obs->timestamp  = mrpt::Clock::now();

  const auto pose = to_optional_pose3d(robotPose);

  py::gil_scoped_release nogil;
  return map.insertObservation(*obs, pose);
}

py::bytes to_bytes(const mrpt::serialization::CSerializable& o)
{
  mrpt::io::CMemoryStream buf;
  auto                    arch = mrpt::serialization::archiveFrom(buf);
  arch << o;
  return {
      static_cast<const char*>(buf.getRawBufferData()),
      static_cast<size_t>(buf.getTotalBytesCount())};
}

void from_bytes(mrpt::serialization::CSerializable& o, const py::bytes& data)
{
  const std::string_view  sv = data;
  mrpt::io::CMemoryStream buf;
  buf.assignMemoryNotOwn(sv.data(), sv.size());
  auto arch = mrpt::serialization::archiveFrom(buf);
  arch >> o;
}

std::string options_repr(const std::string& name, const mrpt::config::CLoadableOptions& o)
{
  std::stringstream ss;
  o.dumpToTextStream(ss);

  std::string ret   = name + "(";
  bool        first = true;
  std::string line;
  while (std::getline(ss, line))
  {
    const auto eq = line.find('=');
    if (eq == std::string::npos)
    {
      continue;
    }
    if (!first)
    {
      ret += ", ";
    }
    first = false;
    ret += mrpt::system::trim(line.substr(0, eq)) + "=" + mrpt::system::trim(line.substr(eq + 1));
  }
  return ret + ")";
}

py::dict nn_single_search_batch(
    const mrpt::maps::NearestNeighborsCapable& nn, const py::object& queries)
{
  const auto  arr = to_points_array(queries);
  const auto  q   = arr.unchecked<2>();
  const auto  n   = q.shape(0);
  const float nan = std::numeric_limits<float>::quiet_NaN();
  const float inf = std::numeric_limits<float>::infinity();

  py::array_t<bool>     found(std::vector<py::ssize_t>{n});
  py::array_t<uint64_t> indices(std::vector<py::ssize_t>{n});
  py::array_t<float>    points(std::vector<py::ssize_t>{n, 3});
  py::array_t<float>    distSqr(std::vector<py::ssize_t>{n});

  auto f = found.mutable_unchecked<1>();
  auto i = indices.mutable_unchecked<1>();
  auto p = points.mutable_unchecked<2>();
  auto d = distSqr.mutable_unchecked<1>();
  {
    py::gil_scoped_release nogil;
    for (py::ssize_t k = 0; k < n; k++)
    {
      mrpt::math::TPoint3Df result;
      float                 dist2 = 0;
      uint64_t              idx   = 0;
      const bool            ok =
          nn.nn_single_search(mrpt::math::TPoint3Df(q(k, 0), q(k, 1), q(k, 2)), result, dist2, idx);
      f(k)    = ok;
      i(k)    = ok ? idx : 0;
      p(k, 0) = ok ? result.x : nan;
      p(k, 1) = ok ? result.y : nan;
      p(k, 2) = ok ? result.z : nan;
      d(k)    = ok ? dist2 : inf;
    }
  }

  py::dict ret;
  ret["found"]    = found;
  ret["indices"]  = indices;
  ret["points"]   = points;
  ret["dist_sqr"] = distSqr;
  return ret;
}

}  // namespace mola_metric_maps_py
