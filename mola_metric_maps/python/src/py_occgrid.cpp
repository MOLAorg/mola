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
 * @file   py_occgrid.cpp
 * @brief  Python bindings: OccGrid
 * @author Jose Luis Blanco Claraco
 * @date   Nov 12, 2026
 */

#include <mola_metric_maps/OccGrid.h>
#include <mrpt/core/format.h>
#include <mrpt/maps/COccupancyGridMap2D.h>
#include <mrpt/math/TPoint2D.h>
#include <mrpt/math/TPose2D.h>
#include <mrpt/obs/CObservation2DRangeScan.h>

#include "py_common.h"

namespace mola_metric_maps_py
{
namespace
{
mrpt::math::TPoint2Df to_point2df(const py::object& obj)
{
  if (py::isinstance<mrpt::math::TPoint2D>(obj))
  {
    const auto p = obj.cast<mrpt::math::TPoint2D>();
    return {static_cast<float>(p.x), static_cast<float>(p.y)};
  }
  const auto arr = FloatArray::ensure(obj);
  if (!arr || arr.ndim() != 1 || arr.shape(0) != 2)
  {
    throw py::type_error("Expected a 2D point: mrpt.math.TPoint2D, or a sequence [x, y]");
  }
  return {arr.at(0), arr.at(1)};
}
}  // namespace

void init_occgrid(py::module_& m)
{
  using T = mola::OccGrid;

  py::class_<T, mrpt::serialization::CSerializable, std::shared_ptr<T>> c(
      m, "OccGrid",
      "A 2D occupancy grid (mrpt.maps.COccupancyGridMap2D) with a super-resolution likelihood "
      "cache (mola::OccGrid). Experimental: the cache is not used yet.");

  using Ins = T::InsertionParameters;
  py::class_<Ins>(c, "InsertionParameters", "Parameters for inserting range scans.")
      .def(py::init<>(), "Default parameters.")
      .def_readwrite(
          "maxDistanceInsertion", &Ins::maxDistanceInsertion,
          "Largest distance [m] at which cells are updated.")
      .def_readwrite(
          "maxOccupancyUpdateCertainty", &Ins::maxOccupancyUpdateCertainty,
          "Value in [0.5, 1] for the Bayesian update of occupied cells.")
      .def_readwrite(
          "maxFreenessUpdateCertainty", &Ins::maxFreenessUpdateCertainty,
          "Value in [0.5, 1] for the update of free cells (0: maxOccupancyUpdateCertainty).")
      .def_readwrite(
          "maxFreenessInvalidRanges", &Ins::maxFreenessInvalidRanges,
          "Like maxFreenessUpdateCertainty, for invalid ranges (0: maxOccupancyUpdateCertainty).")
      .def_readwrite(
          "considerInvalidRangesAsFreeSpace", &Ins::considerInvalidRangesAsFreeSpace,
          "Use invalid ranges as free space, if their neighbor rays are also invalid.")
      .def_readwrite("decimation", &Ins::decimation, "Use one out of N rays.")
      .def_readwrite(
          "wideningBeamsWithDistance", &Ins::wideningBeamsWithDistance,
          "Rays widen with distance, like real laser beams.")
      .def(
          "__repr__",
          [](const Ins& p)
          {
            return mrpt::format(
                "OccGrid.InsertionParameters(maxDistanceInsertion=%g, "
                "maxOccupancyUpdateCertainty=%g, maxFreenessUpdateCertainty=%g, "
                "maxFreenessInvalidRanges=%g, considerInvalidRangesAsFreeSpace=%s, decimation=%u, "
                "wideningBeamsWithDistance=%s)",
                static_cast<double>(p.maxDistanceInsertion),
                static_cast<double>(p.maxOccupancyUpdateCertainty),
                static_cast<double>(p.maxFreenessUpdateCertainty),
                static_cast<double>(p.maxFreenessInvalidRanges),
                p.considerInvalidRangesAsFreeSpace ? "True" : "False",
                static_cast<unsigned int>(p.decimation),
                p.wideningBeamsWithDistance ? "True" : "False");
          });

  using Lik = T::LikelihoodParameters;
  py::class_<Lik>(c, "LikelihoodParameters", "Parameters of the likelihood cache.")
      .def(py::init<>(), "Default parameters.")
      .def_readwrite(
          "superResolutionFactor", &Lik::superResolutionFactor,
          "Resolution of the likelihood cache, as a factor of that of the grid.")
      .def(
          "__repr__",
          [](const Lik& p)
          {
            return mrpt::format(
                "OccGrid.LikelihoodParameters(superResolutionFactor=%g)", p.superResolutionFactor);
          });

  c.def(py::init<>(), "Creates an empty grid.")
      .def(
          "setSize",
          [](T& self, const py::object& minCorner, const py::object& maxCorner, float resolution,
             float occupancy)
          { self.setSize(to_point2df(minCorner), to_point2df(maxCorner), resolution, occupancy); },
          "min_corner"_a, "max_corner"_a, "resolution"_a, "occupancy"_a = 0.5f,
          "Changes the grid size, ERASING its contents. Corners are mrpt.math.TPoint2D or [x, "
          "y] [m], the resolution is the cell size [m].")
      .def(
          "resizeGrid",
          [](T& self, const py::object& minCorner, const py::object& maxCorner, float occupancy)
          { self.resizeGrid(to_point2df(minCorner), to_point2df(maxCorner), occupancy); },
          "min_corner"_a, "max_corner"_a, "new_cells_occupancy"_a = 0.5f,
          "Changes the grid size, keeping its contents.")
      .def(
          "insertObservation", &T::insertObservation, "obs"_a, "robotPose"_a,
          "Inserts a 2D range scan (mrpt.obs.CObservation2DRangeScan) taken from a robot pose "
          "(mrpt.math.TPose2D).")
      .def(
          "grid", &T::grid, py::return_value_policy::reference_internal,
          "The occupancy grid (mrpt.maps.COccupancyGridMap2D).")
      .def(
          "resetLikelihoodCache", &T::resetLikelihoodCache,
          "Invalidates all cached likelihood values.")
      .def_readwrite(
          "insertionParameters", &T::insertionParameters_,
          "Parameters for inserting range scans (InsertionParameters).")
      .def_readwrite(
          "likelihoodParameters", &T::likelihoodParameters_,
          "Parameters of the likelihood cache (LikelihoodParameters).")
      .def(
          "__repr__",
          [](const T& self)
          {
            const auto& g = self.grid();
            return mrpt::format(
                "OccGrid(%ux%u cells, resolution=%g)", static_cast<unsigned int>(g.getSizeX()),
                static_cast<unsigned int>(g.getSizeY()), static_cast<double>(g.getResolution()));
          });

  def_copy_and_pickle<T>(c);
}

}  // namespace mola_metric_maps_py
