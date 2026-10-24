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
 * @file   test-mola_metric_maps_voxel_indexing.cpp
 * @brief  Regression test: all voxels of the hashed voxel maps have the same
 *         size, including those touching the planes x=0, y=0 and z=0, and the
 *         index of a voxel matches the coordinates of the points in it.
 * @author Jose Luis Blanco Claraco
 * @date   Oct 24, 2026
 */

#include <mola_metric_maps/HashedVoxelPointCloud.h>
#include <mola_metric_maps/NDT.h>
#include <mola_metric_maps/SparseTreesPointCloud.h>
#include <mrpt/io/CMemoryStream.h>
#include <mrpt/serialization/CArchive.h>

#include <iostream>
#include <vector>

namespace
{
constexpr float CELL = 1.0f;

/** Points along `axis` at `0.1*i` for `i` in `[first, last]` every 2, with the
 *  other two coordinates at `other`. */
std::vector<mrpt::math::TPoint3Df> pointsAlongAxis(int axis, int first, int last, float other)
{
  std::vector<mrpt::math::TPoint3Df> pts;
  for (int i = first; i <= last; i += 2)
  {
    mrpt::math::TPoint3Df p(other, other, other);
    p[axis] = 0.1f * static_cast<float>(i);
    pts.push_back(p);
  }
  return pts;
}

/** For HashedVoxelPointCloud and NDT: from -4.9 to 4.9, five points per
 *  voxel `[i, i+1)`, ten voxels. Rounding toward zero merges `(-1, 1)` into a
 *  single voxel with ten points, leaving nine voxels. */
std::vector<mrpt::math::TPoint3Df> voxelTestPoints(int axis)
{
  return pointsAlongAxis(axis, -49, 49, 0.5f);
}

/** For SparseTreesPointCloud, whose grids are centered at integers: from -4.4
 *  to 4.4, five points per grid `[i-1/2, i+1/2)`, nine grids. Rounding toward
 *  zero merges `(-3/2, 1/2)` into a single grid, leaving eight. */
std::vector<mrpt::math::TPoint3Df> gridTestPoints(int axis)
{
  return pointsAlongAxis(axis, -44, 44, 0.0f);
}

/// One voxel (or grid): the minimum corner its index maps to, and its points.
struct Cell
{
  mrpt::math::TPoint3Df              corner;
  std::vector<mrpt::math::TPoint3Df> points;
};

template <class MAP>
std::vector<Cell> cellsOf(const MAP& map)
{
  std::vector<Cell> cells;
  map.visitAllVoxels(
      [&](const auto& idx, const auto& voxel)
      {
        Cell c;
        c.corner       = map.globalIdxToCoord(idx);
        const auto pts = voxel.points();
        for (size_t i = 0; i < pts.size(); i++)
        {
          c.points.push_back(pts[i]);
        }
        cells.push_back(c);
      });
  return cells;
}

std::vector<Cell> cellsOf(const mola::SparseTreesPointCloud& map)
{
  std::vector<Cell> cells;
  map.visitAllGrids(
      [&](const auto& idx, const auto& grid)
      {
        Cell c;
        c.corner = map.outerIdxToCoord(idx);
        for (size_t i = 0; i < grid.points().size(); i++)
        {
          mrpt::math::TPoint3Df p;
          grid.points().getPointFast(i, p.x, p.y, p.z);
          c.points.push_back(p);
        }
        cells.push_back(c);
      });
  return cells;
}

/// All cells hold the same number of points, each inside the box of its cell.
void checkCells(const std::vector<Cell>& cells, size_t expectedCells)
{
  ASSERT_EQUAL_(cells.size(), expectedCells);
  for (const auto& c : cells)
  {
    ASSERT_EQUAL_(c.points.size(), 5UL);
    for (const auto& p : c.points)
    {
      for (int k = 0; k < 3; k++)
      {
        ASSERT_GE_(p[k], c.corner[k]);
        ASSERT_LT_(p[k], c.corner[k] + CELL);
      }
    }
  }
}

template <class MAP>
void roundTrip(const MAP& in, MAP& out)
{
  mrpt::io::CMemoryStream buf;
  auto                    arch = mrpt::serialization::archiveFrom(buf);
  arch << in;
  buf.Seek(0);
  arch >> out;
}

/// Index of the voxel containing `x`, as computed before rounding down.
int32_t truncated(float x) { return static_cast<int32_t>(x / CELL); }

void test_hashed_voxels()
{
  for (int axis = 0; axis < 3; axis++)
  {
    const auto pts = voxelTestPoints(axis);

    mola::HashedVoxelPointCloud map(CELL);
    for (const auto& p : pts)
    {
      map.insertPoint(p);
    }
    checkCells(cellsOf(map), 10);

    const auto bbox = map.boundingBox();
    for (const auto& p : pts)
    {
      ASSERT_(bbox.containsPoint(p));
    }

    // A map saved with the former indices must load with the current ones:
    mola::HashedVoxelPointCloud legacy(CELL);
    for (const auto& p : pts)
    {
      legacy.voxelByGlobalIdxs({truncated(p.x), truncated(p.y), truncated(p.z)}, true)
          ->insertPoint(p);
    }
    mola::HashedVoxelPointCloud loaded;
    roundTrip(legacy, loaded);
    checkCells(cellsOf(loaded), 10);
  }
  std::cout << "[voxel_indexing] HashedVoxelPointCloud: OK\n";
}

void test_ndt()
{
  const mrpt::math::TPoint3Df sensor(0, 0, 0);

  for (int axis = 0; axis < 3; axis++)
  {
    const auto pts = voxelTestPoints(axis);

    mola::NDT map(CELL);
    for (const auto& p : pts)
    {
      map.insertPoint(p, sensor);
    }
    checkCells(cellsOf(map), 10);

    const auto bbox = map.boundingBox();
    for (const auto& p : pts)
    {
      ASSERT_(bbox.containsPoint(p));
    }

    mola::NDT legacy(CELL);
    for (const auto& p : pts)
    {
      legacy.voxelByGlobalIdxs({truncated(p.x), truncated(p.y), truncated(p.z)}, true)
          ->insertPoint(p, sensor);
    }
    mola::NDT loaded;
    roundTrip(legacy, loaded);
    checkCells(cellsOf(loaded), 10);
  }
  std::cout << "[voxel_indexing] NDT: OK\n";
}

void test_sparse_trees()
{
  for (int axis = 0; axis < 3; axis++)
  {
    const auto pts = gridTestPoints(axis);

    mola::SparseTreesPointCloud map(CELL);
    for (const auto& p : pts)
    {
      map.insertPoint(p);
    }
    checkCells(cellsOf(map), 9);

    const auto                  half = 0.5f * CELL;
    mola::SparseTreesPointCloud legacy(CELL);
    for (const auto& p : pts)
    {
      legacy
          .gridByOuterIdxs(
              {truncated(p.x + half), truncated(p.y + half), truncated(p.z + half)}, true)
          ->insertPoint(p);
    }
    mola::SparseTreesPointCloud loaded;
    roundTrip(legacy, loaded);
    checkCells(cellsOf(loaded), 9);
  }
  std::cout << "[voxel_indexing] SparseTreesPointCloud: OK\n";
}

}  // namespace

int main([[maybe_unused]] int argc, [[maybe_unused]] char** argv)
{
  try
  {
    test_hashed_voxels();
    test_ndt();
    test_sparse_trees();
  }
  catch (std::exception& e)
  {
    std::cerr << e.what() << "\n";
    return 1;
  }
  return 0;
}
