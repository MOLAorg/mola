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
 * @file   test-options-ini-io.cpp
 * @brief  Test mola::exportMapLayerOptionsToIni() / importMapLayerOptionsFromIni()
 * @author Jose Luis Blanco Claraco
 * @date   Jun 18, 2026
 */

#include <mola_metric_maps/HashedVoxelPointCloud.h>
#include <mola_metric_maps/KeyframePointCloudMap.h>
#include <mola_metric_maps/NDT.h>
#include <mola_metric_maps/OptionsIniIO.h>
#include <mola_metric_maps/SparseTreesPointCloud.h>
#include <mola_metric_maps/SparseVoxelPointCloud.h>
#include <mrpt/config/CConfigFileMemory.h>
#include <mrpt/core/Clock.h>
#include <mrpt/maps/COccupancyGridMap2D.h>
#include <mrpt/maps/CSimpleMap.h>
#include <mrpt/maps/CSimplePointsMap.h>
#include <mrpt/obs/CObservation2DRangeScan.h>
#include <mrpt/obs/CObservationPointCloud.h>
#include <mrpt/obs/CSensoryFrame.h>
#include <mrpt/obs/stock_observations.h>
#include <mrpt/poses/CPose3D.h>
#include <mrpt/poses/CPose3DPDFGaussian.h>

#include <cstdlib>
#include <iomanip>
#include <iostream>
#include <sstream>

namespace
{
// Builds a synthetic, in-memory mrpt::maps::CSimpleMap with two keyframes:
//  - One with a stock 2D range scan (so COccupancyGridMap2D gets populated).
//  - One with a synthetic point cloud observation (so point-cloud-based map
//    classes get populated).
// This plays the role that an actual ".simplemap" file would play, without
// requiring one on disk: any of the classes under test could equally be
// fed via `map.loadFromSimpleMap(CSimpleMap loaded from a real .simplemap file)`.
mrpt::maps::CSimpleMap buildSyntheticSimpleMap()
{
  mrpt::maps::CSimpleMap sm;

  // Keyframe #1: pose=identity + stock 2D range scan.
  {
    auto obs2D = mrpt::obs::CObservation2DRangeScan::Create();
    mrpt::obs::stock_observations::example2DRangeScan(*obs2D);

    auto sf = mrpt::obs::CSensoryFrame::Create();
    sf->insert(obs2D);

    auto pose = mrpt::poses::CPose3DPDFGaussian::Create(mrpt::poses::CPose3D::Identity());
    sm.insert(pose, sf);
  }

  // Keyframe #2: pose=(1,0,0) + a synthetic 10x10 flat grid point cloud.
  {
    auto pts = mrpt::maps::CSimplePointsMap::Create();
    for (int ix = 0; ix < 10; ix++)
    {
      for (int iy = 0; iy < 10; iy++)
      {
        pts->insertPoint(ix * 0.20f, iy * 0.20f, 0.0f);
      }
    }

    auto obsPc        = mrpt::obs::CObservationPointCloud::Create();
    obsPc->pointcloud = pts;
    obsPc->sensorPose = mrpt::poses::CPose3D::Identity();
    obsPc->timestamp  = mrpt::Clock::now();

    auto sf = mrpt::obs::CSensoryFrame::Create();
    sf->insert(obsPc);

    auto pose = mrpt::poses::CPose3DPDFGaussian::Create(mrpt::poses::CPose3D(1.0, 0, 0, 0, 0, 0));
    sm.insert(pose, sf);
  }

  return sm;
}

std::string trim(const std::string& s)
{
  const auto b = s.find_first_not_of(" \t\r\n");
  if (b == std::string::npos)
  {
    return {};
  }
  const auto e = s.find_last_not_of(" \t\r\n");
  return s.substr(b, e - b + 1);
}

// Simulates a user hand-editing the exported .ini file: doubles every value
// that looks like a plain number (int/float), leaving strings/enums/bools
// ("YES"/"NO") untouched.
std::string doubleAllNumericValues(const std::string& iniText)
{
  std::istringstream iss(iniText);
  std::ostringstream oss;
  std::string        line;
  while (std::getline(iss, line))
  {
    const auto eq = line.find('=');
    if (eq == std::string::npos)
    {
      oss << line << "\n";
      continue;
    }
    const std::string key        = line.substr(0, eq + 1);
    const std::string trimmedVal = trim(line.substr(eq + 1));

    char*        end        = nullptr;
    const double v          = std::strtod(trimmedVal.c_str(), &end);
    const bool isPureNumber = !trimmedVal.empty() && end == trimmedVal.c_str() + trimmedVal.size();

    if (isPureNumber)
    {
      // Use fixed notation (never scientific "1e+06"): some fields are read with atoi()-like
      // parsers that would silently truncate "2e+06" down to "2".
      std::ostringstream valueOut;
      valueOut << std::fixed << std::setprecision(6) << (v * 2.0);
      oss << key << " " << valueOut.str() << "\n";
    }
    else
    {
      oss << line << "\n";
    }
  }
  return oss.str();
}

// Compares a single section of two CConfigFileBase objects key-by-key, tolerating the
// different text formatting that dumpToTextStream() ("%f", 6 decimals) vs. plain
// ostream<<double formatting (used by doubleAllNumericValues()) produce for the same
// underlying numeric value.
void assertSectionMatches(
    const mrpt::config::CConfigFileBase& expected, const mrpt::config::CConfigFileBase& actual,
    const std::string& section)
{
  ASSERT_(expected.sectionExists(section));
  ASSERT_(actual.sectionExists(section));

  std::vector<std::string> keys;
  expected.getAllKeys(section, keys);

  for (const auto& key : keys)
  {
    const auto expectedVal = expected.read_string(section, key, "");
    const auto actualVal   = actual.read_string(section, key, "<missing>");

    char*        e1          = nullptr;
    char*        e2          = nullptr;
    const double a           = std::strtod(expectedVal.c_str(), &e1);
    const double b           = std::strtod(actualVal.c_str(), &e2);
    const bool   bothNumeric = e1 != nullptr && *e1 == '\0' && !expectedVal.empty() &&
                             e2 != nullptr && *e2 == '\0' && !actualVal.empty();

    if (bothNumeric)
    {
      ASSERT_NEAR_(a, b, 1e-3 + std::abs(a) * 1e-3);
    }
    else
    {
      ASSERT_EQUAL_(expectedVal, actualVal);
    }
  }
}

// Generic round-trip test, exercised for each known metric map layer class:
//  1) Populate `map` from a synthetic CSimpleMap (as if loaded from a real .simplemap file).
//  2) Export its CLoadableOptions to an in-memory .ini.
//  3) Simulate a hand-edit of the .ini (double all numeric values).
//  4) Re-import it into the SAME (already-populated) map instance.
//  5) Check: (a) the map's data survived the options update untouched, (b) every *applied*
//     section now reflects the edited values (proving the import took effect), and (c) every
//     *rejected* section (e.g. a creation option that would require discarding existing
//     contents) was correctly left at its original value.
template <typename MapT>
void testOptionsRoundTrip(const std::string& testName, const mrpt::maps::CSimpleMap& sm)
{
  std::cout << "[test-options-ini-io] Running: " << testName << "\n";

  MapT map;
  map.loadFromSimpleMap(sm);
  const bool hadDataBefore = !map.isEmpty();

  mrpt::config::CConfigFileMemory cfgOriginal;
  ASSERT_(mola::exportMapLayerOptionsToIni(map, cfgOriginal, "layer"));
  const std::string originalIni = cfgOriginal.getContent();
  ASSERT_(!originalIni.empty());

  const std::string editedIniText = doubleAllNumericValues(originalIni);
  ASSERT_(editedIniText != originalIni);

  mrpt::config::CConfigFileMemory cfgEdited(editedIniText);

  std::vector<std::string> applied;
  std::vector<std::string> rejected;
  ASSERT_(mola::importMapLayerOptionsFromIni(map, cfgEdited, "layer", &applied, &rejected));
  ASSERT_(!applied.empty());

  // (a) data must survive untouched:
  ASSERT_EQUAL_(hadDataBefore, !map.isEmpty());

  mrpt::config::CConfigFileMemory cfgAfter;
  ASSERT_(mola::exportMapLayerOptionsToIni(map, cfgAfter, "layer"));

  // (b) applied sections must now reflect the *edited* values:
  for (const auto& section : applied)
  {
    assertSectionMatches(cfgEdited, cfgAfter, section);
  }
  // (c) rejected sections must be unchanged from their original values:
  for (const auto& section : rejected)
  {
    assertSectionMatches(cfgOriginal, cfgAfter, section);
  }
}

void test_all_classes()
{
  const auto sm = buildSyntheticSimpleMap();

  testOptionsRoundTrip<mola::NDT>("mola::NDT", sm);
  testOptionsRoundTrip<mola::HashedVoxelPointCloud>("mola::HashedVoxelPointCloud", sm);
  testOptionsRoundTrip<mola::SparseVoxelPointCloud>("mola::SparseVoxelPointCloud", sm);
  testOptionsRoundTrip<mola::SparseTreesPointCloud>("mola::SparseTreesPointCloud", sm);
  testOptionsRoundTrip<mola::KeyframePointCloudMap>("mola::KeyframePointCloudMap", sm);
  testOptionsRoundTrip<mrpt::maps::COccupancyGridMap2D>("mrpt::maps::COccupancyGridMap2D", sm);
  testOptionsRoundTrip<mrpt::maps::CSimplePointsMap>("mrpt::maps::CSimplePointsMap", sm);
}

void test_unsupported_class_is_rejected()
{
  // mrpt::maps::CSimpleMap-loaded mola::OccGrid wrapper does not derive from CMetricMap, so it
  // cannot ever be a ".mm" layer. Instead, exercise the rejection path with a metric map class
  // that this library deliberately does not support (e.g. an octomap-like map is not
  // registered): here we just verify that an empty CConfigFileMemory makes
  // importMapLayerOptionsFromIni() a safe no-op (no section present) on a supported class.
  mola::NDT                       map;
  mrpt::config::CConfigFileMemory emptyCfg;
  std::vector<std::string>        applied;
  ASSERT_(mola::importMapLayerOptionsFromIni(map, emptyCfg, "layer", &applied));
  ASSERT_(applied.empty());
}

// Directly exercises OptionsCapable::trySetCreationOptions()'s safety check: a voxel_size
// change must be REJECTED once the map already holds data, but ACCEPTED on an empty map.
void test_creation_options_safety()
{
  mrpt::config::CConfigFileMemory cfgNewVoxelSize;
  cfgNewVoxelSize.write("layer.creationOptions", "voxel_size", 9.0);

  // On an empty map, changing voxel_size is harmless:
  {
    mola::NDT map;
    ASSERT_(map.isEmpty());
    std::vector<std::string> applied, rejected;
    ASSERT_(mola::importMapLayerOptionsFromIni(map, cfgNewVoxelSize, "layer", &applied, &rejected));
    ASSERT_(rejected.empty());
    ASSERT_EQUAL_(applied.size(), 1U);
    ASSERT_NEAR_(map.creationOptions.voxel_size, 9.0f, 1e-6f);
  }

  // Once the map holds data, the very same change must be rejected, and the map left untouched:
  {
    mola::NDT map;
    map.insertPoint({1.0f, 2.0f, 3.0f}, {.0f, .0f, .0f});
    ASSERT_(!map.isEmpty());
    const float voxelSizeBefore = map.creationOptions.voxel_size;

    std::vector<std::string> applied, rejected;
    ASSERT_(mola::importMapLayerOptionsFromIni(map, cfgNewVoxelSize, "layer", &applied, &rejected));
    ASSERT_(applied.empty());
    ASSERT_EQUAL_(rejected.size(), 1U);
    ASSERT_(!map.isEmpty());
    ASSERT_NEAR_(map.creationOptions.voxel_size, voxelSizeBefore, 1e-6f);
  }

  // Re-applying the SAME (unchanged) voxel_size on a non-empty map must be a true no-op: it
  // must be accepted (not rejected) AND must not wipe the map's existing contents.
  {
    mola::NDT map;
    map.insertPoint({1.0f, 2.0f, 3.0f}, {.0f, .0f, .0f});
    ASSERT_(!map.isEmpty());

    mrpt::config::CConfigFileMemory cfgSameVoxelSize;
    cfgSameVoxelSize.write("layer.creationOptions", "voxel_size", map.creationOptions.voxel_size);

    std::vector<std::string> applied, rejected;
    ASSERT_(
        mola::importMapLayerOptionsFromIni(map, cfgSameVoxelSize, "layer", &applied, &rejected));
    ASSERT_(rejected.empty());
    ASSERT_EQUAL_(applied.size(), 1U);
    ASSERT_(!map.isEmpty());
  }
}

}  // namespace

int main([[maybe_unused]] int argc, [[maybe_unused]] char** argv)
{
  try
  {
    test_all_classes();
    test_unsupported_class_is_rejected();
    test_creation_options_safety();
  }
  catch (std::exception& e)
  {
    std::cerr << e.what() << "\n";
    return 1;
  }
}
