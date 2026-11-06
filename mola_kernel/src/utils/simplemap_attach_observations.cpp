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
 * @file   simplemap_attach_observations.cpp
 * @brief  Adds observations from a dataset to the keyframes of a simplemap.
 * @author Jose Luis Blanco Claraco
 * @date   2026
 */

#include <mola_kernel/utils/simplemap_attach_observations.h>
#include <mrpt/core/format.h>
#include <mrpt/maps/CSimpleMap.h>
#include <mrpt/obs/CObservationComment.h>
#include <mrpt/obs/CObservationImage.h>
#include <mrpt/system/filesystem.h>

#include <algorithm>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <iterator>
#include <limits>
#include <optional>
#include <stdexcept>
#include <utility>
#include <vector>

namespace mola
{
namespace
{
struct Keyframe
{
  double                        stamp = 0;
  mrpt::obs::CSensoryFrame::Ptr sf;
  /// Sensor labels it already has: it never gets a second one.
  std::set<std::string> labels;
};

/// Keyframe time: that of its "metadata" comment, else of its first observation.
std::optional<double> keyframe_stamp(const mrpt::obs::CSensoryFrame& sf)
{
  for (const auto& o : sf)
  {
    if (o->sensorLabel == "metadata" &&
        std::dynamic_pointer_cast<const mrpt::obs::CObservationComment>(o))
    {
      return mrpt::Clock::toDouble(o->timestamp);
    }
  }
  if (sf.empty())
  {
    return {};
  }
  return mrpt::Clock::toDouble((*sf.begin())->timestamp);
}

/// Whether the frame holds sensor data, not only the "metadata" comment of a
/// non-keyframe.
bool has_sensor_data(const mrpt::obs::CSensoryFrame& sf)
{
  return std::any_of(
      sf.begin(), sf.end(),
      [](const auto& o)
      { return !std::dynamic_pointer_cast<const mrpt::obs::CObservationComment>(o); });
}

std::string lazy_load_dir_of(const std::string& simplemapFile)
{
  return mrpt::system::fileNameChangeExtension(simplemapFile, "") + "_Images";
}

bool same_contents(const std::filesystem::path& a, const std::filesystem::path& b)
{
  namespace fs = std::filesystem;
  if (fs::equivalent(a, b))
  {
    return true;
  }
  if (fs::file_size(a) != fs::file_size(b))
  {
    return false;
  }
  std::ifstream fa(a, std::ios::binary);
  std::ifstream fb(b, std::ios::binary);
  return std::equal(
      std::istreambuf_iterator<char>(fa), std::istreambuf_iterator<char>(),
      std::istreambuf_iterator<char>(fb));
}

/** Hard links (or copies) the files of `from` into `to`, with their relative
 *  paths. Existing files of `to` are never replaced: if one differs from its
 *  source, it throws before changing anything. */
void mirror_directory(const std::string& from, const std::string& to)
{
  namespace fs = std::filesystem;
  if (fs::exists(to))
  {
    for (const auto& e : fs::recursive_directory_iterator(from))
    {
      if (!e.is_regular_file())
      {
        continue;
      }
      const auto dst = fs::path(to) / e.path().lexically_relative(from);
      if (fs::exists(dst) && !same_contents(e.path(), dst))
      {
        throw std::runtime_error(
            "Output file exists with different contents: " + dst.string() +
            ". Use another output name.");
      }
    }
  }
  fs::create_directories(to);
  for (const auto& e : fs::recursive_directory_iterator(from))
  {
    if (!e.is_regular_file())
    {
      continue;
    }
    const auto dst = fs::path(to) / e.path().lexically_relative(from);
    if (fs::exists(dst))
    {
      continue;  // same contents, checked above
    }
    fs::create_directories(dst.parent_path());
    std::error_code ec;
    fs::create_hard_link(e.path(), dst, ec);
    if (ec)
    {
      fs::copy_file(e.path(), dst);
    }
  }
}

/** A copy of `img` whose image refers to `fileName` (lazy-load) instead of
 *  holding the pixels. `img` is left untouched: CImage copies are shallow. */
mrpt::obs::CObservationImage::Ptr external_image_copy(
    const mrpt::obs::CObservationImage& img, const std::string& fileName)
{
  auto copy   = std::make_shared<mrpt::obs::CObservationImage>(img);
  copy->image = mrpt::img::CImage();
  copy->image.setExternalStorage(fileName);
  return copy;
}

}  // namespace

SimplemapAttachResult simplemap_attach_observations(
    const std::string& inputFile, const std::string& outputFile,
    const OfflineDatasetSource& dataset, const SimplemapAttachOptions& options,
    const std::function<void(double)>& progress)
{
  const auto& fmt = options.image_format;
  if (fmt != "jpg" && fmt != "png" && fmt != "inline")
  {
    throw std::invalid_argument("image_format must be one of: jpg, png, inline");
  }
  if (options.max_age < 0)
  {
    throw std::invalid_argument("max_age must not be negative");
  }
  if (options.jpeg_quality < 1 || options.jpeg_quality > 100)
  {
    throw std::invalid_argument("jpeg_quality must be in [1,100]");
  }
  // Image file names come from the labels: they must not collide.
  {
    std::set<std::string> fileLabels;
    for (const auto& l : options.sensor_labels)
    {
      if (!fileLabels.insert(mrpt::system::fileNameStripInvalidChars(l)).second)
      {
        throw std::invalid_argument("Sensor labels give the same file names: rename '" + l + "'");
      }
    }
  }

  mrpt::maps::CSimpleMap sm;
  if (!sm.loadFromFile(inputFile))
  {
    throw std::runtime_error("Cannot read: " + inputFile);
  }

  // Keyframes, and the observations already in them (never added again):
  std::vector<Keyframe>                                     kfs;
  std::set<std::pair<std::string, mrpt::Clock::time_point>> existing;
  for (const auto& kf : sm)
  {
    if (!kf.sf || !has_sensor_data(*kf.sf))
    {
      continue;
    }
    std::set<std::string> labels;
    for (const auto& o : *kf.sf)
    {
      if (options.sensor_labels.count(o->sensorLabel) != 0)
      {
        existing.emplace(o->sensorLabel, o->timestamp);
        labels.insert(o->sensorLabel);
      }
    }
    if (const auto t = keyframe_stamp(*kf.sf); t)
    {
      kfs.push_back({*t, kf.sf, labels});
    }
  }
  std::sort(kfs.begin(), kfs.end(), [](const auto& a, const auto& b) { return a.stamp < b.stamp; });
  if (kfs.empty())
  {
    throw std::runtime_error("The input simplemap has no keyframes with sensor data");
  }

  SimplemapAttachResult result;
  result.keyframes = kfs.size();
  for (const auto& l : options.sensor_labels)
  {
    result.attached[l] = 0;
  }

  // External files: those of the input map, plus the new images:
  const auto outDir = lazy_load_dir_of(outputFile);
  const auto inDir  = lazy_load_dir_of(inputFile);
  if (mrpt::system::directoryExists(inDir) &&
      std::filesystem::weakly_canonical(inDir) != std::filesystem::weakly_canonical(outDir))
  {
    mirror_directory(inDir, outDir);
  }
  if (fmt != "inline")
  {
    std::filesystem::create_directories(outDir);
  }

  // Per label and keyframe index, the closest observation so far:
  struct Candidate
  {
    mrpt::obs::CObservation::Ptr obs;
    double                       dt = 0;
  };
  std::map<std::string, std::map<size_t, Candidate>> best;

  // Stores a final candidate in its keyframe (images: as an external file).
  const auto commit = [&](const std::string& label, const Candidate& c, Keyframe& kf)
  {
    auto obs = c.obs;
    if (auto img = std::dynamic_pointer_cast<mrpt::obs::CObservationImage>(obs); img)
    {
      // Images stored as files of the dataset are loaded: the map must not
      // depend on the dataset directory.
      if (img->image.isExternallyStored())
      {
        const auto file = img->image.getExternalStorageFileAbsolutePath();
        auto       copy = std::make_shared<mrpt::obs::CObservationImage>(*img);
        copy->image     = mrpt::img::CImage();
        if (!copy->image.loadFromFile(file))
        {
          throw std::runtime_error("Cannot read: " + file);
        }
        img = copy;
        obs = copy;
      }
      if (fmt != "inline" && img->image.getPixelDepth() == mrpt::img::PixelDepth::D8U)
      {
        const auto name = mrpt::format(
            "%s_%.09f.%s", mrpt::system::fileNameStripInvalidChars(label).c_str(),
            mrpt::Clock::toDouble(img->timestamp), fmt.c_str());
        const auto path = mrpt::system::pathJoin({outDir, name});
        // Never write through a hard link to a file of the input map:
        std::filesystem::remove(path);
        if (!img->image.saveToFile(path, options.jpeg_quality))
        {
          throw std::runtime_error("Cannot write: " + path);
        }
        obs = external_image_copy(*img, name);
      }
    }
    kf.sf->insert(obs);
    kf.labels.insert(label);
    result.attached[label]++;
  };

  // Commits the candidates of keyframes that can no longer get a closer one.
  const auto commitUpTo = [&](double t)
  {
    for (auto& [label, m] : best)
    {
      for (auto it = m.begin(); it != m.end();)
      {
        if (kfs.at(it->first).stamp + options.max_age < t)
        {
          commit(label, it->second, kfs.at(it->first));
          it = m.erase(it);
        }
        else
        {
          ++it;
        }
      }
    }
  };

  // With a margin: datasets are in receive order, not in stamp order.
  constexpr double STAMP_ORDER_MARGIN = 2.0;  // [s]

  const double tFirst = kfs.front().stamp - options.max_age;
  const double tLast  = kfs.back().stamp + options.max_age;
  const size_t N      = dataset.datasetSize();
  for (size_t i = 0; i < N; i++)
  {
    if (progress && i % 1000 == 0)
    {
      progress(static_cast<double>(i) / static_cast<double>(N));
    }
    const auto sf = dataset.datasetGetObservations(i);
    if (!sf)
    {
      continue;
    }
    for (const auto& o : *sf)
    {
      if (!o || options.sensor_labels.count(o->sensorLabel) == 0)
      {
        continue;
      }
      result.observations_read++;
      if (existing.count({o->sensorLabel, o->timestamp}) != 0)
      {
        continue;
      }
      const double t = mrpt::Clock::toDouble(o->timestamp);
      if (t < tFirst || t > tLast)
      {
        continue;
      }
      // Nearest keyframe in time:
      const auto it = std::lower_bound(
          kfs.begin(), kfs.end(), t, [](const Keyframe& k, double v) { return k.stamp < v; });
      auto k = static_cast<size_t>(std::distance(kfs.begin(), it));
      if (k == kfs.size() || (k > 0 && t - kfs[k - 1].stamp < kfs[k].stamp - t))
      {
        k--;
      }
      const double dt = std::abs(t - kfs[k].stamp);
      if (dt > options.max_age || kfs[k].labels.count(o->sensorLabel) != 0)
      {
        continue;  // too far, or that keyframe already has one
      }
      auto& slot = best[o->sensorLabel];
      if (const auto c = slot.find(k); c == slot.end() || dt < c->second.dt)
      {
        slot[k] = {std::const_pointer_cast<mrpt::obs::CObservation>(o), dt};
      }
      commitUpTo(t - STAMP_ORDER_MARGIN);
    }
  }
  commitUpTo(std::numeric_limits<double>::max());
  if (progress)
  {
    progress(1.0);
  }

  // Through a temporary file, since the output may be the input map:
  const std::string tmpFile = outputFile + ".tmp";
  if (!sm.saveToFile(tmpFile))
  {
    throw std::runtime_error("Cannot write: " + tmpFile);
  }
  std::filesystem::rename(tmpFile, outputFile);
  return result;
}

}  // namespace mola
