/*               _
 _ __ ___   ___ | | __ _
| '_ ` _ \ / _ \| |/ _` | Modular Optimization framework for
| | | | | | (_) | | (_| | Localization and mApping (MOLA)
|_| |_| |_|\___/|_|\__,_| https://github.com/MOLAorg/mola

 Copyright (C) 2018-2026 Jose Luis Blanco, University of Almeria,
                         and individual contributors.
 SPDX-License-Identifier: GPL-3.0
 See LICENSE for full license information.
 Closed-source licenses available upon request, for this odometry package
 alone or in combination with the complete SLAM system.
*/

/**
 * @file   RegexCache.h
 * @brief  Cached regular expression
 * @author Jose Luis Blanco Claraco
 * @date   Jan 29, 2026
 */
#pragma once

#include <mrpt/core/exceptions.h>

#include <mutex>
#include <optional>
#include <regex>
#include <string>

namespace mola
{

/** Holds a cached regex. Recompiles only if the expression changed.
 *  Thread-safe: multiple threads may call get_regex() concurrently.
 */
class RegexCache
{
 public:
  RegexCache()  = default;
  ~RegexCache() = default;

  RegexCache(const RegexCache& other)
  {
    std::lock_guard<std::mutex> lck(other.mutex_);
    cachedExpression_ = other.cachedExpression_;
    cachedRegex_      = other.cachedRegex_;
  }

  RegexCache(RegexCache&& other) noexcept
  {
    std::lock_guard<std::mutex> lck(other.mutex_);
    cachedExpression_ = std::move(other.cachedExpression_);
    cachedRegex_      = std::move(other.cachedRegex_);
  }

  RegexCache& operator=(const RegexCache& other)
  {
    if (this != &other)
    {
      std::lock_guard<std::mutex> lckSrc(other.mutex_);
      std::lock_guard<std::mutex> lckDst(mutex_);
      cachedExpression_ = other.cachedExpression_;
      cachedRegex_      = other.cachedRegex_;
    }
    return *this;
  }

  RegexCache& operator=(RegexCache&& other) noexcept
  {
    if (this != &other)
    {
      std::lock_guard<std::mutex> lckSrc(other.mutex_);
      std::lock_guard<std::mutex> lckDst(mutex_);
      cachedExpression_ = std::move(other.cachedExpression_);
      cachedRegex_      = std::move(other.cachedRegex_);
    }
    return *this;
  }

  // Returns a copy of the cached regex. Recompiles only if the expression changed.
  std::regex get_regex(const std::string& regExpression)
  {
    std::lock_guard<std::mutex> lck(mutex_);
    if (!cachedRegex_ || regExpression != cachedExpression_)
    {
      try
      {
        cachedExpression_ = regExpression;
        cachedRegex_.emplace(cachedExpression_, std::regex::ECMAScript);
      }
      catch (const std::regex_error& e)
      {
        cachedRegex_.reset();
        throw std::runtime_error(mrpt::format(
            "Error compiling regex expression '%s': %s (code: %d)", regExpression.c_str(), e.what(),
            static_cast<int>(e.code())));
      }
    }
    return *cachedRegex_;
  }

 private:
  mutable std::mutex        mutex_;
  std::string               cachedExpression_;
  std::optional<std::regex> cachedRegex_;
};

}  // namespace mola
