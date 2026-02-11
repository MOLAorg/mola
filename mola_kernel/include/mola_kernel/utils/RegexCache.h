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

#include <optional>
#include <regex>
#include <string>

namespace mola
{

/** Holds a reference to a cached regex. Recompiles only if the expression changed.
 */
class RegexCache
{
 public:
  // Returns a reference to a cached regex. Recompiles only if the expression changed.
  const std::regex& get_regex(const std::string& regExpression)
  {
    if (!cachedRegex_ || regExpression != cachedExpression_)
    {
      try
      {
        cachedExpression_ = regExpression;
        cachedRegex_.emplace(cachedExpression_, std::regex::ECMAScript);
      }
      catch (const std::regex_error& e)
      {
        // Clear the cache state so we don't return a stale regex next time
        cachedRegex_.reset();

        throw std::runtime_error(mrpt::format(
            "Error compiling regex expression '%s': %s (code: %d)", regExpression.c_str(), e.what(),
            static_cast<int>(e.code())));
      }
    }
    return *cachedRegex_;
  }

 private:
  std::string               cachedExpression_;
  std::optional<std::regex> cachedRegex_;
};

}  // namespace mola