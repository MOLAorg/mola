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
 * @file   DiagnosticsProvider.h
 * @brief  Virtual interface for modules providing structured REP-107 diagnostics
 * @author Jose Luis Blanco Claraco
 * @date   Apr 19, 2026
 */
#pragma once

#include <cstdint>
#include <string>
#include <vector>

namespace mola
{
/** \addtogroup mola_kernel_interfaces_grp
 * @{ */

/// Severity levels, matching diagnostic_msgs::msg::DiagnosticStatus (REP-107)
enum class DiagnosticLevel : uint8_t
{
  OK    = 0,
  WARN  = 1,
  ERROR = 2,
  STALE = 3
};

/// A single key-value diagnostic entry
struct DiagnosticKeyValue
{
  std::string key;
  std::string value;
};

/// A structured diagnostic status message (REP-107 compatible)
struct DiagnosticStatusMsg
{
  DiagnosticLevel                 level = DiagnosticLevel::OK;
  std::string                     name;  ///< e.g. "ICP Quality"
  std::string                     message;  ///< Human-readable summary
  std::string                     hardware_id;  ///< e.g. "mola_lo"
  std::vector<DiagnosticKeyValue> values;  ///< Structured k-v pairs
};

/** Virtual interface for MOLA modules that provide structured diagnostics.
 *
 * Modules implementing this interface will have their diagnostics
 * automatically collected and published on the standard ROS 2
 * /diagnostics topic by BridgeROS2.
 *
 * The `hardware_id` field will be auto-filled with the module instance
 * name if left empty.
 *
 * \ingroup mola_kernel_interfaces_grp
 */
class DiagnosticsProvider
{
 public:
  virtual ~DiagnosticsProvider() = default;

  /** Populate the diagnostic status. Called at ~1 Hz by the diagnostics
   *  collection loop. Implementations should append one or more
   *  DiagnosticStatusMsg entries to `status`.
   */
  virtual void getDiagnostics(std::vector<DiagnosticStatusMsg>& status) = 0;
};

/** @} */

}  // namespace mola
