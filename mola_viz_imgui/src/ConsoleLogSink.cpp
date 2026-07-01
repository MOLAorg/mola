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
 * @file   ConsoleLogSink.cpp
 * @brief  Thread-safe rolling store of captured mrpt-logger output.
 * @author Jose Luis Blanco Claraco
 * @date   2026
 */

#include <mola_viz_imgui/MolaVizImGuiCore.h>

using namespace mola;

void ConsoleLogSink::push(const ConsoleLogEntry& e)
{
  std::lock_guard<std::mutex> lk(mtx_);
  entries_.push_back(e);
  sources_.insert(e.source);
  const size_t cap = max_entries.load();
  while (entries_.size() > cap)
  {
    entries_.pop_front();
  }
}

std::deque<ConsoleLogEntry> ConsoleLogSink::snapshot() const
{
  std::lock_guard<std::mutex> lk(mtx_);
  return entries_;
}

void ConsoleLogSink::clear()
{
  std::lock_guard<std::mutex> lk(mtx_);
  entries_.clear();
}

void ConsoleLogSink::note_source(const std::string& s)
{
  std::lock_guard<std::mutex> lk(mtx_);
  sources_.insert(s);
}

std::vector<std::string> ConsoleLogSink::sources() const
{
  std::lock_guard<std::mutex> lk(mtx_);
  return {sources_.begin(), sources_.end()};
}
