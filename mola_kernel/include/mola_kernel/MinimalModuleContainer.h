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
 * @file   MinimalModuleContainer.h
 * @brief  A simple module container for use without mola_launcher
 * @author Jose Luis Blanco Claraco
 * @date   Dec 26, 2024
 */
#pragma once

#include <mola_kernel/interfaces/ExecutableBase.h>

#include <string>
#include <vector>

namespace mola
{
/** \addtogroup mola_kernel_grp
 * @{ */

/**
 * A minimal MOLA application container, for use programatically when it is
 * not advisable to use the mola-cli application.
 *
 * This implements basic discoverability and visibility between modules.
 */
class MinimalModuleContainer
{
 public:
  MinimalModuleContainer() = default;
  ~MinimalModuleContainer();

  MinimalModuleContainer(const std::vector<mola::ExecutableBase::Ptr>& mods) : modules_(mods)
  {
    for (auto& m : modules_)
    {
      ASSERT_(m);
      installNameServer(*m);
    }
  }

  void add(const mola::ExecutableBase::Ptr& m)
  {
    ASSERT_(m);
    modules_.push_back(m);
    installNameServer(*m);
  }

  const auto& modules() const { return modules_; }

 private:
  std::vector<mola::ExecutableBase::Ptr> modules_;

  void installNameServer(mola::ExecutableBase& m);

  ExecutableBase::Ptr nameServerImpl(const std::string& name);
};

/** @} */

}  // namespace mola
