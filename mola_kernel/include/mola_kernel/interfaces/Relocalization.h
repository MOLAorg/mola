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
 * @file   Relocalization.h
 * @brief  Virtual interface for relocalization offered by MOLA modules
 * @author Jose Luis Blanco Claraco
 * @date   Jul 29, 2024
 */
#pragma once

#include <mrpt/poses/CPose3D.h>
#include <mrpt/poses/CPose3DPDFGaussian.h>

namespace mola
{
/** Virtual interface for relocalization offered by MOLA modules
 * \ingroup mola_kernel_interfaces_grp */
class Relocalization
{
 public:
  Relocalization() = default;

  /** @name Virtual interface of Relocalization
   *{ */

  /** Re-localize near this pose, including uncertainty.
   *  \param[in] pose The pose, in the local map frame.
   *  There is no return value from this method.
   */
  virtual void relocalize_near_pose_pdf(const mrpt::poses::CPose3DPDFGaussian& p) = 0;

  /** Re-localize with the next incoming GNSS message.
   *  There is no return value from this method.
   */
  virtual void relocalize_from_gnss() = 0;

  /** @} */
};

}  // namespace mola
