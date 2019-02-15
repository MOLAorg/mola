/* -------------------------------------------------------------------------
 *   A Modular Optimization framework for Localization and mApping  (MOLA)
 * Copyright (C) 2018-2019 Jose Luis Blanco, University of Almeria
 * See LICENSE for license information.
 * ------------------------------------------------------------------------- */
/**
 * @file   RawlogDataset.h
 * @brief  RawDataSource for datasets in MRPT rawlog format
 * @author Jose Luis Blanco Claraco
 * @date   Feb 15, 2019
 */
#pragma once

#include <mola-kernel/RawDataSourceBase.h>

namespace mola
{
namespace sensor_rawlog_dataset
{
/**
 *
 * \ingroup mola_sensor_rawlog_grp */
class RawlogDataset : public RawDataSourceBase
{
   public:
    RawlogDataset();
    ~RawlogDataset() override = default;

    // See docs in base class
    void initialize(const std::string& cfg_block) override;
    void spinOnce() override;

   private:
};

}  // namespace sensor_rawlog_dataset
}  // namespace mola
