/* -------------------------------------------------------------------------
 *   A Modular Optimization framework for Localization and mApping  (MOLA)
 * Copyright (C) 2018-2024 Jose Luis Blanco, University of Almeria
 * See LICENSE for license information.
 * ------------------------------------------------------------------------- */
/**
 * @file   FactorBase.h
 * @brief  Base class for all "factors" in the world model
 * @author Jose Luis Blanco Claraco
 * @date   Jan 08, 2019
 */
#pragma once

#include <mola_kernel/id.h>
#include <mrpt/serialization/CSerializable.h>

namespace mola
{
/** Defines a type of robust (m-Estimator) factor
 * \ingroup mola_kernel_grp
 */
enum class Robust : uint8_t
{
    REGULAR_L2 = 0 /**!< Regular L2 least-squares (no robust kernel) */,
    HUBER /**!< Huber kernel */,
    CAUCHY,  //
    TUKEY,  //
    WELSH,  //
    GEMANMCCLURE  //
};

/** Base class for all "factors" in the world model
 *
 * \ingroup mola_kernel_grp
 */
class FactorBase : public mrpt::serialization::CSerializable
{
    DEFINE_VIRTUAL_SERIALIZABLE(FactorBase)

   public:
    FactorBase() = default;
    virtual ~FactorBase();

    /** Number of entities involved in this factor: 1 for unary factors, 2 for
     * binary, etc. */
    virtual std::size_t edge_count() const = 0;
    /** Access entity indices involved in this factor */
    virtual mola::id_t edge_indices(const std::size_t i) const = 0;

    /** The unique ID of this factor in the world model.
     * Stored here for convenience, notice that it is redundant since entities
     * are already stored in the WorldModel indexed by ID.
     */
    mola::fid_t my_id_{mola::INVALID_FID};

    /** Type of robust error function to use
     */
    mola::Robust robust_type_{mola::Robust::REGULAR_L2};

    /** Parameter for the robust error function, if so defined in  robust_type_
     */
    double robust_param_{1.0};

   protected:
    // Derived classes mus call these methods to serialize the common data in
    // this base class:
    void baseSerializeTo(mrpt::serialization::CArchive& out) const;
    void baseSerializeFrom(mrpt::serialization::CArchive& in);
};

}  // namespace mola
