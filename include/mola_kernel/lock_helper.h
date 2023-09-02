/* -------------------------------------------------------------------------
 *   A Modular Optimization framework for Localization and mApping  (MOLA)
 * Copyright (C) 2018-2023 Jose Luis Blanco, University of Almeria
 * See LICENSE for license information.
 * ------------------------------------------------------------------------- */
/**
 * @file   lock_helpers.h
 * @brief  std mutexes helper templates
 * @author Jose Luis Blanco Claraco
 * @date   Jan 26, 2019
 */
#pragma once

#include <type_traits>

namespace mola
{
template <class T>
class LockHelper
{
    using Tnc = std::remove_const_t<T>;

   public:
    LockHelper(const Tnc* l) : l_{const_cast<Tnc*>(l)} { l_->lock(); }
    ~LockHelper()
    {
        if (l_) l_->unlock();
    }

    LockHelper(const LockHelper& o) = delete;
    LockHelper& operator=(const LockHelper& o) = delete;

    LockHelper(LockHelper&& o) : l_{o.l} { o.l = nullptr; }
    LockHelper& operator=(LockHelper&& o)
    {
        l_  = o.l;
        o.l = nullptr;
        return *this;
    }

   private:
    Tnc* l_{nullptr};
};

template <class T>
LockHelper<T> lockHelper(T& t)
{
    return LockHelper<T>(&t);
}
}  // namespace mola
