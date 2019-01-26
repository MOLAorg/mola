/* -------------------------------------------------------------------------
 *   A Modular Optimization framework for Localization and mApping  (MOLA)
 * Copyright (C) 2018-2019 Jose Luis Blanco, University of Almeria
 * See LICENSE for license information.
 * ------------------------------------------------------------------------- */
/**
 * @file   lock_helpers.h
 * @brief  std mutexes helper templates
 * @author Jose Luis Blanco Claraco
 * @date   Jan 26, 2019
 */
#pragma once

namespace mola
{
template <class T>
class LockHelper
{
   public:
    LockHelper(T* l) : l_{l} { l_->lock(); }
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
    T* l_{nullptr};
};

template <class T>
LockHelper<T> lockHelper(T& t)
{
    return LockHelper<T>(&t);
}
}  // namespace mola
