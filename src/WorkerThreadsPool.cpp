/* -------------------------------------------------------------------------
 *   A Modular Optimization framework for Localization and mApping  (MOLA)
 * Copyright (C) 2018-2019 Jose Luis Blanco, University of Almeria
 * See LICENSE for license information.
 * ------------------------------------------------------------------------- */
/**
 * @file   WorkerThreadsPool.cpp
 * @brief  Simple thread pool
 * @author Jose Luis Blanco Claraco
 * @date   Dec 6, 2018
 */

#include <mola-kernel/WorkerThreadsPool.h>
#include <mrpt/core/exceptions.h>
#include <iostream>

using namespace mola;

MRPT_TODO("Remove file when mrpt v2.0.5 is released");

void WorkerThreadsPool::clear()
{
    {
        std::unique_lock<std::mutex> lock(queue_mutex_);
        do_stop_ = true;
    }
    condition_.notify_all();

    if (!tasks_.empty())
        std::cerr << "[threadPool] Warning: clear() called (probably from a "
                     "dtor) while having "
                  << tasks_.size() << " pending tasks. Aborting them.\n";

    for (auto& t : threads_)
        if (t.joinable()) t.join();
    threads_.clear();
}

std::size_t WorkerThreadsPool::pendingTasks() const noexcept
{
    return tasks_.size();
}

void WorkerThreadsPool::resize(std::size_t num_threads)
{
    for (std::size_t i = 0; i < num_threads; ++i)
        threads_.emplace_back([this] {
            for (;;)
            {
                try
                {
                    std::function<void()> task;
                    {
                        std::unique_lock<std::mutex> lock(queue_mutex_);
                        condition_.wait(lock, [this] {
                            return do_stop_ || !tasks_.empty();
                        });
                        if (do_stop_) return;
                        task = std::move(tasks_.front());
                        tasks_.pop();
                    }
                    // Execute:
                    task();
                }
                catch (std::exception& e)
                {
                    std::cerr << "[thread-pool] Exception:\n"
                              << mrpt::exception_to_str(e) << "\n";
                }
            }
        });
}
