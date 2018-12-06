/* -------------------------------------------------------------------------
 *   A Modular Optimization framework for Localization and mApping  (MOLA)
 * Copyright (C) 2018-2019 Jose Luis Blanco, University of Almeria
 * See LICENSE for license information.
 * ------------------------------------------------------------------------- */
/**
 * @file   WorkerThreadsPool.h
 * @brief  Simple thread pool
 * @author Jose Luis Blanco Claraco
 * @date   Dec 6, 2018
 */

#include <mola-kernel/WorkerThreadsPool.h>

using namespace mola;

void WorkerThreadsPool::clear()
{
    {
        std::unique_lock<std::mutex> lock(queue_mutex_);
        do_stop_ = true;
    }
    condition_.notify_all();
    for (auto& t : threads_)
        if (t.joinable()) t.join();
    threads_.clear();
}

void WorkerThreadsPool::resize(std::size_t num_threads)
{
    for (std::size_t i = 0; i < num_threads; ++i)
        threads_.emplace_back([this] {
            for (;;)
            {
                std::function<void()> task;
                {
                    std::unique_lock<std::mutex> lock(queue_mutex_);
                    condition_.wait(
                        lock, [this] { return do_stop_ || !tasks_.empty(); });
                    if (do_stop_ && tasks_.empty()) return;
                    task = std::move(tasks_.front());
                    tasks_.pop();
                }
                // Execute:
                task();
            }
        });
}
