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
#pragma once

#include <atomic>
#include <condition_variable>
#include <functional>
#include <future>
#include <mutex>
#include <queue>
#include <thread>
#include <vector>

namespace mola
{
/**
 * @brief A simple thread pool
 *
 * \note Partly based on: https://github.com/progschj/ThreadPool (ZLib license)
 */
class WorkerThreadsPool
{
   public:
    WorkerThreadsPool() = default;
    WorkerThreadsPool(std::size_t num_threads) { resize(num_threads); }
    ~WorkerThreadsPool() { clear(); }
    void resize(std::size_t num_threads);
    /** Stops and deletes all worker threads */
    void clear();

    /** Enqueue one new working item, to be executed by threads when any is
     * available. */
    template <class F, class... Args>
    auto enqueue(F&& f, Args&&... args)
        -> std::future<typename std::result_of<F(Args...)>::type>;

   private:
    std::vector<std::thread>          threads_;
    std::atomic_bool                  do_stop_{false};
    std::mutex                        queue_mutex_;
    std::condition_variable           condition_;
    std::queue<std::function<void()>> tasks_;
};

template <class F, class... Args>
auto WorkerThreadsPool::enqueue(F&& f, Args&&... args)
    -> std::future<typename std::result_of<F(Args...)>::type>
{
    using return_type = typename std::result_of<F(Args...)>::type;

    auto task = std::make_shared<std::packaged_task<return_type()>>(
        std::bind(std::forward<F>(f), std::forward<Args>(args)...));

    std::future<return_type> res = task->get_future();
    {
        std::unique_lock<std::mutex> lock(queue_mutex_);
        // don't allow enqueueing after stopping the pool
        if (do_stop_) throw std::runtime_error("enqueue on stopped ThreadPool");
        tasks_.emplace([task]() { (*task)(); });
    }
    condition_.notify_one();
    return res;
}

}  // namespace mola
