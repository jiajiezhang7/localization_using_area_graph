/**
 * @file thread_pool.hpp
 * @author AGLoc优化
 * @brief 高效线程池实现，用于CloudInitializer中的并行计算
 * @date 2025-03-10
 * 
 * @details 线程池用于并行处理多个位姿评估任务，
 *          优化定位过程中的双重循环，提高系统效率
 */
#pragma once
#ifndef _THREAD_POOL_HPP_
#define _THREAD_POOL_HPP_

#include <vector>
#include <queue>
#include <memory>
#include <thread>
#include <mutex>
#include <condition_variable>
#include <future>
#include <functional>
#include <stdexcept>

namespace agloc {

/**
 * @class ThreadPool
 * @brief 实现高效线程池，用于并行计算任务
 * 
 * 这个线程池实现基于C++11标准，提供简单易用的接口用于提交任务
 * 并自动管理线程的创建、任务分配和线程生命周期
 */
class ThreadPool {
public:
    /**
     * @brief 构造函数，初始化线程池
     * @param threads 线程数量，默认使用硬件支持的线程数
     */
    ThreadPool(size_t threads = std::thread::hardware_concurrency()) : stop(false) {
        // 创建指定数量的工作线程
        for(size_t i = 0; i < threads; ++i) {
            workers.emplace_back(
                [this] {
                    // 线程主循环
                    for(;;) {
                        std::function<void()> task;
                        {
                            // 等待或获取任务
                            std::unique_lock<std::mutex> lock(this->queue_mutex);
                            this->condition.wait(lock,
                                [this]{ return this->stop || !this->tasks.empty(); });
                            
                            // 如果线程池停止且任务队列为空，退出线程
                            if(this->stop && this->tasks.empty())
                                return;
                            
                            // 获取一个任务
                            task = std::move(this->tasks.front());
                            this->tasks.pop();
                        }
                        
                        // 执行任务
                        task();
                    }
                }
            );
        }
    }

    /**
     * @brief 提交任务到线程池
     * @tparam F 函数类型
     * @tparam Args 参数类型包
     * @param f 要执行的函数
     * @param args 函数参数
     * @return future对象，用于获取任务结果
     */
    template<class F, class... Args>
    auto enqueue(F&& f, Args&&... args) 
        -> std::future<typename std::result_of<F(Args...)>::type> {
        using return_type = typename std::result_of<F(Args...)>::type;

        // 创建任务包装
        auto task = std::make_shared<std::packaged_task<return_type()>>(
            std::bind(std::forward<F>(f), std::forward<Args>(args)...)
        );
            
        std::future<return_type> res = task->get_future();
        {
            // 将任务添加到队列
            std::unique_lock<std::mutex> lock(queue_mutex);

            // 如果线程池已停止，不能添加任务
            if(stop)
                throw std::runtime_error("enqueue on stopped ThreadPool");

            tasks.emplace([task](){ (*task)(); });
        }
        
        // 通知一个等待中的线程
        condition.notify_one();
        return res;
    }

    /**
     * @brief 析构函数，停止所有线程
     */
    ~ThreadPool() {
        {
            std::unique_lock<std::mutex> lock(queue_mutex);
            stop = true;
        }
        
        // 通知所有线程
        condition.notify_all();
        
        // 等待所有线程完成
        for(std::thread &worker: workers)
            worker.join();
    }

private:
    // 工作线程
    std::vector<std::thread> workers;
    // 任务队列
    std::queue<std::function<void()>> tasks;
    
    // 线程同步工具
    std::mutex queue_mutex;
    std::condition_variable condition;
    bool stop;
};

} // namespace agloc

#endif // _THREAD_POOL_HPP_
