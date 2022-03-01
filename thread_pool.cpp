#include "thread_pool.h"

ThreadPool::ThreadPool(size_t ThreadCount) {
    workers_.reserve(ThreadCount);

    for (size_t i = 0; i < ThreadCount; ++i){
        workers_.emplace_back([this, i] {
                    while (true) {
                        std::function<void()> CurrentTask;

                        {
                            std::unique_lock lk(mut_);
                            cv_.wait(lk, [this] {
                                return !tasks_.empty();
                            });
                            workers_[i].is_busy = true;

                            if (tasks_.empty())
                                return;

                            CurrentTask = std::move(tasks_.front());
                            tasks_.pop();
                        }

                        CurrentTask();
                        workers_[i].is_busy = false;
                    }
                });
    }
}

ThreadPool::~ThreadPool() {
    cv_.notify_all();

    for (auto & worker : workers_){
        worker.thread.join();
    }
}

size_t ThreadPool::WorkersCount() {
    return workers_.size();
}