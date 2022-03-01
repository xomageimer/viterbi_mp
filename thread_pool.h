#ifndef VITERBI_THREAD_POOL_H
#define VITERBI_THREAD_POOL_H

#include <mutex>
#include <condition_variable>
#include <thread>
#include <atomic>
#include <future>

#include <vector>
#include <queue>
#include <functional>
#include <optional>
#include <memory>

class ThreadPool {
public:
    explicit ThreadPool(size_t ThreadCount);
    ~ThreadPool();
private:
    std::mutex mut;
    std::condition_variable cv;

    std::vector<std::thread> Workers;
    std::vector<bool> IsWork;

    std::queue<std::function<void()>> Tasks;

    std::atomic_bool Stop = false;

public:
    template<typename F, typename... Args>
    auto AddTask(F &&f, Args &&... args) -> std::future<void> {
        auto TaskPtr = std::make_shared<std::packaged_task<void()>>(std::bind(std::forward<F>(f), std::forward<Args>(args)...));

        auto ReturnFuture = TaskPtr->get_future();

        {
            std::lock_guard lk(mut);
            Tasks.emplace([TaskPtr]{(*TaskPtr)();});
        }

        cv.notify_one();

        return ReturnFuture;
    }
};


#endif //VITERBI_THREAD_POOL_H
