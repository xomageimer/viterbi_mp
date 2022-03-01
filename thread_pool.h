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

struct ThreadPool {
public:
    explicit ThreadPool(size_t ThreadCount);
    ~ThreadPool();

    size_t WorkersCount();
    template <typename F, typename... Args>
    auto AddTask(F&& f, Args&&... args) -> std::future<void> {
        auto TaskPtr = std::make_shared<std::packaged_task<void()>>([&]{
            f(args...);
        });
        auto ReturnFuture = TaskPtr->get_future();
        {
            std::lock_guard lk (mut_);
            tasks_.emplace([TaskPtr]{(*TaskPtr)();});
        }
        cv_.notify_one();

        return ReturnFuture;
    }
private:
    struct worker {
        explicit worker(std::function<void()> f) : thread(std::move(f)) {}

        std::thread thread;
        bool is_busy = false;
    };
    std::vector<worker> workers_;

    std::mutex mut_;
    std::condition_variable cv_;

    std::queue<std::function<void()>> tasks_;
};


#endif //VITERBI_THREAD_POOL_H
