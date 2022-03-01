#include "thread_pool.h"

ThreadPool::ThreadPool(size_t ThreadCount) {
    Workers.reserve(ThreadCount);
    IsWork.resize(ThreadCount);

    for (size_t i = 0; i < ThreadCount; i++)
        Workers.emplace_back(
                [this, i] {
                    while (true) {
                        std::function<void()> CurrentTask;

                        {
                            std::unique_lock lk(mut);
                            cv.wait(lk, [this]{
                                return !Tasks.empty() || Stop;
                            });

                            IsWork[i] = true;

                            if (Stop && Tasks.empty())
                                return;

                            CurrentTask = std::move(Tasks.front());
                            Tasks.pop();
                        }

                        CurrentTask();
                        IsWork[i] = false;
                    }
                });

}

ThreadPool::~ThreadPool() {
    {
        std::lock_guard lk(mut);
        Stop = true;
    }

    cv.notify_all();

    for (auto & Thread : Workers)
        Thread.join();
}

void ThreadPool::Reset() {
    std::lock_guard lk(mut);
    while (!Tasks.empty())
        Tasks.pop();
}

size_t ThreadPool::WorkersCount() {
    size_t WorkersCount = 0;
    {
        std::lock_guard lk(mut);
        for (auto cur_state : IsWork)
            WorkersCount += cur_state;
    }

    return WorkersCount;
}
