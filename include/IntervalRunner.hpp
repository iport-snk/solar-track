#pragma once
#include <thread>
#include <atomic>
#include <chrono>
#include <functional>

class IntervalHandle {
public:
    IntervalHandle(std::function<void()> fn, int ms) : running(true), worker([this, fn, ms] {
        while (running.load()) {
            auto start = std::chrono::steady_clock::now();
            fn();
            auto end = std::chrono::steady_clock::now();
            std::this_thread::sleep_for( std::chrono::milliseconds(ms) - (end - start));
        }
    }) {}

    void stop() {
        running = false;
        if (worker.joinable()) worker.join();
    }

    ~IntervalHandle() {
        stop();
    }

private:
    std::atomic<bool> running;
    std::thread worker;
};

inline std::unique_ptr<IntervalHandle> setInterval(std::function<void()> fn, int ms) {
    return std::make_unique<IntervalHandle>(std::move(fn), ms);
}