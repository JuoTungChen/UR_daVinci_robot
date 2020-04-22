#pragma once

#include <mutex>

template <typename T>
class synchronized
{
public:
    template <typename... Args>
    explicit synchronized(Args&&... args)
        : value_(std::forward<Args>(args)...)
    {
    }

    synchronized(const synchronized&) = delete;
    synchronized& operator=(const synchronized&) = delete;

    operator T() const
    {
        std::lock_guard<std::mutex> lock(mutex_);
        return value_;
    }

    template <typename F>
    auto withLock(F&& f)
    {
        std::lock_guard<std::mutex> lock(mutex_);
        std::forward<F>(f)(value_);
    }

private:
    T value_;
    mutable std::mutex mutex_;
};
