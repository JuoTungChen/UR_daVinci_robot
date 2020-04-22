#pragma once

#include <mutex>

template <typename T>
class synchronized
{
public:
    synchronized() = default;

    explicit synchronized(const T& value)
        : value_(value)
    {
    }

    synchronized& operator=(const T& value)
    {
        std::lock_guard<std::mutex> lock(mutex_);
        value_ = value;
        return *this;
    }

    // No move/copy from other synchronized types
    synchronized& operator=(const synchronized&) = delete;
    synchronized& operator=(synchronized&&) = delete;
    synchronized(const synchronized&) = delete;
    synchronized(synchronized&&) = delete;

    operator T() const
    {
        std::lock_guard<std::mutex> lock(mutex_);
        return value_;
    }

    template <typename F>
    auto withLock(F&& f)
    {
        std::lock_guard<std::mutex> lock(mutex_);
        return std::forward<F>(f)(value_);
    }

private:
    T value_;
    mutable std::mutex mutex_;
};
