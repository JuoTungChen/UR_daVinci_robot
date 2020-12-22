#pragma once

#include <atomic>
#include <condition_variable>
#include <exception>
#include <mutex>
#include <queue>

class blocking_queue_abort : public std::exception
{
public:
    virtual char const* what() const noexcept override
    {
        return "blocking_queue: aborted";
    }
};

template<typename T>
class blocking_queue
{
    std::atomic<bool> aborted_;
    std::condition_variable cond_;
    std::mutex mutex_;
    std::queue<T> queue_;

public:
    blocking_queue()
        : aborted_(false)
    {}

    blocking_queue(const blocking_queue&) = delete;
    blocking_queue& operator=(const blocking_queue&) = delete;

    T pop()
    {
        std::unique_lock<std::mutex> lock(mutex_);
        cond_.wait(lock, [this]() { return !queue_.empty() || aborted_; });

        if (aborted_)
            throw blocking_queue_abort();

        T val = std::move(queue_.front());
        queue_.pop();
        return val;
    }

    void push(const T& x)
    {
        {
            std::unique_lock<std::mutex> lock(mutex_);
            queue_.push(x);
        }

        cond_.notify_one();
    }

    void push(T&& x)
    {
        {
            std::unique_lock<std::mutex> lock(mutex_);
            queue_.push(std::move(x));
        }

        cond_.notify_one();
    }

    void abort()
    {
        aborted_ = true;
        cond_.notify_all();
    }
};
