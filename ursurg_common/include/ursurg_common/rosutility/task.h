#pragma once

#include <ros/callback_queue_interface.h>

#include <future>

template<typename F, typename R = std::result_of_t<F()>>
class ROSTask : public ros::CallbackInterface
{
    F f_;
    std::promise<R> promise_;

public:
    ROSTask() = default;
    explicit ROSTask(const F& f) : f_(f) {};
    explicit ROSTask(F&& f) : f_(std::move(f)) {};

    std::future<R> get_future()
    {
        return promise_.get_future();
    }

    virtual CallResult call() override
    {
        do_call(promise_);
        return Success;
    }

private:
    template<typename Res>
    void do_call(std::promise<Res>& p)
    {
        p.set_value(f_());
    }

    // overload to handle std::promise<void>
    void do_call(std::promise<void>& p)
    {
        f_();
        p.set_value();
    }
};

template<typename F>
auto make_ros_task(F&& f)
{
    return ROSTask<std::decay_t<F>>(std::forward<F>(f));
}

// ROS callback queues take boost::shared_ptr<ros::CallbackInterface>'s
template<typename F>
auto make_ros_task_shared(F&& f)
{
    return boost::make_shared<ROSTask<std::decay_t<F>>>(std::forward<F>(f));
}
