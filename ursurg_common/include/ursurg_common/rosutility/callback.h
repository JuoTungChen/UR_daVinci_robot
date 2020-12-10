#pragma once

#include <ros/callback_queue.h>

template<typename F>
class ROSCallbackWrapper : public ros::CallbackInterface
{
    F f_;

public:
    ROSCallbackWrapper(const F& f) : f_(f) {};
    ROSCallbackWrapper(F&& f) : f_(std::move(f)) {};

    virtual CallResult call() override
    {
        f_();
        return Success;
    }
};

template<typename F>
boost::shared_ptr<ROSCallbackWrapper<F>> make_shared_ros_callback(F&& f)
{
    return boost::make_shared<ROSCallbackWrapper<F>>(std::forward<F>(f));
}
