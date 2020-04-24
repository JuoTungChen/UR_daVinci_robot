#pragma once

#include <HD/hd.h>

#include <functional>
#include <list>

namespace hd {

void error_check();

class Device
{
public:
    Device(const std::string& name);
    ~Device();

    // Disallow copies
    Device(const Device&) = delete;
    Device& operator=(const Device&) = delete;

    Device(Device&& other);
    Device& operator=(Device&& other);

    void set_force_enabled(bool enable);

public:
    HHD handle;
    std::string name;
};

class Scheduler
{
    using callback_t = std::function<HDCallbackCode()>;

public:
    Scheduler();
    ~Scheduler();

    void set_rate(unsigned scheduler_rate_hz);
    void start();
    void stop();

    template<typename F>
    HDSchedulerHandle schedule_asynchronous(F&& callback,
                                            HDushort priority = HD_DEFAULT_SCHEDULER_PRIORITY)
    {
        // Pass pointer to callback as "user data" and call it from an
        // intermediate captureless lambda
        callbacks_.push_back(std::move(callback));
        handles_.push_back(
            hdScheduleAsynchronous(
                [](void* f) -> HDCallbackCode {
                    return (*static_cast<callback_t*>(f))();
                },
                &callbacks_.back(),
                priority));
        error_check();
        return handles_.back();
    }

    template<typename F>
    void schedule_synchronous(F&& callback,
                              HDushort priority = HD_DEFAULT_SCHEDULER_PRIORITY)
    {
        hdScheduleSynchronous(
            [](void* f) -> HDCallbackCode {
                return (*static_cast<decltype(callback)*>(f))();
            },
            &callback,
            priority);
        error_check();
    }

public:
    unsigned rate_hz;

private:
    std::list<callback_t> callbacks_;
    std::list<HDSchedulerHandle> handles_;
};

class ScopedFrame
{
public:
    explicit ScopedFrame(HHD handle);
    ~ScopedFrame();

private:
    HHD handle_;
};

} // namespace hd
