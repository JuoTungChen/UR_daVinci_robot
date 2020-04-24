#ifndef HDPLUSPLUS_H
#define HDPLUSPLUS_H

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

public:
    HHD handle;
};

class Scheduler
{
public:
    using callback_t = std::function<HDCallbackCode()>;

    Scheduler(unsigned rate);
    ~Scheduler();

    void start();

    template<typename F>
    HDSchedulerHandle schedule_asynchronous(F&& callback,
                                            HDushort priority = HD_DEFAULT_SCHEDULER_PRIORITY)
    {
        // Pass pointer to callback as "userdata" and call it from an
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
    void schedule_synchronous(F callback,
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
    int rate;

private:
    std::list<callback_t> callbacks_;
    std::list<HDSchedulerHandle> handles_;
};

class Frame
{
public:
    explicit Frame(HHD handle);
    ~Frame();

private:
    HHD handle_;
};

} // namespace hd

#endif // HDPLUSPLUS_H
