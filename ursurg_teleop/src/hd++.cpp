#include "hd++.h"

namespace hd {

void error_check()
{
    auto e = hdGetError();

    if (e.errorCode != HD_SUCCESS)
        throw std::runtime_error(hdGetErrorString(e.errorCode));
}

Device::Device(const std::string& name)
    : handle(HD_INVALID_HANDLE)
{
    handle = hdInitDevice(name.c_str());
    error_check();
}

Device::~Device()
{
    if (handle != HD_INVALID_HANDLE)
        hdDisableDevice(handle);
}

Device::Device(Device&& other)
    : handle(std::move(other.handle))
{
    other.handle = HD_INVALID_HANDLE;
}

Device& Device::operator=(Device&& other)
{
    handle = std::move(other.handle);
    other.handle = HD_INVALID_HANDLE;
    return *this;
}

Scheduler::Scheduler(unsigned rate)
    : rate(-1)
{
    if (rate != 500 && rate != 1000)
        throw std::runtime_error("Scheduler rate must be either 500 or 1000");

    hdSetSchedulerRate(rate);
    error_check();
    this->rate = rate;
}

Scheduler::~Scheduler()
{
    hdStopScheduler();

    for (auto handle : handles_)
        hdUnschedule(handle);
}

void Scheduler::start()
{
    hdStartScheduler();
    error_check();
}

Frame::Frame(HHD handle)
    : handle_(handle)
{
    hdBeginFrame(handle_);
    error_check();
}

Frame::~Frame()
{
    hdEndFrame(handle_);
    error_check();
}

} // namespace hd
