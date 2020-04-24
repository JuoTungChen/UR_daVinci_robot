#include "hd++.h"

#include <iomanip>
#include <sstream>

namespace hd {

void error_check()
{
    auto e = hdGetError();

    if (e.errorCode != HD_SUCCESS) {
        std::stringstream ss;
        ss << hdGetErrorString(e.errorCode)
           << " ("
           << "0x" << std::hex << std::setfill('0') << std::setw(4) << e.errorCode
           << ")";
        throw std::runtime_error(ss.str());
    }
}

Device::Device(const std::string& name)
    : handle(HD_INVALID_HANDLE)
    , name(name)
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
    , name(std::move(other.name))
{
    other.handle = HD_INVALID_HANDLE;
}

Device& Device::operator=(Device&& other)
{
    handle = std::move(other.handle);
    name = std::move(other.name);
    other.handle = HD_INVALID_HANDLE;
    return *this;
}

void Device::set_force_enabled(bool enable)
{
    hdMakeCurrentDevice(handle);
    error_check();

    if (enable)
        hdEnable(HD_FORCE_OUTPUT);
    else
        hdDisable(HD_FORCE_OUTPUT);
}

Scheduler::Scheduler()
    : rate_hz(0)
{
}

Scheduler::~Scheduler()
{
    hdStopScheduler();

    for (auto h : handles_)
        hdUnschedule(h);
}

void Scheduler::set_rate(unsigned scheduler_rate_hz)
{
    hdSetSchedulerRate(scheduler_rate_hz);
    error_check();
    rate_hz = scheduler_rate_hz;
}

void Scheduler::start()
{
    hdStartScheduler();
    error_check();
}

void Scheduler::stop()
{
    hdStopScheduler();
    error_check();
}

ScopedFrame::ScopedFrame(HHD handle)
    : handle_(handle)
{
    hdBeginFrame(handle_);
    error_check();
}

ScopedFrame::~ScopedFrame()
{
    hdEndFrame(handle_);
    error_check();
}

} // namespace hd
