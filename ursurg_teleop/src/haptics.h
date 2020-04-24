#pragma once

#include "common.h"
#include "hd++.h"

#include <ursurg_common/math.h>
#include <ursurg_common/synchronized.h>

#include <Eigen/Geometry>

#include <chrono>

enum class ButtonState
{
    PRESSED,
    RELEASED,
};

// Difference from previous to current state
struct HapticsStateDiff
{
    std::chrono::steady_clock::duration dt;
    Eigen::Isometry3d tf = Eigen::Isometry3d::Identity();
};

struct HapticsState
{
    std::chrono::steady_clock::time_point stamp;

    // Transform of the end-effector (column-major) wrt. the base frame
    // of the Haptics device
    Eigen::Isometry3d tf = Eigen::Isometry3d::Identity();

    struct
    {
        ButtonState grey = ButtonState::RELEASED;
        ButtonState white = ButtonState::RELEASED;
    } button_state;

    HapticsStateDiff operator-(const HapticsState& rhs) const;
};

class HapticsStateReader
{
public:
    explicit HapticsStateReader(const std::string& name_left,
                                const std::string& name_right,
                                unsigned scheduler_rate_hz);

    HapticsState currentState(Index idx) const;

private:
    HDCallbackCode updateDeviceState(Index idx);

private:
    std::array<hd::Device, 2> devices_;
    hd::Scheduler scheduler_;
    std::array<synchronized<HapticsState>, 2> states_;
};
