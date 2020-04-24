#include "haptics.h"

HapticsStateDiff HapticsState::operator-(const HapticsState& rhs) const
{
    HapticsStateDiff diff;

    diff.dt = stamp - rhs.stamp;

    // w/ quaternion
    // diff.pos = tf.translation() - rhs.tf.translation();
    // diff.ori = rhs.tf.rotation().inverse() * tf.rotation();

    diff.tf.translation() = tf.translation() - rhs.tf.translation();
    diff.tf.linear() = rhs.tf.linear().inverse() * tf.linear();

    // FIXME is this the same as above?
    // diff.tf = rhs.tf.inverse() * tf;

    return diff;
}

HapticsStateReader::HapticsStateReader(const std::string& name_left,
                                       const std::string& name_right,
                                       unsigned scheduler_rate_hz)
    : devices_{name_left, name_right}
{
    devices_[L].set_force_enabled(false);
    devices_[R].set_force_enabled(false);

    scheduler_.schedule_asynchronous([this]() { return updateDeviceState(L); }, HD_MAX_SCHEDULER_PRIORITY);
    scheduler_.schedule_asynchronous([this]() { return updateDeviceState(R); }, HD_MAX_SCHEDULER_PRIORITY);

    scheduler_.set_rate(scheduler_rate_hz);
    scheduler_.start();
}

HapticsState HapticsStateReader::currentState(Index idx) const
{
    return states_[idx];
}

HDCallbackCode HapticsStateReader::updateDeviceState(Index idx)
{
    states_[idx].withLock([&](auto& s) {
        hd::ScopedFrame frame(devices_[idx].handle);

        s.stamp = std::chrono::steady_clock::now();

        // Transform (column-major, translation in millimeters)
        hdGetDoublev(HD_CURRENT_TRANSFORM, s.tf.data());
        s.tf.translation() /= 1000; // Scale to meters

        // To rotate stylus around X (but don't rotate translation part)
        s.tf.linear() = s.tf.linear() * Eigen::AngleAxisd(math::half_pi, Eigen::Vector3d::UnitX());

        // Buttons
        int buttons = 0;
        hdGetIntegerv(HD_CURRENT_BUTTONS, &buttons);
        s.button_state.grey = (buttons & HD_DEVICE_BUTTON_1) ? ButtonState::PRESSED : ButtonState::RELEASED;
        s.button_state.white = (buttons & HD_DEVICE_BUTTON_2) ? ButtonState::PRESSED : ButtonState::RELEASED;
    });

    //    if (!ros::ok())
    //        return HD_CALLBACK_DONE;

    return HD_CALLBACK_CONTINUE;
}
