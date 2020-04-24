/*
 * Raven2 - Software for the Raven II surgical robot
 * Copyright (C) 2016-2017 Kim Lindberg Schwaner <kils@mmmi.sdu.dk>
 *
 * This file is part of Raven2.
 *
 * Raven2 is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Lesser General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * Raven2 is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public License
 * along with Raven2.  If not, see <http://www.gnu.org/licenses/>.
 */

#include "hd++.h"

#include <geometry_msgs/Pose.h>

#include <ros/ros.h>

#include <Eigen/Geometry>

#include <chrono>

constexpr double pi = 3.14159265358979323846;

struct State
{
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    std::chrono::steady_clock::time_point stamp;

    // Transform of the end-effector (column-major) wrt. the base frame
    // of the Touch device
    Eigen::Isometry3d tf = Eigen::Isometry3d::Identity();

    // Button states (pushed or released)
    struct {
        bool grey = false;
        bool white = false;
    } button;
};

struct Diff
{
    // Difference from previous to current transform
    Eigen::Vector3d pos = Eigen::Vector3d::Zero();
    Eigen::Quaterniond ori = Eigen::Quaterniond::Identity();
    double grasp = 0;
};

geometry_msgs::Pose eigen_to_pose(const Eigen::Vector3d& p,
                                  const Eigen::Quaterniond& q)
{
    geometry_msgs::Pose pose;
    pose.position.x = p.x();
    pose.position.y = p.y();
    pose.position.z = p.z();
    pose.orientation.w = q.w();
    pose.orientation.x = q.x();
    pose.orientation.y = q.y();
    pose.orientation.z = q.z();
    return pose;
}

int main(int argc, char* argv[])
{
    using namespace std::string_literals;

    ros::init(argc, argv, "haptics_differential");
    ros::NodeHandle nh;
    ros::NodeHandle nh_priv("~");

    int scheduler_rate = nh_priv.param("scheduler_rate", 1000);
    double grasp_rate = nh_priv.param("grasp_rate", pi / 4); // rad/s
    double scaling_factor = nh_priv.param("scaling_factor", 1.0);

    std::array<ros::Publisher, 2> pub{{
        nh.advertise<geometry_msgs::Pose>("left/pose_increment", 1),
        nh.advertise<geometry_msgs::Pose>("right/pose_increment", 1)
    }};
    std::array<hd::Device, 2> dev{{
        hd::Device(nh_priv.param("device_name_left", "left"s)),
        hd::Device(nh_priv.param("device_name_right", "right"s))
    }};
    std::array<State, 2> state_current;
    std::array<State, 2> state_previous;

    hd::Scheduler scheduler(scheduler_rate);

    scheduler.schedule_asynchronous([&]() {
        for (std::size_t i = 0; i < 2; ++i) {
            hd::Frame frame(dev[i].handle);
            state_current[i].stamp = std::chrono::steady_clock::now();

            // Transform (column-major, translation in millimeters)
            hdGetDoublev(HD_CURRENT_TRANSFORM, state_current[i].tf.data());
            state_current[i].tf.translation() /= 1000; // Scale to meters


            // To rotate stylus around X (don't rotate translation part)
            state_current[i].tf.linear() = state_current[i].tf.linear()
                                           * Eigen::AngleAxisd(pi / 2, Eigen::Vector3d::UnitX());

            // Buttons
            int buttons = 0;
            hdGetIntegerv(HD_CURRENT_BUTTONS, &buttons);
            state_current[i].button.grey = (buttons & HD_DEVICE_BUTTON_1);
            state_current[i].button.white = (buttons & HD_DEVICE_BUTTON_2);
        }

        if (!nh.ok())
            return HD_CALLBACK_DONE;

        return HD_CALLBACK_CONTINUE;
    }, HD_MAX_SCHEDULER_PRIORITY);

    // Disable force output
    for (auto& d : dev) {
        hdMakeCurrentDevice(d.handle);
        hdDisable(HD_FORCE_OUTPUT);
    }

    scheduler.start();

    std::array<Diff, 2> diff;
    ros::Rate rate(nh_priv.param("publish_rate", scheduler_rate));

    while (nh.ok()) {
        scheduler.schedule_synchronous([&]() {
            for (std::size_t i = 0; i < 2; ++i) {
                std::chrono::duration<double> dt = state_current[i].stamp - state_previous[i].stamp;

                diff[i].pos = state_current[i].tf.translation() - state_previous[i].tf.translation();
                diff[i].ori = state_previous[i].tf.rotation().inverse() * state_current[i].tf.rotation();

                if (state_current[i].button.grey && !state_current[i].button.white) {
                    // Decrease grasp angle
                    diff[i].grasp = -grasp_rate * dt.count();
                } else if (!state_current[i].button.grey && state_current[i].button.white) {
                    // Increase grasp angle
                    diff[i].grasp = grasp_rate * dt.count();
                } else {
                    // Grasp angle remains unchanged
                    diff[i].grasp = 0;
                }

                state_previous[i] = std::move(state_current[i]);
            }

            return HD_CALLBACK_DONE;
        }, HD_MIN_SCHEDULER_PRIORITY);

        for (std::size_t i = 0; i < 2; ++i) {
            auto m = eigen_to_pose(diff[i].pos * scaling_factor, diff[i].ori);
            pub[i].publish(m);
        }

        rate.sleep();
    }

    return EXIT_SUCCESS;
}
