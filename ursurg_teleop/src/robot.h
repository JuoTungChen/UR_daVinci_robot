#pragma once

#include "common.h"

#include <ros/ros.h>

struct RobotState
{
    Eigen::Isometry3d tf = Eigen::Isometry3d::Identity();
};

class RobotStateReader
{
public:
    RobotStateReader(ros::NodeHandle& nh_left, ros::NodeHandle& nh_right);
    RobotState currentState(Index idx);

private:
    std::array<ros::Subscriber, 2> sub_;
    std::array<RobotState, 2> state_; // no need for synchronization iff ROS processes single-threaded
};
