#pragma once

#include "common.h"

#include <ursurg_common/pair.h>

#include <ros/ros.h>

struct RobotState
{
    Eigen::Isometry3d tf = Eigen::Isometry3d::Identity();
};

class RobotStateReader
{
public:
    RobotStateReader(Pair<ros::NodeHandle>& nh);
    RobotState currentState(PairIndex idx);

private:
    Pair<ros::Subscriber> sub_;
    Pair<RobotState> state_; // no need for synchronization iff ROS processes single-threaded
};
