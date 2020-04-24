#include "robot.h"

#include <geometry_msgs/PoseStamped.h>

#include <ursurg_common/rosutility.h>

RobotStateReader::RobotStateReader(ros::NodeHandle& nh_left, ros::NodeHandle& nh_right)
{
    // TODO should really subscribe to the current TCP "target" pose
    sub_[L] = mksub<geometry_msgs::PoseStamped>(
        nh_left, "pose_tcp_current", 1, [this](const auto& m) {
            state_[L].tf = convert(m.pose);
        },
        ros::TransportHints().tcpNoDelay());
    sub_[R] = mksub<geometry_msgs::PoseStamped>(
        nh_right, "pose_tcp_current", 1, [this](const auto& m) {
            state_[R].tf = convert(m.pose);
        },
        ros::TransportHints().tcpNoDelay());
}

RobotState RobotStateReader::currentState(Index idx)
{
    return state_[idx];
}
