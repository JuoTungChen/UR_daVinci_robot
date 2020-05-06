#include "robot.h"

#include <geometry_msgs/PoseStamped.h>

#include <ursurg_common/rosutility.h>

RobotStateReader::RobotStateReader(Pair<ros::NodeHandle>& nh)
{
    // TODO should really subscribe to the current TCP "target" pose
    sub_[LEFT] = mksub<geometry_msgs::PoseStamped>(
        nh.left, "tcp_pose_current", 1, [this](const auto& m) {
            state_[LEFT].tf = convert(m.pose);
        },
        ros::TransportHints().tcpNoDelay());
    sub_[RIGHT] = mksub<geometry_msgs::PoseStamped>(
        nh.right, "tcp_pose_current", 1, [this](const auto& m) {
            state_[RIGHT].tf = convert(m.pose);
        },
        ros::TransportHints().tcpNoDelay());
}

RobotState RobotStateReader::currentState(PairIndex idx)
{
    return state_[idx];
}
