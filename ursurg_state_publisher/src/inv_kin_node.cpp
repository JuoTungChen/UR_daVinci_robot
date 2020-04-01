#include <ros/ros.h>

#include <vector>
#include <string>
#include <memory>


#include <trajectory_msgs/JointTrajectory.h>
#include <trajectory_msgs/JointTrajectoryPoint.h>

#include "devicekinematics.h"

int main (int argc, char **argv)
{
    ros::init(argc, argv, "inv_kin_node");

    std::string fileName = "src/ur-surg/workcell/ROSOR_2x_UR_platform.wc.xml";

    ros::NodeHandle nh;

    WorkCellContainer wcc(fileName);

    DeviceKinematics left(
                wcc,
                "UR5e_Left",
                "Tool_Left",
                "/left_robot/target_pose",
                "/left_robot/current_pose",
                "192.168.10.201",
                10,
                nh);


    DeviceKinematics right(
                wcc,
                "UR5_Right",
                "Tool_Right",
                "/right_robot/target_pose",
                "/right_robot/current_pose",
                "192.168.10.202",
                10,
                nh);


//
    ros::spin();


    return 0;
}
