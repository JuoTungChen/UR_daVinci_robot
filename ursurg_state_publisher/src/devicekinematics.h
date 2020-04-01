#ifndef DEVICEKINEMATICS_H
#define DEVICEKINEMATICS_H

#include <thread>
#include <vector>

#include <geometry_msgs/Pose.h>
#include <ros/ros.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <trajectory_msgs/JointTrajectoryPoint.h>

#include <rw/invkin/JacobianIKSolver.hpp>
#include <rw/math/Transform3D.hpp>
#include <rw/math/Q.hpp>

#include <stdexcept>
#include <mutex>

#include <ur_rtde/rtde_control_interface.h>
#include <ur_rtde/rtde_receive_interface.h>


#include "workcellcontainer.h"

class PoseFilter
{
public:
    // https://link-springer-com.proxy1-bib.sdu.dk/content/pdf/10.1023/A:1011129215388.pdf
    PoseFilter(size_t size = 5) : filterSize(size) {}

    geometry_msgs::Quaternion getAverageQua()
    {
        geometry_msgs::Quaternion quatAcc;
        for (auto &p : prevPoses)
        {
            quatAcc.x += p.orientation.x;
            quatAcc.y += p.orientation.y;
            quatAcc.z += p.orientation.z;
            quatAcc.w += p.orientation.w;
        }

        double norm = std::sqrt(std::pow(quatAcc.x,2)
                    + std::pow(quatAcc.y,2)
                    + std::pow(quatAcc.z,2)
                    + std::pow(quatAcc.w,2));

        quatAcc.x /= norm;
        quatAcc.y /= norm;
        quatAcc.z /= norm;
        quatAcc.w /= norm;

        return quatAcc;
    }
    geometry_msgs::Point getAveragePos()
    {
        geometry_msgs::Point avgPoint;
        const double scaling = 1.0 / prevPoses.size();
        for (auto &p : prevPoses)
        {
            avgPoint.x += p.position.x * scaling;
            avgPoint.y += p.position.y * scaling;
            avgPoint.z += p.position.z * scaling;
        }

        return avgPoint;
    }



    geometry_msgs::Pose filter(const geometry_msgs::Pose newPose)
    {
        prevPoses.push_back(newPose);

        if (prevPoses.size() > filterSize)
        {
            prevPoses.erase(prevPoses.begin());
        }

        geometry_msgs::Pose filteredPose;
        filteredPose.position = newPose.position;//getAveragePos();
        filteredPose.orientation = getAverageQua();

        return filteredPose;
    }

private:
    size_t filterSize;
    std::vector<geometry_msgs::Pose> prevPoses;
};

class DeviceKinematics
{
public:
    DeviceKinematics(WorkCellContainer& wcContain10er,
                     std::string deviceName,
                     std::string endEffectorFrameName,
                     std::string targetPoseTopic,
                     std::string currentPoseTopic,
                     std::string ip,
                     size_t filterTaps,
                     ros::NodeHandle& nh);

    void targetPoseCallback(const geometry_msgs::PoseConstPtr &msg);

private:
    rw::math::Transform3D<double> convertToRw(const geometry_msgs::Pose &msg);
    geometry_msgs::Pose convertToRos(const rw::math::Transform3D<double> &transform);
    rw::math::Q getClosestJointConfig(rw::math::Q referenceQ, const std::vector<rw::math::Q> &configurations);
    void servoLoop();
    void posePublishLoop();

private:

    WorkCellContainer& wcc;
    rw::kinematics::State state;
    std::string robotIp;
    rw::models::SerialDevice::Ptr device;
    rw::invkin::JacobianIKSolver::Ptr solver;
    PoseFilter filter;

    
    std::mutex targetJointConfigMtx;
    std::vector<double> targetJointConfig;
    rw::kinematics::Frame* toolFrame;


    ros::Subscriber targetPoseSub;
    ros::Publisher currentPosePub;
    ros::Publisher filteredPosePub;

    std::thread servoThread;
    std::atomic_bool servoThreadRunning;

    std::thread posePublishThread;
    std::atomic_bool posePublishThreadRunning;


};

#endif // DEVICEKINEMATICS_H
