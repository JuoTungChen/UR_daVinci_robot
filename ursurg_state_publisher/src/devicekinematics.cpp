#include "devicekinematics.h"

#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/Vector3.h>





DeviceKinematics::DeviceKinematics(WorkCellContainer& wcContainer,
                                   std::string deviceName,
                                   std::string endEffectorFrameName,
                                   std::string targetPoseTopic,
                                   std::string currentPoseTopic,
                                   std::string ip, size_t filterTaps,
                                   ros::NodeHandle& nh)
    : wcc(wcContainer)
    , robotIp(ip)
    , filter(filterTaps)
{
    device = wcc.workcell->findDevice<rw::models::SerialDevice>(deviceName);
    if(device.isNull())
        throw std::runtime_error("Could not find device" + deviceName);

    targetPoseSub = nh.subscribe(targetPoseTopic, 10, &DeviceKinematics::targetPoseCallback, this);
    currentPosePub = nh.advertise<geometry_msgs::Pose>(currentPoseTopic, 10);
    filteredPosePub = nh.advertise<geometry_msgs::Pose>("servotopic", 10);



    // find end effector frame in workcell
    toolFrame  = wcc.workcell->findFrame(endEffectorFrameName);

    if (!toolFrame)
        throw std::runtime_error("Toolframe not found!");

    {
        std::unique_lock<std::mutex> lock(wcc.stateMtx);
        state = wcc.state;
    }
    solver = new rw::invkin::JacobianIKSolver(device, toolFrame, state);

    // start pose publish loop
    posePublishThread = std::thread([this](){ posePublishLoop(); });

    // start servo loop
    servoThread = std::thread([this](){ servoLoop(); });

}

void DeviceKinematics::targetPoseCallback(const geometry_msgs::PoseConstPtr &msg)
{


    geometry_msgs::Pose filteredPose = filter.filter(*msg);

    rw::math::Transform3D<double> wTtarget = convertToRw(filteredPose);
    filteredPosePub.publish(filteredPose);

    // oops, also deletetest this
    //wTtarget.R() = device->baseTend(state).R();

    rw::math::Q currentQ = device->getQ(state);

    std::vector<rw::math::Q> configurations = solver->solve(wTtarget,state);

    if (configurations.size() == 0) // error handling, error handling
        return;

    rw::math::Q bestConfig =  getClosestJointConfig(currentQ, configurations);
    if (bestConfig.size() == 6)
    {
        device->setQ(bestConfig, state);
        std::unique_lock<std::mutex> lock(targetJointConfigMtx);
        targetJointConfig = bestConfig.toStdVector();
    }
}

void DeviceKinematics::servoLoop()
{
    ur_rtde::RTDEControlInterface rtdeControl(robotIp);

    ros::Rate loop_rate(125);
    servoThreadRunning = true;
    while(ros::ok() && servoThreadRunning)
    {
        loop_rate.sleep();

        std::unique_lock<std::mutex> lock(targetJointConfigMtx);
        if (targetJointConfig.size() == 6)
        {
            /*for (int i = 0; i < 6; i++)
                std::cout << targetJointConfig[i] << ", ";
            std::cout << std::endl;*/
            rtdeControl.servoJ(targetJointConfig, 0, 0, 0.08, 0.1, 600);

        }

        
    }
    rtdeControl.servoStop();

}

void DeviceKinematics::posePublishLoop()
{
    ur_rtde::RTDEReceiveInterface rtdeReceive(robotIp);

    ros::Rate loop_rate(125);
    posePublishThreadRunning = true;
    while(ros::ok() && posePublishThreadRunning)
    {
        std::vector<double> joint_position = rtdeReceive.getActualQ();
        rw::math::Transform3D<double> transform;
        {
            device->setQ(rw::math::Q(joint_position),state);
            // get transform from the device
            transform = device->baseTframe(toolFrame,state);
        }
        geometry_msgs::Pose pose = convertToRos(transform);

        currentPosePub.publish(pose);
        loop_rate.sleep();
    }

}

rw::math::Transform3D<double> DeviceKinematics::convertToRw(const geometry_msgs::Pose &msg)
{
    rw::math::Vector3D<double> pos(
                msg.position.x,
                msg.position.y,
                msg.position.z);
    //std::cerr << pos << std::endl;

    const rw::math::Quaternion<double> qua(
                msg.orientation.x,
                msg.orientation.y,
                msg.orientation.z,
                msg.orientation.w);

    //std::cerr << qua << std::endl;
    return rw::math::Transform3D<double>(pos, qua.toRotation3D());
}

geometry_msgs::Pose DeviceKinematics::convertToRos(const rw::math::Transform3D<double> &transform)
{
    geometry_msgs::Pose pose;
    const rw::math::Vector3D<double> vec = transform.P();

    pose.position.x = vec[0];
    pose.position.y = vec[1];
    pose.position.z = vec[2];

    const rw::math::Quaternion<double> qua(transform.R());

    pose.orientation.x = qua.getQx();
    pose.orientation.y = qua.getQy();
    pose.orientation.z = qua.getQz();
    pose.orientation.w = qua.getQw();

    return pose;

}

rw::math::Q DeviceKinematics::getClosestJointConfig(rw::math::Q referenceQ, const std::vector<rw::math::Q> &configurations)
{
    rw::math::Q bestQ;
    double lowestQSpaceDistance = 1000;
    for (auto &config : configurations)
    {
        double distanceFromCurrent = (referenceQ - config).norm2();
        if (distanceFromCurrent < lowestQSpaceDistance)
        {
            bestQ = config;
            lowestQSpaceDistance = distanceFromCurrent;
        }
    }
    return bestQ;
}
