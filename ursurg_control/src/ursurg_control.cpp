#include <ursurg_common/rosutility.h>

#include <geometry_msgs/PoseStamped.h>
#include <sensor_msgs/JointState.h>

//#include <rw/invkin/JacobianIKSolver.hpp>
#include "WeightedJacobianIKSolver.hpp" // to be moved to rw later
#include <rw/kinematics/FixedFrame.hpp>
#include <rw/loaders/WorkCellLoader.hpp>
#include <rw/models/CompositeJointDevice.hpp>
#include <rw/models/SerialDevice.hpp>
#include <rw/models/TreeDevice.hpp>
#include <rw/models/WorkCell.hpp>

#include <ros/ros.h>

#include <optional>

// Basic rotation about the z axis with angle a
rw::math::Rotation3D<double> rot_z(double a)
{
    auto ca = std::cos(a);
    auto sa = std::sin(a);
    // clang-format off
    return {ca, -sa,  0,
            sa,  ca,  0,
            0,   0,   1};
    // clang-format on
}

geometry_msgs::Pose convert(const rw::math::Transform3D<double>& tf)
{
    geometry_msgs::Pose m;
    m.position.x = tf.P()[0];
    m.position.y = tf.P()[1];
    m.position.z = tf.P()[2];
    rw::math::Quaternion<double> q(tf.R());
    m.orientation.x = q.getQx();
    m.orientation.y = q.getQy();
    m.orientation.z = q.getQz();
    m.orientation.w = q.getQw();
    return m;
}

rw::math::Transform3D<double> convert(const geometry_msgs::Pose& m)
{
    rw::math::Vector3D<double> p(m.position.x, m.position.y, m.position.z);
    rw::math::Quaternion<double> q(m.orientation.x, m.orientation.y, m.orientation.z, m.orientation.w);
    return {p, q.toRotation3D()};
}

int main(int argc, char* argv[])
{
    using namespace std::string_literals;

    ros::init(argc, argv, "ursurg_control");
    ros::NodeHandle nh;
    ros::NodeHandle nh_priv("~");

    auto workcell_path = nh_priv.param("rw_workcell_path", ""s);
    auto robot_name = nh_priv.param("robot_name", ""s);
    auto tool_name = nh_priv.param("tool_name", ""s);

    auto workcell = rw::loaders::WorkCellLoader::Factory::load(workcell_path);

    if (!workcell)
        throw std::runtime_error("Workcell '" + workcell_path + "' not found");

    auto robot = workcell->findDevice(robot_name);
    auto tool = workcell->findDevice(tool_name);

    if (!robot)
        throw std::runtime_error("Robot device '" + robot_name + "' not found");

    if (!tool)
        throw std::runtime_error("Tool device '" + tool_name + "' not found");

    auto tcp = tool->getEnd(); // aka. tool->_ends.front()

    ROS_INFO_STREAM("Robot: " << robot->getName());
    ROS_INFO_STREAM("Tool: " << tool->getName());
    ROS_INFO_STREAM("TCP: " << tcp->getName());

    // Access to 'state' does not have to be synchronized when processing ROS
    // callbacks from only a single thread
    auto state = workcell->getDefaultState();

    // Composite device makes a single entity from the combined robot and tool
    rw::models::CompositeJointDevice cdev(robot->getBase(),
                                          {robot, tool},
                                          tcp,
                                          robot_name + "_" + tool_name,
                                          state);

    WeightedJacobianIKSolver ik_solver(&cdev, state);
    ik_solver.setCheckJointLimits(true);
    ik_solver.setEnableInterpolation(false);
    auto weights = nh_priv.param("weights", std::vector<double>(cdev.getDOF(), 1.0));
    ik_solver.setWeightVector(Eigen::VectorXd::Map(weights.data(), weights.size()));

    auto pub_pose = nh.advertise<geometry_msgs::PoseStamped>("tcp_pose_current", 1);
    auto pub_robot_move_joint = nh.advertise<sensor_msgs::JointState>("ur_move_joint", 1);
    auto pub_robot_servo_joint = nh.advertise<sensor_msgs::JointState>("ur_servo_joint", 1);
    auto pub_tool_move_joint = nh.advertise<sensor_msgs::JointState>("tool_move_joint", 1);
    auto pub_tool_servo_joint = nh.advertise<sensor_msgs::JointState>("tool_servo_joint", 1);

    auto solve_and_make_msgs = [&](const auto& m) -> std::optional<std::pair<sensor_msgs::JointState, sensor_msgs::JointState>> {
        auto solutions = ik_solver.solve(convert(m.pose), state);

        if (solutions.empty()) {
            ROS_WARN("No IK solutions");
            return {};
        }

        // JacobianIKSolver returns only one solution
        assert(solutions.front().size() == 10);
        auto dptr = solutions.front().e().data(); // e() returns ref. to Q::_vec (an Eigen vector)

        // The first 6 elements of the solution vector is the robot configuration
        sensor_msgs::JointState m_robot;
        m_robot.header.stamp = ros::Time::now();
        std::copy(dptr, std::next(dptr, 6), std::back_inserter(m_robot.position));

        // and the last 4 is the tool configuration
        sensor_msgs::JointState m_tool;
        m_tool.header.stamp = m_robot.header.stamp;
        m_tool.name = {"roll", "wrist", "jaw1", "jaw2"}; // names matter to the eua_control node
        std::copy(std::next(dptr, 6), std::next(dptr, 10), std::back_inserter(m_tool.position));

        return std::make_pair(m_robot, m_tool);
    };

    std::list<ros::Subscriber> subscribers{
        mksub<sensor_msgs::JointState>(
            nh, "ur_joint_states", 1, [&](const auto& m) {
                robot->setQ(m.position, state);
            },
            ros::TransportHints().tcpNoDelay()),
        mksub<sensor_msgs::JointState>(
            nh, "tool_joint_states", 1, [&](const auto& m) {
                tool->setQ(m.position, state);
            },
            ros::TransportHints().tcpNoDelay()),
        mksub<geometry_msgs::PoseStamped>(
            nh, "move_joint_ik", 1, [&](const auto& m) {
                // "move" set-points may lie be anywhere in the workspace, so
                // enable linear interpolation of via-points toward the target
                ik_solver.setEnableInterpolation(true);
                auto s = solve_and_make_msgs(m);
                ik_solver.setEnableInterpolation(false);

                if (s) {
                    pub_robot_move_joint.publish(s->first);
                    pub_tool_move_joint.publish(s->second);
                }
            },
            ros::TransportHints().tcpNoDelay()),
        mksub<geometry_msgs::PoseStamped>(
            nh, "servo_joint_ik", 1, [&](const auto& m) {
                auto s = solve_and_make_msgs(m);

                if (s) {
                    pub_robot_move_joint.publish(s->first);
                    pub_tool_move_joint.publish(s->second);
                }
            },
            ros::TransportHints().tcpNoDelay()),
    };

    // Schedule timer to publish robot state
    auto timer = nh.createSteadyTimer(
        ros::WallDuration(1.0 / 100),
        [&](const auto&) {
            geometry_msgs::PoseStamped m;
            m.header.stamp = ros::Time::now();
            m.header.frame_id = cdev.getBase()->getName();
            m.pose = convert(cdev.baseTend(state));
            pub_pose.publish(m);
        });

    ros::spin();

    return EXIT_SUCCESS;
}
