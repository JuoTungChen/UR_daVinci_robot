#include <ursurg_common/rosutility.h>

#include <geometry_msgs/PoseStamped.h>
#include <sensor_msgs/JointState.h>

#include <rw/invkin/JacobianIKSolver.hpp>
#include <rw/loaders/WorkCellLoader.hpp>
#include <rw/models/SerialDevice.hpp>
#include <rw/models/WorkCell.hpp>

#include <ros/ros.h>

#include <optional>

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

    auto workcell_path = nh_priv.param("workcell", ""s);
    auto device_name = nh_priv.param("device", ""s);

    auto workcell = rw::loaders::WorkCellLoader::Factory::load(workcell_path);

    if (!workcell)
        throw std::runtime_error("Workcell '" + workcell_path + "' not found");

    auto device = workcell->findDevice<rw::models::SerialDevice>(device_name);

    if (!device)
        throw std::runtime_error("Device '" + device_name + "' not found");

    // Accessing 'state' does not have to be synchronized when processing ROS
    // callbacks from only a single thread
    auto state = workcell->getDefaultState();

    rw::invkin::JacobianIKSolver ik_solver(device, state);
    //ik_solver.setCheckJointLimits(true);

    auto solve_ik = [&](rw::math::Transform3D<double> tf) -> std::optional<std::vector<double>> {
        auto solutions = ik_solver.solve(tf, state);

        if (solutions.empty()) {
            ROS_WARN("No IK solutions");
            return {};
        }

        // JacobianIKSolver returns only one solution
        return solutions.front().toStdVector();
    };

    auto pub_pose = nh.advertise<geometry_msgs::PoseStamped>("pose_tcp_current", 1);
    auto pub_movej = nh.advertise<sensor_msgs::JointState>("move_j", 1);
    auto pub_servoj = nh.advertise<sensor_msgs::JointState>("servo_j", 1);

    std::list<ros::Subscriber> subscribers{
        mksub<sensor_msgs::JointState>(nh, "joint_states", 1, [&](const auto& m) {
            device->setQ(m.position, state);

            geometry_msgs::PoseStamped s;
            s.header.stamp = ros::Time::now();
            s.pose = convert(device->baseTend(state));
            pub_pose.publish(s);
        }),
        mksub<geometry_msgs::PoseStamped>(nh, "move_j_ik", 1, [&](const auto& m) {
            auto q = solve_ik(convert(m.pose));

            if (!q)
                return;

            sensor_msgs::JointState s;
            s.header.stamp = ros::Time::now();
            s.position = std::move(*q);
            pub_movej.publish(s);
        }),
        mksub<geometry_msgs::PoseStamped>(nh, "servo_j_ik", 1, [&](const auto& m) {
            auto q = solve_ik(convert(m.pose));

            if (!q)
                return;

            sensor_msgs::JointState s;
            s.header.stamp = ros::Time::now();
            s.position = std::move(*q);
            pub_servoj.publish(s);
        }),
    };

    ros::spin();

    return EXIT_SUCCESS;
}
