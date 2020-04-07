#include <rw/invkin/JacobianIKSolver.hpp>
#include <rw/loaders/WorkCellLoader.hpp>
#include <rw/models/SerialDevice.hpp>
#include <rw/models/WorkCell.hpp>

#include <geometry_msgs/PoseStamped.h>
#include <sensor_msgs/JointState.h>
#include <ur_control_msgs/MoveJoint.h>
#include <ur_control_msgs/ServoJoint.h>

#include <ros/ros.h>

auto convert(const rw::math::Transform3D<double>& tf)
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

auto convert(const geometry_msgs::Pose& m)
{
    rw::math::Vector3D<double> p(m.position.x, m.position.y, m.position.z);
    rw::math::Quaternion<double> q(m.orientation.x, m.orientation.y, m.orientation.z, m.orientation.w);
    return rw::math::Transform3D<double>(p, q.toRotation3D());
}

class StatePublisher
{
public:
    StatePublisher(const ros::NodeHandle& nh,
                   const rw::models::SerialDevice::Ptr& device,
                   rw::kinematics::State& state)
        : device_(device)
        , state_(state)
        , nh_(nh)
        , pub_pose_(nh_.advertise<geometry_msgs::PoseStamped>("tcp_pose_current", 1))
        , pub_movej(nh_.advertise<ur_control_msgs::MoveJoint>("move_j", 1))
        , pub_servoj(nh_.advertise<ur_control_msgs::ServoJoint>("servo_j", 1))
        , sub_joint_state_(nh_.subscribe("joint_state", 1, &StatePublisher::do_fk, this))
    {
    }

    void do_fk(const sensor_msgs::JointState& m)
    {
        device_->setQ(m.position, state_); // updates the shared state
        pub_pose_.publish(convert(device_->baseTend(state_)));
    }

private:
    rw::models::SerialDevice::Ptr device_;
    rw::kinematics::State& state_;

    ros::NodeHandle nh_;
    ros::Publisher pub_pose_;
    ros::Publisher pub_movej;
    ros::Publisher pub_servoj;
    ros::Subscriber sub_joint_state_;
};

class Controller
{
public:
    Controller(const ros::NodeHandle& nh,
               const rw::models::SerialDevice::Ptr& device,
               rw::kinematics::State& state)
        : device_(device)
        , state_(state)
        , ik_solver_(device_, state_)
        , nh_(nh)
    {
        ik_solver_.setCheckJointLimits(true);
        pub_movej = nh_.advertise<ur_control_msgs::MoveJoint>("move_j", 1);
        pub_servoj = nh_.advertise<ur_control_msgs::ServoJoint>("servo_j", 1);
        subscribers_.push_back(nh_.subscribe("move_j_ik", 1, &Controller::move_l_ik, this));
        subscribers_.push_back(nh_.subscribe("servo_j_ik", 1, &Controller::servo_j_ik, this));
    }

    void move_l_ik(const geometry_msgs::PoseStamped& m)
    {
        auto sols = ik_solver_.solve(convert(m.pose), state_);
        ROS_DEBUG("sols.size() = %lu", sols.size());

        if (sols.empty()) {
            ROS_WARN("No IK solutions");
            return;
        }

        ur_control_msgs::MoveJoint s;
        s.header.stamp = ros::Time::now();
        s.position = sols.front().toStdVector();
        pub_movej.publish(s);
    }

    void servo_j_ik(const geometry_msgs::PoseStamped& m)
    {
        auto sols = ik_solver_.solve(convert(m.pose), state_);
        ROS_DEBUG("sols.size() = %lu", sols.size());

        if (sols.empty()) {
            ROS_WARN("No IK solutions");
            return;
        }

        ur_control_msgs::ServoJoint s;
        s.header.stamp = ros::Time::now();
        s.position = sols.front().toStdVector();
        s.lookahead_time = 0.1;
        s.gain = 600;
        pub_servoj.publish(s);
    }

private:
    rw::models::SerialDevice::Ptr device_;
    rw::kinematics::State& state_;
    rw::invkin::JacobianIKSolver ik_solver_;

    ros::NodeHandle nh_;
    ros::Publisher pub_pose_;
    ros::Publisher pub_movej;
    ros::Publisher pub_servoj;
    std::list<ros::Subscriber> subscribers_;
};

int main(int argc, char* argv[])
{
    using namespace std::string_literals;

    ros::init(argc, argv, "ursurg_control");
    ros::NodeHandle nh;
    ros::NodeHandle nh_priv("~");

    auto workcell_path = nh_priv.param("workcell_path", "todo"s);
    auto workcell = rw::loaders::WorkCellLoader::Factory::load(workcell_path);

    auto state = workcell->getDefaultState(); // shared

    ros::NodeHandle nh_left(nh, "left");
    auto device_left = workcell->findDevice<rw::models::SerialDevice>("UR5e_Left");
    StatePublisher pub_state_left(nh_left, device_left, state);
    Controller ctrl_left(nh_left, device_left, state);

    ros::NodeHandle nh_right(nh, "right");
    auto device_right = workcell->findDevice<rw::models::SerialDevice>("UR5_Right");
    StatePublisher pub_state_right(nh_right, device_right, state);
    Controller ctrl_right(nh_right, device_right, state);

    ros::spin();

    return EXIT_SUCCESS;
}
