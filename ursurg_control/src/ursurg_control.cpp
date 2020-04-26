#include <ursurg_common/pair.h>

#include <geometry_msgs/PoseStamped.h>
#include <sensor_msgs/JointState.h>

#include <rw/invkin/JacobianIKSolver.hpp>
#include <rw/loaders/WorkCellLoader.hpp>
#include <rw/models/SerialDevice.hpp>
#include <rw/models/WorkCell.hpp>

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
        , pub_pose_(nh_.advertise<geometry_msgs::PoseStamped>("pose_tcp_current", 1))
        , pub_movej(nh_.advertise<sensor_msgs::JointState>("move_j", 1))
        , pub_servoj(nh_.advertise<sensor_msgs::JointState>("servo_j", 1))
        , sub_joint_state_(nh_.subscribe("joint_states", 1, &StatePublisher::do_fk, this))
    {
    }

    void do_fk(const sensor_msgs::JointState& m)
    {
        device_->setQ(m.position, state_);

        geometry_msgs::PoseStamped y;
        y.header.stamp = ros::Time::now();
        y.pose = convert(device_->baseTend(state_));
        pub_pose_.publish(y);
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
        pub_movej = nh_.advertise<sensor_msgs::JointState>("move_j", 1);
        pub_servoj = nh_.advertise<sensor_msgs::JointState>("servo_j", 1);
        subscribers_.push_back(nh_.subscribe("move_j_ik", 1, &Controller::move_j_ik, this));
        subscribers_.push_back(nh_.subscribe("servo_j_ik", 1, &Controller::servo_j_ik, this));
    }

    void move_j_ik(const geometry_msgs::PoseStamped& m)
    {
        auto sols = ik_solver_.solve(convert(m.pose), state_);
        ROS_DEBUG("sols.size() = %lu", sols.size());

        if (sols.empty()) {
            ROS_WARN("No IK solutions");
            return;
        }

        sensor_msgs::JointState s;
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

        sensor_msgs::JointState s;
        s.header.stamp = ros::Time::now();
        s.position = sols.front().toStdVector();
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

    auto workcell = rw::loaders::WorkCellLoader::Factory::load(nh_priv.param("workcell", ""s));

    auto state = workcell->getDefaultState();

    auto device = make_pair(
        workcell->findDevice<rw::models::SerialDevice>("UR5e_Left"),
        workcell->findDevice<rw::models::SerialDevice>("UR5_Right"));
    Pair<ros::NodeHandle> nh_lr({nh, "left"}, {nh, "right"});
    Pair<StatePublisher> pub_state({nh_lr.left, device.left, state},
                                   {nh_lr.right, device.right, state});
    Pair<Controller> controller({nh_lr.left, device.left, state},
                                {nh_lr.right, device.right, state});

    ros::spin();

    return EXIT_SUCCESS;
}
