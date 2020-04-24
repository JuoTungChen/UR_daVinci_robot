#include <ur_rtde/rtde_control_interface.h>
#include <ur_rtde/rtde_receive_interface.h>

#include <geometry_msgs/PoseStamped.h>
#include <std_msgs/Bool.h>

#include <ros/ros.h>

#include <Eigen/Geometry>

#include <thread>

Eigen::Isometry3d to_eigen_tf(const geometry_msgs::Pose& pose)
{
    return Eigen::Translation3d(pose.position.x, pose.position.y, pose.position.z)
           * Eigen::Quaterniond(pose.orientation.w, pose.orientation.x, pose.orientation.y, pose.orientation.z);
}

Eigen::Isometry3d to_eigen_tf(const std::vector<double>& pose)
{
    Eigen::Vector3d r(pose[3], pose[4], pose[5]);
    return Eigen::Translation3d(pose[0], pose[1], pose[2]) * Eigen::AngleAxisd(r.norm(), r.normalized());
}

std::tuple<Eigen::Vector3d, Eigen::Quaterniond> to_eigen(const std::vector<double>& pose)
{
    Eigen::Vector3d r(pose[3], pose[4], pose[5]);
    Eigen::AngleAxisd aa(r.norm(), r.normalized());
    return {Eigen::Vector3d(pose[0], pose[1], pose[2]), Eigen::Quaterniond(aa)};
}

std::tuple<Eigen::Vector3d, Eigen::Quaterniond> to_eigen(const geometry_msgs::Pose& pose)
{
    return {
        Eigen::Vector3d(pose.position.x, pose.position.y, pose.position.z),
        Eigen::Quaterniond(pose.orientation.w, pose.orientation.x, pose.orientation.y, pose.orientation.z)
    };
}

std::vector<double> to_ur(const Eigen::Vector3d& p, const Eigen::Quaterniond& q)
{
    Eigen::AngleAxisd aa(q);
    Eigen::Vector3d r = aa.axis() * aa.angle();
    return {p[0], p[1], p[2], r[0], r[1], r[2]};
}
std::vector<double> to_ur(const Eigen::Isometry3d& tf)
{
    Eigen::AngleAxisd aa(tf.rotation());
    Eigen::Vector3d r = aa.axis() * aa.angle();
    return {tf.translation().x(), tf.translation().y(), tf.translation().z(), r[0], r[1], r[2]};
}

geometry_msgs::Pose to_msg(const Eigen::Vector3d& p, const Eigen::Quaterniond& q)
{
    geometry_msgs::Pose pose;
    pose.position.x = p.x();
    pose.position.y = p.y();
    pose.position.z = p.z();
    pose.orientation.w = q.w();
    pose.orientation.x = q.x();
    pose.orientation.y = q.y();
    pose.orientation.z = q.z();
    return pose;
}

geometry_msgs::Pose to_msg(const std::vector<double>& pose)
{
    using F = geometry_msgs::Pose (const Eigen::Vector3d&, const Eigen::Quaterniond&);
    return std::apply<F>(to_msg, to_eigen(pose));
}

template<typename M, typename F>
auto sub(ros::NodeHandle& nh,
         const std::string& topic,
         uint32_t queue_size,
         F callback,
         const ros::TransportHints& transport_hints = ros::TransportHints())
{
    return nh.subscribe<M>(topic,
                           queue_size,
                           boost::function<void(const M&)>(std::move(callback)),
                           ros::VoidConstPtr(),
                           transport_hints);
}

int main(int argc, char* argv[])
{
    using namespace std::string_literals;

    ros::init(argc, argv, "ur_teleop");
    ros::NodeHandle nh;
    ros::NodeHandle nh_priv("~");

    auto robot_ip = nh_priv.param("robot_ip", "192.168.10.201"s);
    ur_rtde::RTDEReceiveInterface rtde_recv(robot_ip);
    ur_rtde::RTDEControlInterface rtde_ctrl(robot_ip);

    auto update_period = rtde_ctrl.getStepTime();
    std::atomic<bool> clutched = false;
    Eigen::Isometry3d t_geomagic_base = Eigen::Isometry3d::Identity();
    t_geomagic_base.linear() << 1,  0,  0,
                                0,  0,  1,
                                0, -1,  0;
    Eigen::Isometry3d t_base_geomagic = t_geomagic_base.inverse();
    Eigen::Isometry3d t_base_tcp_desired = to_eigen_tf(rtde_recv.getActualTCPPose());
    std::mutex mutex_t_desired;

    auto pub_pose = nh.advertise<geometry_msgs::PoseStamped>("pose_tcp", 1);

    auto timer = nh.createTimer(ros::Duration(update_period), [&](const auto&) {
        // Publish robot pose
        geometry_msgs::PoseStamped m;
        m.header.stamp = ros::Time::now();
        m.pose = to_msg(rtde_recv.getActualTCPPose());
        pub_pose.publish(m);
    });

    std::list<ros::Subscriber> subscribers{
        sub<std_msgs::Bool>(nh, "clutch", 4, [&](const auto& m) {
            clutched = m.data;

            if (clutched) {
                // Set desired = current
                std::lock_guard<std::mutex> lock(mutex_t_desired);
                t_base_tcp_desired = to_eigen_tf(rtde_recv.getActualTCPPose());
            }
        }, ros::TransportHints().tcpNoDelay()),
        sub<geometry_msgs::Pose>(nh, "pose_increment", 16, [&](const auto& m) {
            // Add increment to the desired TCP transform if clutched
            if (clutched) {
//                auto t_incr = to_eigen_tf(m);
                Eigen::Isometry3d t_incr = t_base_geomagic * to_eigen_tf(m) * t_base_geomagic.inverse();
                std::lock_guard<std::mutex> lock(mutex_t_desired);
                t_base_tcp_desired.translation() += t_incr.translation();
                t_base_tcp_desired.linear() = t_incr.linear() * t_base_tcp_desired.linear();
            }
        }, ros::TransportHints().udp().tcp().tcpNoDelay())
    };

    std::thread thread_servo([&]() {
        while (nh.ok()) {
            if (clutched) {
                auto pose_tcp_desired = [&]() {
                    std::lock_guard<std::mutex> lock(mutex_t_desired);
                    return to_ur(t_base_tcp_desired);
                }();

                rtde_ctrl.servoL(pose_tcp_desired, 0, 0, update_period, 0.08, 400);
            }
        }

        rtde_ctrl.servoStop();
    });

    ros::spin();

    thread_servo.join();

    return EXIT_SUCCESS;
}
