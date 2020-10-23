#include <ursurg_common/conversions/eigen.h>
#include <ursurg_common/math.h>
#include <ursurg_common/rosutility.h>

#include <geometry_msgs/PoseStamped.h>
#include <sensor_msgs/JointState.h>
#include <std_msgs/Bool.h>
#include <touch_msgs/ButtonEvent.h>

#include <ros/ros.h>
#include <tf2_ros/transform_listener.h>

#include <Eigen/Geometry>

#include <chrono>
#include <thread>

int main(int argc, char* argv[])
{
    using namespace std::chrono_literals;
    using namespace std::string_literals;

    ros::init(argc, argv, "teleop");
    ros::NodeHandle nh;
    ros::NodeHandle nh_priv("~");

    bool clutch_engaged = false;
    std::string pose_tcp_frame_id;
    Eigen::Isometry3d t_robotbase_robottcp_current = Eigen::Isometry3d::Identity();
    Eigen::Isometry3d t_robotbase_robottcp_desired = t_robotbase_robottcp_current;
    Eigen::Isometry3d t_robotbase_haptictcp_current = Eigen::Isometry3d::Identity();
    Eigen::Isometry3d t_robotbase_haptictcp_last = t_robotbase_haptictcp_current;
    std::array<bool, 2> buttons = {false, false};
    double grasp_current = math::radians(45);
    double grasp_desired = grasp_current;

    // FIXME assumes 'robot base' to 'haptic base' doesn't change while program is running
    auto t_robotbase_hapticbase = [&]() {
        tf2_ros::Buffer tf_buffer;
        tf2_ros::TransformListener tf_listener(tf_buffer);
        auto robot_base = nh_priv.param("ur_prefix", ""s) + "base_link";
        auto haptic_base = nh_priv.param("haptic_prefix", ""s) + "base";
        auto deadline = std::chrono::steady_clock::now() + 5s;

        while (std::chrono::steady_clock::now() < deadline) {
            try {
                auto tf = tf_buffer.lookupTransform(robot_base, haptic_base, ros::Time());
                return convert_to<Eigen::Isometry3d>(tf.transform);
            } catch (const tf2::LookupException&) {
                std::this_thread::sleep_for(200ms);
            }
        }

        throw std::runtime_error("Failed to get transform from '" + robot_base + "' to '" + haptic_base + "'");
    }();

    auto pub_pose_desired = nh.advertise<geometry_msgs::PoseStamped>("tcp_pose_desired", 1);
    auto pub_grasp_desired = nh.advertise<sensor_msgs::JointState>("grasp_desired", 1);

    std::list<ros::Subscriber> subs{
        mksub<std_msgs::Bool>(
            nh, "clutch_engaged", 10, [&](const auto& m) {
                // Initially: desired <- current
                if (m.data) {
                    t_robotbase_robottcp_desired = t_robotbase_robottcp_current;
                    grasp_desired = grasp_current;
                    clutch_engaged = true;
                } else {
                    clutch_engaged = false;
                }
            },
            ros::TransportHints().tcpNoDelay()),
        mksub<geometry_msgs::PoseStamped>(
            nh, "tcp_pose_current", 1, [&](const auto& m) {
                // Cache the most recent robot TCP pose
                t_robotbase_robottcp_current = convert_to<Eigen::Isometry3d>(m.pose);
                pose_tcp_frame_id = m.header.frame_id;
            },
            ros::TransportHints().tcpNoDelay()),
        mksub<sensor_msgs::JointState>(
            nh, "grasp_current", 1, [&](const auto& m) {
                // Cache the most recent opening angle of tool grasper jaws
                grasp_current = m.position.front();
            },
            ros::TransportHints().tcpNoDelay()),
        mksub<geometry_msgs::PoseStamped>(
            nh, "haptic_pose", 1, [&](const auto& m) {
                t_robotbase_haptictcp_last = std::move(t_robotbase_haptictcp_current);

                // Transform incoming haptic TCP poses seen wrt. haptic base
                // frame to the robot base coordinate system
                t_robotbase_haptictcp_current = t_robotbase_hapticbase * convert_to<Eigen::Isometry3d>(m.pose);

                if (clutch_engaged) {
                    // Transform from the previous to the current haptic pose
                    Eigen::Isometry3d t_incr = t_robotbase_haptictcp_last.inverse() * t_robotbase_haptictcp_current;

                    // "added to" the desired robot TCP pose
                    t_robotbase_robottcp_desired = t_robotbase_robottcp_desired * t_incr;
                }
            },
            ros::TransportHints().tcpNoDelay()),
        mksub<touch_msgs::ButtonEvent>(
            nh, "haptic_buttons", 10, [&](const auto& m) {
                if (m.button == touch_msgs::ButtonEvent::BUTTON_GRAY)
                    buttons[0] = (m.event == touch_msgs::ButtonEvent::EVENT_PRESSED);
                else if (m.button == touch_msgs::ButtonEvent::BUTTON_WHITE)
                    buttons[1] = (m.event == touch_msgs::ButtonEvent::EVENT_PRESSED);
            },
            ros::TransportHints().tcpNoDelay()),
    };

    auto timer = nh.createSteadyTimer(
        ros::WallDuration(1.0 / nh_priv.param("publish_rate", 125)),
        [&](const ros::SteadyTimerEvent& e) {
            if (!clutch_engaged)
                return;

            double grasp_rate = math::pi / 4; // TODO: enable setting grasp rate via service call
            auto dt = e.current_real - e.last_real;

            geometry_msgs::PoseStamped m;
            m.header.stamp = ros::Time::now();
            m.header.frame_id = pose_tcp_frame_id;
            m.pose = convert_to<geometry_msgs::Pose>(t_robotbase_robottcp_desired);
            pub_pose_desired.publish(m);

            if (buttons[0] && !buttons[1]) {
                grasp_desired -= grasp_rate * dt.toSec();

                sensor_msgs::JointState m_grasp;
                m_grasp.header.stamp = m.header.stamp;
                m_grasp.position.push_back(grasp_desired);
                pub_grasp_desired.publish(m_grasp);
            } else if (buttons[1] && !buttons[0]) {
                grasp_desired += grasp_rate * dt.toSec();

                sensor_msgs::JointState m_grasp;
                m_grasp.header.stamp = m.header.stamp;
                m_grasp.position.push_back(grasp_desired);
                pub_grasp_desired.publish(m_grasp);
            }
        });

    ros::spin();
    return EXIT_SUCCESS;
}
