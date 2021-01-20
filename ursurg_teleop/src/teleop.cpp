#include <ursurg_common/conversions/eigen.h>
#include <ursurg_common/math.h>
#include <ursurg_common/rosutility/subscription.h>

#include <geometry_msgs/PoseStamped.h>
#include <std_msgs/Bool.h>
#include <touch_msgs/ButtonEvent.h>
#include <ursurg_msgs/ToolEndEffectorStateStamped.h>

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

    bool init_current = false;
    bool clutch_engaged = false;
    std::string pose_tcp_frame_id;
    Eigen::Isometry3d t_robotbase_robottcp_current = Eigen::Isometry3d::Identity();
    Eigen::Isometry3d t_robotbase_robottcp_desired = t_robotbase_robottcp_current;
    Eigen::Isometry3d t_robotbase_haptictcp_current = Eigen::Isometry3d::Identity();
    Eigen::Isometry3d t_robotbase_haptictcp_last = t_robotbase_haptictcp_current;
    std::array<bool, 2> buttons = {false, false};
    double grasp_current = math::radians(60);
    double grasp_desired = grasp_current;
    double grasp_rate = nh_priv.param("grasp_rate", math::pi / 4);

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

    auto pub_ee_desired = nh.advertise<ursurg_msgs::ToolEndEffectorState>("ee_state_desired", 1);

    std::list<ros::Subscriber> subs{
        mksub<std_msgs::Bool>(
            nh, "clutch_engaged", 10, [&](const auto& m) {
                // Initially: desired <- current
                if (m.data) {
                    t_robotbase_robottcp_desired = t_robotbase_robottcp_current;
                    // FIXME: this is not nice because grasper angle computed
                    // from kinematics is quite inaccurate
                    grasp_desired = grasp_current;
                    clutch_engaged = true;
                } else {
                    clutch_engaged = false;
                }
            },
            ros::TransportHints().tcp().tcpNoDelay()),
        mksub<ursurg_msgs::ToolEndEffectorStateStamped>(
            nh, "ee_state_current", 1, [&](const auto& m) {
                // Cache the most recent end-effector state computed from forward kinematics
                t_robotbase_robottcp_current = convert_to<Eigen::Isometry3d>(m.ee.pose);
                grasp_current = m.ee.grasper_angle;
                pose_tcp_frame_id = m.header.frame_id;
                init_current = true;
            },
            ros::TransportHints().udp().tcp().tcpNoDelay()),
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
            ros::TransportHints().udp().tcp().tcpNoDelay()),
        mksub<touch_msgs::ButtonEvent>(
            nh, "haptic_buttons", 10, [&](const auto& m) {
                if (m.button == touch_msgs::ButtonEvent::BUTTON_GRAY)
                    buttons[0] = (m.event == touch_msgs::ButtonEvent::EVENT_PRESSED);
                else if (m.button == touch_msgs::ButtonEvent::BUTTON_WHITE)
                    buttons[1] = (m.event == touch_msgs::ButtonEvent::EVENT_PRESSED);
            },
            ros::TransportHints().tcp().tcpNoDelay()),
    };

    auto timer = nh.createSteadyTimer(
        ros::WallDuration(1.0 / nh_priv.param("publish_rate", 125)),
        [&](const ros::SteadyTimerEvent& e) {
            if (!init_current || !clutch_engaged)
                return;

            auto dt = e.current_real - e.last_real;

            if (buttons[0] && !buttons[1]) {
                grasp_desired -= grasp_rate * dt.toSec();
            } else if (buttons[1] && !buttons[0]) {
                grasp_desired += grasp_rate * dt.toSec();
            }

            ursurg_msgs::ToolEndEffectorState m;
            m.header.stamp = ros::Time::now();
            m.header.frame_id = pose_tcp_frame_id;
            m.pose = convert_to<geometry_msgs::Pose>(t_robotbase_robottcp_desired);
            m.grasper_angle = grasp_desired;
            pub_ee_desired.publish(m);
        });

    ros::spin();
    return EXIT_SUCCESS;
}
