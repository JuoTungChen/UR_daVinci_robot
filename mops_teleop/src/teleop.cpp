#include "mops_common/conversions/eigen.hpp"
#include "mops_common/mathutility.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "std_msgs/msg/bool.hpp"
#include "touch_msgs/msg/button_event.hpp"
#include "mops_msgs/msg/tool_end_effector_state_stamped.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include "rclcpp/rclcpp.hpp"

#include <Eigen/Geometry>

#include <chrono>
#include <thread>

template<typename T>
auto sec_to_dur(T seconds) {
    return std::chrono::duration<T, std::ratio<1>>(seconds);
}

int main(int argc, char* argv[])
{
    using namespace std::chrono_literals;
    using namespace std::string_literals;

    rclcpp::init(argc, argv);
    auto node = std::make_shared<rclcpp::Node>("teleop");

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
    double grasp_rate = node->declare_parameter("grasp_rate", math::pi / 4);

    // FIXME assumes 'robot base' to 'haptic base' doesn't change while program is running
    auto t_robotbase_hapticbase = [&]() {
        tf2_ros::Buffer tf_buffer(node->get_clock());
        tf2_ros::TransformListener tf_listener(tf_buffer);
        auto robot_base = node->declare_parameter("ur_prefix", ""s) + "base_link";
        auto haptic_base = node->declare_parameter("haptic_prefix", ""s) + "base";
        auto deadline = std::chrono::steady_clock::now() + 5s;

        while (std::chrono::steady_clock::now() < deadline) {
            try {
                auto tf = tf_buffer.lookupTransform(robot_base, haptic_base, tf2::TimePointZero);
                return convert_to<Eigen::Isometry3d>(tf.transform);
            } catch (const tf2::LookupException&) {
                std::this_thread::sleep_for(200ms);
            }
        }

        throw std::runtime_error("Failed to get transform from '" + robot_base + "' to '" + haptic_base + "'");
    }();

    auto pub_ee_desired = node->create_publisher<mops_msgs::msg::ToolEndEffectorState>("ee_state_desired", 1);

    std::list<rclcpp::SubscriptionBase::SharedPtr> subs{
        node->create_subscription<std_msgs::msg::Bool>(
            "clutch_engaged",
            rclcpp::QoS(rclcpp::KeepLast(10)).transient_local(),
             [&](const std_msgs::msg::Bool& m) {
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
            }),
        node->create_subscription<mops_msgs::msg::ToolEndEffectorStateStamped>(
            "ee_state_current",
            rclcpp::SensorDataQoS(),
            [&](const mops_msgs::msg::ToolEndEffectorStateStamped& m) {
                // Cache the most recent end-effector state computed from forward kinematics
                t_robotbase_robottcp_current = convert_to<Eigen::Isometry3d>(m.ee.pose);
                grasp_current = m.ee.grasper_angle;
                pose_tcp_frame_id = m.header.frame_id;
                init_current = true;
            }),
        node->create_subscription<geometry_msgs::msg::PoseStamped>(
            "haptic_pose",
            rclcpp::SensorDataQoS(),
            [&](const geometry_msgs::msg::PoseStamped& m) {
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
            }),
        node->create_subscription<touch_msgs::msg::ButtonEvent>(
            "haptic_buttons",
            rclcpp::ServicesQoS(),
            [&](const touch_msgs::msg::ButtonEvent& m) {
                if (m.button == touch_msgs::msg::ButtonEvent::BUTTON_GRAY)
                    buttons[0] = (m.event == touch_msgs::msg::ButtonEvent::EVENT_PRESSED);
                else if (m.button == touch_msgs::msg::ButtonEvent::BUTTON_WHITE)
                    buttons[1] = (m.event == touch_msgs::msg::ButtonEvent::EVENT_PRESSED);
            }),
    };

    auto stamp_prev = node->get_clock()->now();
    auto timer = node->create_wall_timer(
        sec_to_dur(1.0 / node->declare_parameter("publish_rate", 125.0)),
        [&]() {
            if (!init_current || !clutch_engaged)
                return;

            auto stamp_now = node->get_clock()->now();
            auto dt = (stamp_now - stamp_prev).seconds();
            stamp_prev = stamp_now;

            if (buttons[0] && !buttons[1]) {
                grasp_desired -= grasp_rate * dt;
            } else if (buttons[1] && !buttons[0]) {
                grasp_desired += grasp_rate * dt;
            }

            mops_msgs::msg::ToolEndEffectorState m;
            m.pose = convert_to<geometry_msgs::msg::Pose>(t_robotbase_robottcp_desired);
            m.grasper_angle = grasp_desired;
            pub_ee_desired->publish(m);
        });

    rclcpp::spin(node);
    rclcpp::shutdown();
    return EXIT_SUCCESS;
}
