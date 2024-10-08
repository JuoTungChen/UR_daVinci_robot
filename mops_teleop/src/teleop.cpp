﻿#include "mops_common/conversions/eigen.hpp"
#include "mops_common/mathutility.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/float64.hpp"
#include "touch_msgs/msg/button_event.hpp"
#include "mops_msgs/msg/tool_end_effector_state_stamped.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include "rclcpp/rclcpp.hpp"

#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/quaternion.hpp>

#include <Eigen/Geometry>

#include <chrono>
#include <thread>

using namespace std::chrono_literals;
using namespace std::string_literals;

namespace mops_teleop {

template<typename T>
auto sec_to_dur(T seconds) {
    return std::chrono::duration<T, std::ratio<1>>(seconds);
}

class TeleopNode : public rclcpp::Node
{
public:
    explicit TeleopNode(const rclcpp::NodeOptions& options)
        : rclcpp::Node("teleop", options)
        , init_current_(false)
        , clutch_engaged_(false)
        , rotation_tracking_engaged_(false)
        , t_robotbase_robottcp_current_(Eigen::Isometry3d::Identity())
        , t_robotbase_robottcp_desired_(t_robotbase_robottcp_current_)
        // , t_robotbase_robottcp_desired_new(t_robotbase_robottcp_current_)
        , t_robotbase_haptictcp_current_(Eigen::Isometry3d::Identity())
        , t_robotbase_haptictcp_last_(t_robotbase_haptictcp_current_)
        , t_robotbase_hapticbase_(Eigen::Isometry3d::Identity())
        , buttons_({false, false})
        , grasp_current_(math::radians(60))
        , grasp_desired_(grasp_current_)
        , grasp_rate_(math::pi / 4)
    {
        auto robot_base = declare_parameter("ur_prefix", ""s) + "base_link";
        auto haptic_base = declare_parameter("haptic_prefix", ""s) + "base";

        t_robotbase_hapticbase_ = [&]() {
            tf2_ros::Buffer tf_buffer(get_clock());
            tf2_ros::TransformListener tf_listener(tf_buffer);
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

        pub_ee_desired_ = create_publisher<mops_msgs::msg::ToolEndEffectorState>("ee_state_desired", 1);
        pub_t_incr_ = create_publisher<geometry_msgs::msg::Pose>("haptic_stylus_increment", 1);

        subscribers_ = {
            create_subscription<std_msgs::msg::Bool>(
                "clutch_engaged",
                10,
                [this](std_msgs::msg::Bool::ConstSharedPtr m) {
                    // Initially: desired <- current
                    if (m->data) {
                        t_robotbase_robottcp_desired_ = t_robotbase_robottcp_current_;
                        // t_robotbase_robottcp_desired_new = t_robotbase_robottcp_current_;
                        // FIXME: this is not nice because grasper angle computed
                        // from kinematics is quite inaccurate
                        grasp_desired_ = grasp_current_;
                        clutch_engaged_ = true;
                    } else {
                        clutch_engaged_ = false;
                    }
                }
            ),
            create_subscription<std_msgs::msg::Bool>(
                "rotation_tracking_engaged",
                10,
                [this](std_msgs::msg::Bool::ConstSharedPtr m) {
                    // Initially: desired <- current
                    if (m->data) {
                        // t_robotbase_robottcp_desired_.rotate(t_robotbase_robottcp_current_.rotation());
                        t_robotbase_robottcp_desired_ = t_robotbase_robottcp_current_;
                        // t_robotbase_robottcp_desired_new = t_robotbase_robottcp_current_;

                        rotation_tracking_engaged_ = true;
                    } else {
                        rotation_tracking_engaged_ = false;
                    }
                }
            ),
            create_subscription<mops_msgs::msg::ToolEndEffectorStateStamped>(
                "ee_state_current",
                rclcpp::QoS(1).best_effort(),
                [this](mops_msgs::msg::ToolEndEffectorStateStamped::ConstSharedPtr m) {
                    // Cache the most recent end-effector state computed from forward kinematics
                    t_robotbase_robottcp_current_ = convert_to<Eigen::Isometry3d>(m->ee.pose);
                    grasp_current_ = m->ee.grasper_angle;
                    init_current_ = true;
                }
            ),
            create_subscription<geometry_msgs::msg::PoseStamped>(
                "/omega_7__right_hand_00/measured_cp",
                rclcpp::QoS(1).best_effort(),
                [this](geometry_msgs::msg::PoseStamped::ConstSharedPtr m) {
                    t_robotbase_haptictcp_last_ = std::move(t_robotbase_haptictcp_current_);

                    // Transform incoming haptic TCP poses seen wrt. haptic base
                    // frame to the robot base coordinate system
                    t_robotbase_haptictcp_current_ = convert_to<Eigen::Isometry3d>(m->pose);
                    // t_robotbase_haptictcp_current_ = t_robotbase_hapticbase_ * convert_to<Eigen::Isometry3d>(m->pose);
                    // if (rotation_tracking_engaged_){
                    //     t_robotbase_robottcp_desired_.rotate(t_robotbase_haptictcp_current_.rotation());

                    //     RCLCPP_INFO(this->get_logger(), "orientation justified");

                    // }
                    if (clutch_engaged_) {
                        // Transform from the previous to the current haptic pose
                        Eigen::Isometry3d t_incr = t_robotbase_haptictcp_last_.inverse() * t_robotbase_haptictcp_current_;

                        // "added to" the desired robot TCP pose
                        // t_robotbase_robottcp_desired_ = t_robotbase_robottcp_desired_ * t_incr;
                        // t_robotbase_robottcp_desired_new = t_incr * t_robotbase_robottcp_desired_new;

                        // Extract translation and rotation components
                        Eigen::Vector3d translation = t_robotbase_haptictcp_current_.translation() - t_robotbase_haptictcp_last_.translation();
                        // Eigen::Vector3d translation = t_incr.translation();
                        Eigen::Quaterniond rotation(t_incr.rotation());

                        // Add translation of t_incr to the translation of t_robotbase_robottcp_desired_
                        t_robotbase_robottcp_desired_.translation() += translation;

                        // Apply rotation of t_incr directly to the rotation of t_robotbase_robottcp_desired_
                        t_robotbase_robottcp_desired_.rotate(rotation);

                        // Convert Eigen vectors to ROS geometry messages
                        // pose_msg.position.x = translation.x();
                        // pose_msg.position.y = translation.y();
                        // pose_msg.position.z = translation.z();

                        // pose_msg.orientation.x = rotation.x();
                        // pose_msg.orientation.y = rotation.y();
                        // pose_msg.orientation.z = rotation.z();
                        // pose_msg.orientation.w = rotation.w();

                        // Print out the values
                        // RCLCPP_INFO(this->get_logger(), "(%f, %f, %f)", translation_msg.x, translation_msg.y, translation_msg.z);
                        // RCLCPP_INFO("t_incr translation: (%f, %f, %f)", translation_msg.x, translation_msg.y, translation_msg.z);
                        // RCLCPP_INFO("t_incr rotation: (x: %f, y: %f, z: %f, w: %f)", rotation_msg.x, rotation_msg.y, rotation_msg.z, rotation_msg.w);
                    }
                }
            ),
            create_subscription<touch_msgs::msg::ButtonEvent>(
                "haptic_buttons",
                10,
                [this](touch_msgs::msg::ButtonEvent::ConstSharedPtr m) {
                    if (m->button == touch_msgs::msg::ButtonEvent::BUTTON_GRAY)
                        buttons_[0] = (m->event == touch_msgs::msg::ButtonEvent::EVENT_PRESSED);
                    else if (m->button == touch_msgs::msg::ButtonEvent::BUTTON_WHITE)
                        buttons_[1] = (m->event == touch_msgs::msg::ButtonEvent::EVENT_PRESSED);
                }
            ),
            create_subscription<std_msgs::msg::Float64>(
                "grasp_angle",
                10,
                [this](std_msgs::msg::Float64::ConstSharedPtr m) {
                    grasp_desired_ = m->data ;
                }
            ),
        };

        stamp_prev_ = now();

        timer_ = create_wall_timer(
            sec_to_dur(1.0 / declare_parameter("publish_rate", 125.0)),
            [this]() {
                if (!init_current_ || !clutch_engaged_)
                    return;

                auto stamp_now = now();
                auto dt = (stamp_now - stamp_prev_).seconds();
                stamp_prev_ = stamp_now;

                // if (buttons_[0] && !buttons_[1]) {
                //     grasp_desired_ -= grasp_rate_ * dt;
                // } else if (buttons_[1] && !buttons_[0]) {
                //     grasp_desired_ += grasp_rate_ * dt;
                // }

                auto m = std::make_unique<mops_msgs::msg::ToolEndEffectorState>();
                pose_msg = convert_to<geometry_msgs::msg::Pose>(t_robotbase_robottcp_desired_);
                m->pose = convert_to<geometry_msgs::msg::Pose>(t_robotbase_robottcp_desired_);
                m->grasper_angle = grasp_desired_;
                pub_ee_desired_->publish(std::move(m));
                pub_t_incr_->publish(pose_msg);
            }
        );
    }

private:
    bool init_current_;
    bool clutch_engaged_;
    bool rotation_tracking_engaged_;
    Eigen::Isometry3d t_robotbase_robottcp_current_;
    Eigen::Isometry3d t_robotbase_robottcp_desired_;
    // Eigen::Isometry3d t_robotbase_robottcp_desired_new;
    Eigen::Isometry3d t_robotbase_haptictcp_current_;
    Eigen::Isometry3d t_robotbase_haptictcp_last_;
    Eigen::Isometry3d t_robotbase_hapticbase_;
    geometry_msgs::msg::Pose pose_msg;

    std::array<bool, 2> buttons_;
    double grasp_current_;
    double grasp_desired_;
    double grasp_rate_;
    rclcpp::Publisher<mops_msgs::msg::ToolEndEffectorState>::SharedPtr pub_ee_desired_;
    rclcpp::Publisher<geometry_msgs::msg::Pose>::SharedPtr pub_t_incr_;
    std::list<rclcpp::SubscriptionBase::SharedPtr> subscribers_;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Time stamp_prev_;
};

} // namespace mops_teleop

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(mops_teleop::TeleopNode)
