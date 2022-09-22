/*********************************************************************
* Software License Agreement (BSD License)
*
*  Copyright (c) 2008, Willow Garage, Inc.
*  All rights reserved.
*
*  Redistribution and use in source and binary forms, with or without
*  modification, are permitted provided that the following conditions
*  are met:
*
*   * Redistributions of source code must retain the above copyright
*     notice, this list of conditions and the following disclaimer.
*   * Redistributions in binary form must reproduce the above
*     copyright notice, this list of conditions and the following
*     disclaimer in the documentation and/or other materials provided
*     with the distribution.
*   * Neither the name of the Willow Garage nor the names of its
*     contributors may be used to endorse or promote products derived
*     from this software without specific prior written permission.
*
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
*  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
*  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
*  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
*  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
*  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
*  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
*  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
*  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
*  POSSIBILITY OF SUCH DAMAGE.
*********************************************************************/

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "mops_msgs/msg/tool_end_effector_state_stamped.hpp"
#include "std_msgs/msg/string.hpp"
#include "tf2_ros/static_transform_broadcaster.h"
#include "tf2_ros/transform_broadcaster.h"
#include "kdl_parser/kdl_parser.hpp"
#include "kdl/chainfksolverpos_recursive.hpp"
#include "kdl/tree.hpp"
#include "urdf/model.h"
#include "rclcpp/rclcpp.hpp"

#include <unordered_map>

// TODO: Refactor so that RobotStatePublisher and PoseGraspPublisher utilizes the
// same forward kinematics computations (transforms from each link to the next).

using namespace std::string_literals;
using namespace std::chrono_literals;

template<typename T>
auto sec_to_dur(T seconds) {
    return std::chrono::duration<T, std::ratio<1>>(seconds);
}

geometry_msgs::msg::Pose kdl_to_pose(const KDL::Frame& tf)
{
    geometry_msgs::msg::Pose m;
    m.position.x = tf.p.x();
    m.position.y = tf.p.y();
    m.position.z = tf.p.z();
    tf.M.GetQuaternion(m.orientation.x, m.orientation.y, m.orientation.z, m.orientation.w);
    return m;
}

geometry_msgs::msg::Transform kdl_to_transform(const KDL::Frame& tf)
{
  geometry_msgs::msg::Transform m;
  m.translation.x = tf.p.x();
  m.translation.y = tf.p.y();
  m.translation.z = tf.p.z();
  tf.M.GetQuaternion(m.rotation.x, m.rotation.y, m.rotation.z, m.rotation.w);
  return m;
}

std::string slashStripped(const std::string& s)
{
    if (!s.empty() && s.front() == '/')
        return s.substr(1);

    return s;
}

// Compute yaw0 joint state and append it to m
void appendYaw0(const std::string& prefix, sensor_msgs::msg::JointState& m)
{
    // Find indices of yaw1 and yaw2 joints
    std::ptrdiff_t j = -1;
    std::ptrdiff_t k = -1;

    for (std::ptrdiff_t i = m.name.size() - 1; i >= 0; --i) { // expect yaw1 and yaw2 at end
        if ((j == -1) && (m.name[i].rfind("yaw1") != std::string::npos))
            j = i;
        else if ((k == -1) && (m.name[i].rfind("yaw2") != std::string::npos))
            k = i;
    }

    if (j != -1 && k != -1) {
        // yaw0 angle is between yaw1 and yaw2
        m.name.push_back(prefix + "yaw0");
        m.position.push_back((m.position[j] - m.position[k]) / 2);
    }
};

struct SegmentData
{
    SegmentData(const KDL::Segment& segment, const std::string& root, const std::string& tip)
        : segment(segment)
        , root(slashStripped(root))
        , tip(slashStripped(tip))
        , q(0)
        , stamp(0, 0, RCL_ROS_TIME)
    {}

    KDL::Segment segment;
    std::string root;
    std::string tip;
    double q;
    rclcpp::Time stamp; // last time q was updated

    geometry_msgs::msg::TransformStamped transform() const
    {
        geometry_msgs::msg::TransformStamped tf;
        tf.transform = kdl_to_transform(segment.pose(q));
        tf.header.stamp = stamp;
        tf.header.frame_id = root;
        tf.child_frame_id = tip;
        return tf;
    }
};

class RobotStatePublisher
{
public:
    explicit RobotStatePublisher(rclcpp::Node::SharedPtr node, const KDL::Tree& tree)
        : tf_broadcaster_(node)
        , static_tf_broadcaster_(node)
    {
        addChildren(tree.getRootSegment());
    }

    void updateJointPositions(const sensor_msgs::msg::JointState& m)
    {
        for (std::size_t i = 0; i < m.name.size(); ++i) {
            auto& sd = segments_moving_.at(m.name[i]);
            sd.stamp = m.header.stamp;
            sd.q = m.position[i];
        }
    }

    void publishTransforms(rclcpp::Node::SharedPtr node)
    {
        std::vector<geometry_msgs::msg::TransformStamped> transforms;
        transforms.reserve(segments_moving_.size());
        auto now = node->now();

        for (auto const& [name, sd] : segments_moving_)
            // Don't publish old data
            if ((now - sd.stamp) < 1s)
                transforms.push_back(sd.transform());

        tf_broadcaster_.sendTransform(transforms);
    }

    void publishFixedTransforms()
    {
        std::vector<geometry_msgs::msg::TransformStamped> transforms;
        transforms.reserve(segments_fixed_.size());

        for (auto const& [name, sd] : segments_fixed_)
            transforms.push_back(sd.transform());

        static_tf_broadcaster_.sendTransform(transforms);
    }

private:
    void addChildren(KDL::SegmentMap::const_iterator root_it)
    {
        for (const auto& child_it : root_it->second.children) {
            const auto& seg = child_it->second.segment;

            if (seg.getJoint().getType() == KDL::Joint::None)
                segments_fixed_.insert({seg.getJoint().getName(), {seg, root_it->second.segment.getName(), seg.getName()}});
            else
                segments_moving_.insert({seg.getJoint().getName(), {seg, root_it->second.segment.getName(), seg.getName()}});

            addChildren(child_it);
        }
    }

private:
    tf2_ros::TransformBroadcaster tf_broadcaster_;
    tf2_ros::StaticTransformBroadcaster static_tf_broadcaster_;
    std::unordered_map<std::string, SegmentData> segments_moving_;
    std::unordered_map<std::string, SegmentData> segments_fixed_;
};

struct PoseGraspPublisher
{
public:
    PoseGraspPublisher(rclcpp::Node::SharedPtr node, const std::string& id, const KDL::Tree& tree)
        : root_(id + "_ur_base_link")
        , chain_(initChain(root_, id + "_tool_tcp0", tree))
        , fk_solver_(chain_)
        , q_current_(chain_.getNrOfJoints())
        , q_current_yaw_{}
        , last_update_(0, 0, RCL_ROS_TIME)
    {
        unsigned i = 0;

        // Map to q_current by joint name
        for (const auto& seg : chain_.segments)
            if (seg.getJoint().getType() != KDL::Joint::None)
                q_current_index_.insert({seg.getJoint().getName(), &q_current_(i++)});

        // Also map grasper joint states (yaw1, yaw2) that are not part of chain
        q_current_index_.insert({id + "_tool_yaw1", &q_current_yaw_[0]});
        q_current_index_.insert({id + "_tool_yaw2", &q_current_yaw_[1]});

        pub_ = node->create_publisher<mops_msgs::msg::ToolEndEffectorStateStamped>("/" + id + "/ee_state_current", rclcpp::SensorDataQoS());
    }

    void updateJointPositions(const sensor_msgs::msg::JointState& m)
    {
        for (std::size_t i = 0; i < m.name.size(); ++i)
            *q_current_index_.at(m.name[i]) = m.position[i];

        last_update_ = m.header.stamp;
    }

    void publish(rclcpp::Node::SharedPtr node)
    {
        auto now = node->now();

        // Don't publish old data
        if ((now - last_update_) > 1s)
            return;

        KDL::Frame tf;

        if (auto err = fk_solver_.JntToCart(q_current_, tf); err != KDL::ChainFkSolverPos_recursive::E_NOERROR)
            throw std::runtime_error("FK failed with error: "s + fk_solver_.strError(err));

        mops_msgs::msg::ToolEndEffectorStateStamped m;
        m.header.stamp = now;
        m.header.frame_id = root_;
        m.ee.pose = kdl_to_pose(tf);
        m.ee.grasper_angle = q_current_yaw_[0] + q_current_yaw_[1];
        pub_->publish(m);
    }

private:
    static KDL::Chain initChain(const std::string& root,
                                const std::string& tip,
                                const KDL::Tree& tree)
    {
        KDL::Chain chain;

        if (!tree.getChain(root, tip, chain))
            throw std::runtime_error("Failed to get kinematic chain between links: " + root + ", " + tip);

        return chain;
    }

private:
    std::string root_;
    KDL::Chain chain_;
    KDL::ChainFkSolverPos_recursive fk_solver_;
    KDL::JntArray q_current_;
    std::array<double, 2> q_current_yaw_;
    std::unordered_map<std::string, double*> q_current_index_;
    rclcpp::Time last_update_;
    rclcpp::Publisher<mops_msgs::msg::ToolEndEffectorStateStamped>::SharedPtr pub_;
};

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<rclcpp::Node>("mops_state_publisher");
    auto urdf_xml = node->declare_parameter("robot_description", ""s);

    if (urdf_xml.empty())
        throw std::runtime_error("The 'robot_description' parameter is empty");

    urdf::Model model;

    if (!model.initString(urdf_xml))
        throw std::runtime_error("Failed to initialize urdf::Model");

    KDL::Tree tree;

    if (!kdl_parser::treeFromUrdfModel(model, tree))
        throw std::runtime_error("Failed to construct KDL tree from URDF model");

    RobotStatePublisher robot_state_publisher(node, tree);
    PoseGraspPublisher pose_pub_a(node, "a", tree);
    PoseGraspPublisher pose_pub_b(node, "b", tree);

    auto pub_descr = node->create_publisher<std_msgs::msg::String>("robot_description", rclcpp::QoS(1).transient_local());

    std::list<rclcpp::SubscriptionBase::SharedPtr> subscribers {
        node->create_subscription<sensor_msgs::msg::JointState>(
            "/a/ur/joint_states",
            rclcpp::SensorDataQoS(),
            [&](const sensor_msgs::msg::JointState& m) {
                robot_state_publisher.updateJointPositions(m);
                pose_pub_a.updateJointPositions(m);
            }),
        node->create_subscription<sensor_msgs::msg::JointState>(
            "/a/tool/joint_states",
            rclcpp::SensorDataQoS(),
            [&](sensor_msgs::msg::JointState m) {
                appendYaw0("a_tool_", m);
                robot_state_publisher.updateJointPositions(m);
                pose_pub_a.updateJointPositions(m);
            }),
        node->create_subscription<sensor_msgs::msg::JointState>(
            "/b/ur/joint_states",
            rclcpp::SensorDataQoS(),
            [&](const sensor_msgs::msg::JointState& m) {
                robot_state_publisher.updateJointPositions(m);
                pose_pub_b.updateJointPositions(m);
            }),
        node->create_subscription<sensor_msgs::msg::JointState>(
            "/b/tool/joint_states",
            rclcpp::SensorDataQoS(),
            [&](sensor_msgs::msg::JointState m) {
                appendYaw0("b_tool_", m);
                robot_state_publisher.updateJointPositions(m);
                pose_pub_b.updateJointPositions(m);
            }),
    };

    auto ee_publish_frequency = node->declare_parameter("ee_publish_frequency", 125.0);
    std::list<rclcpp::TimerBase::SharedPtr> timers {
        node->create_wall_timer(
            sec_to_dur(1.0 / node->declare_parameter("tf_publish_frequency", 50.0)),
            [&robot_state_publisher, node]() { robot_state_publisher.publishTransforms(node); }
        ),
        node->create_wall_timer(
            sec_to_dur(1.0 / ee_publish_frequency),
            [&pose_pub_a, node]() { pose_pub_a.publish(node); }
        ),
        node->create_wall_timer(
            sec_to_dur(1.0 / ee_publish_frequency),
            [&pose_pub_b, node]() { pose_pub_b.publish(node); }
        ),
    };

    auto msg = std::make_unique<std_msgs::msg::String>();
    msg->data = urdf_xml;
    pub_descr->publish(std::move(msg));

    robot_state_publisher.publishFixedTransforms();

    rclcpp::spin(node);
    rclcpp::shutdown();
    return EXIT_SUCCESS;
}
