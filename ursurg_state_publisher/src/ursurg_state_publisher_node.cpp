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

#include <ursurg_common/conversions/kdl.h>
#include <ursurg_common/rosutility/subscription.h>

#include <geometry_msgs/PoseStamped.h>
#include <kdl_parser/kdl_parser.hpp>
#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <tf2_kdl/tf2_kdl.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_ros/transform_broadcaster.h>

#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/tree.hpp>
#include <urdf/model.h>

#include <unordered_map>

// TODO: Refactor so that RobotStatePublisher and PoseGraspPublisher utilizes the
// same forward kinematics computations (transforms from each link to the next).

using namespace std::string_literals;

std::string slashStripped(const std::string& s)
{
    if (!s.empty() && s.front() == '/')
        return s.substr(1);

    return s;
}

struct SegmentData
{
    SegmentData(const KDL::Segment& segment, const std::string& root, const std::string& tip)
        : segment(segment)
        , root(slashStripped(root))
        , tip(slashStripped(tip))
        , q(0)
    {}

    KDL::Segment segment;
    std::string root;
    std::string tip;
    double q;
    ros::Time stamp; // last time q was updated

    geometry_msgs::TransformStamped transform() const
    {
        auto tf = tf2::kdlToTransform(segment.pose(q));
        tf.header.stamp = stamp;
        tf.header.frame_id = root;
        tf.child_frame_id = tip;
        return tf;
    }
};

class RobotStatePublisher
{
public:
    explicit RobotStatePublisher(const KDL::Tree& tree)
    {
        addChildren(tree.getRootSegment());
    }

    void updateJointPositions(const sensor_msgs::JointState& m)
    {
        for (std::size_t i = 0; i < m.name.size(); ++i) {
            auto& sd = segments_moving_.at(m.name[i]);
            sd.stamp = m.header.stamp;
            sd.q = m.position[i];
        }
    }

    void publishTransforms(const ros::TimerEvent& e)
    {
        std::vector<geometry_msgs::TransformStamped> transforms;
        transforms.reserve(segments_moving_.size());

        for (auto const& [name, sd] : segments_moving_)
            // Don't publish old data
            if ((e.current_real - sd.stamp) < ros::Duration(1))
                transforms.push_back(sd.transform());

        tf_broadcaster_.sendTransform(transforms);
    }

    void publishFixedTransforms()
    {
        std::vector<geometry_msgs::TransformStamped> transforms;
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

    std::unordered_map<std::string, SegmentData> segments_moving_;
    std::unordered_map<std::string, SegmentData> segments_fixed_;
    tf2_ros::TransformBroadcaster tf_broadcaster_;
    tf2_ros::StaticTransformBroadcaster static_tf_broadcaster_;
};

struct PoseGraspPublisher
{
public:
    PoseGraspPublisher(const std::string& id, const KDL::Tree& tree)
        : root_(id + "_ur_base_link")
        , chain_(initChain(root_, id + "_tool_tcp0", tree))
        , fk_solver_(chain_)
        , q_current_(chain_.getNrOfJoints())
        , q_current_yaw_{}
    {
        unsigned i = 0;

        // Map to q_current by joint name
        for (const auto& seg : chain_.segments)
            if (seg.getJoint().getType() != KDL::Joint::None)
                q_current_index_.insert({seg.getJoint().getName(), &q_current_(i++)});

        // Also map grasper joint states (yaw1, yaw2) that are not part of chain
        q_current_index_.insert({id + "_tool_yaw1", &q_current_yaw_[0]});
        q_current_index_.insert({id + "_tool_yaw2", &q_current_yaw_[1]});

        grasper_angle_name_ = id + "_tool_grasper_angle";

        pub_pose_ = nh_.advertise<geometry_msgs::PoseStamped>("/" + id + "/tcp_pose_current", 1);
        pub_grasp_ = nh_.advertise<sensor_msgs::JointState>("/" + id + "/grasp_current", 1);
    }

    void updateJointPositions(const sensor_msgs::JointState& m)
    {
        for (std::size_t i = 0; i < m.name.size(); ++i)
            *q_current_index_.at(m.name[i]) = m.position[i];

        last_update_ = m.header.stamp;
    }

    void publish(const ros::TimerEvent& e)
    {
        // Don't publish old data
        if ((e.current_real - last_update_) > ros::Duration(1))
            return;

        KDL::Frame tf;
        auto err = fk_solver_.JntToCart(q_current_, tf);

        if (err != KDL::SolverI::E_NOERROR)
            throw std::runtime_error("FK failed with error: "s + fk_solver_.strError(err));

        geometry_msgs::PoseStamped m_pose;
        m_pose.header.stamp = e.current_real;
        m_pose.header.frame_id = root_;
        m_pose.pose = convert_to<geometry_msgs::Pose>(tf);
        pub_pose_.publish(m_pose);

        sensor_msgs::JointState m_grasp;
        m_grasp.header.stamp = e.current_real;
        m_grasp.name.push_back(grasper_angle_name_);
        m_grasp.position.push_back(q_current_yaw_[0] + q_current_yaw_[1]);
        pub_grasp_.publish(m_grasp);
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
    ros::Time last_update_;
    ros::NodeHandle nh_;
    ros::Publisher pub_pose_;
    ros::Publisher pub_grasp_;
    std::string grasper_angle_name_;
};

int main(int argc, char* argv[])
{
    ros::init(argc, argv, "ursurg_state_publisher");
    ros::NodeHandle nh;
    ros::NodeHandle nh_priv("~");

    urdf::Model model;

    if (!model.initParam("robot_description"))
        throw std::runtime_error("Failed to load URDF model from parameter server");

    KDL::Tree tree;

    if (!kdl_parser::treeFromUrdfModel(model, tree))
        throw std::runtime_error("Failed to construct KDL tree from URDF model");

    RobotStatePublisher robot_state_publisher(tree);
    PoseGraspPublisher pose_pub_a("a", tree);
    PoseGraspPublisher pose_pub_b("b", tree);

    std::list<ros::Subscriber> subscribers = {
        mksub<sensor_msgs::JointState>(
            nh, "/a/ur/joint_states", 1, [&](const auto& m) {
                robot_state_publisher.updateJointPositions(m);
                pose_pub_a.updateJointPositions(m);
            }, ros::TransportHints().udp().tcp().tcpNoDelay()),
        mksub<sensor_msgs::JointState>(
            nh, "/a/tool/joint_states", 1, [&](auto m) {
                // Find indices of yaw1 and yaw2 joints
                std::ptrdiff_t j = -1;
                std::ptrdiff_t k = -1;

                for (std::size_t i = 0; i < m.name.size(); ++i) {
                    if (m.name[i].rfind("yaw1") != std::string::npos)
                        j = i;
                    else if (m.name[i].rfind("yaw2") != std::string::npos)
                        k = i;
                }

                // Append yaw0 joint configuration between yaw1 and yaw2
                if (j != -1 && k != -1) {
                    m.name.push_back("a_tool_yaw0");
                    m.position.push_back((m.position[j] - m.position[k]) / 2);
                }

                robot_state_publisher.updateJointPositions(m);
                pose_pub_a.updateJointPositions(m);
            }, ros::TransportHints().udp().tcp().tcpNoDelay()),
        mksub<sensor_msgs::JointState>(
            nh, "/b/ur/joint_states", 1, [&](const auto& m) {
                robot_state_publisher.updateJointPositions(m);
                pose_pub_b.updateJointPositions(m);
            }, ros::TransportHints().udp().tcp().tcpNoDelay()),
        mksub<sensor_msgs::JointState>(
            nh, "/b/tool/joint_states", 1, [&](auto m) {
                // Find indices of yaw1 and yaw2 joints
                std::ptrdiff_t j = -1;
                std::ptrdiff_t k = -1;

                for (std::size_t i = 0; i < m.name.size(); ++i) {
                    if (m.name[i].rfind("yaw1") != std::string::npos)
                        j = i;
                    else if (m.name[i].rfind("yaw2") != std::string::npos)
                        k = i;
                }

                // Append yaw0 joint configuration between yaw1 and yaw2
                if (j != -1 && k != -1) {
                    m.name.push_back("b_tool_yaw0");
                    m.position.push_back((m.position[j] - m.position[k]) / 2);
                }

                robot_state_publisher.updateJointPositions(m);
                pose_pub_b.updateJointPositions(m);
            }, ros::TransportHints().udp().tcp().tcpNoDelay()),
    };

    std::list<ros::Timer> timers = {
        nh.createTimer(ros::Duration(1.0 / nh_priv.param("tf_publish_frequency", 50.0)),
                       &RobotStatePublisher::publishTransforms,
                       &robot_state_publisher),
        nh.createTimer(ros::Duration(1.0 / nh_priv.param("pose_publish_frequency", 125.0)),
                       &PoseGraspPublisher::publish,
                       &pose_pub_a),
        nh.createTimer(ros::Duration(1.0 / nh_priv.param("pose_publish_frequency", 125.0)),
                       &PoseGraspPublisher::publish,
                       &pose_pub_b),
    };

    robot_state_publisher.publishFixedTransforms();
    ros::spin();
    return EXIT_SUCCESS;
}
