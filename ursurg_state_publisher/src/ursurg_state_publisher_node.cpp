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

#include <kdl_parser/kdl_parser.hpp>
#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <tf2_kdl/tf2_kdl.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_ros/transform_broadcaster.h>

#include <kdl/tree.hpp>
#include <urdf/model.h>

#include <unordered_map>

std::string slashStripped(const std::string& s)
{
    if ((s.size() >= 1) && s[0] == '/')
        return s.substr(1);

    return s;
}

class SegmentPair
{
public:
    SegmentPair(const KDL::Segment& segment, const std::string& root, const std::string& tip)
        : segment(segment)
        , root(slashStripped(root))
        , tip(slashStripped(tip))
    {}

    KDL::Segment segment;
    std::string root;
    std::string tip;
};

class RobotStatePublisher
{
public:
    RobotStatePublisher()
    {
        if (!model_.initParam("robot_description"))
            throw std::runtime_error("Failed to load URDF model from parameter server");

        if (!kdl_parser::treeFromUrdfModel(model_, tree_))
            throw std::runtime_error("Failed to construct KDL tree from URDF model");

        addChildren(tree_.getRootSegment());
    }

    void updateJointPositions(const sensor_msgs::JointState& m)
    {
        if (m.name.size() != m.position.size()) {
            ROS_WARN("Ignored an invalid JointState message");
            return;
        }

        for (std::size_t i = 0; i < m.name.size(); ++i) {
            joint_positions_[m.name[i]] = m.position[i];
        }
    }

    void publishTransforms(const ros::TimerEvent& e)
    {
        std::vector<geometry_msgs::TransformStamped> transforms;

        for (auto const& [name, pos] : joint_positions_) {
            auto sp = segments_.find(name);

            if (sp != segments_.end()) {
                auto tf = tf2::kdlToTransform(sp->second.segment.pose(pos));
                tf.header.stamp = e.current_real;
                tf.header.frame_id = sp->second.root;
                tf.child_frame_id = sp->second.tip;
                transforms.push_back(tf);
            } else {
                ROS_WARN_THROTTLE(10, "State for joint named '%s' was received but joint is not in URDF model", name.c_str());
            }
        }

        tf_broadcaster_.sendTransform(transforms);
    }

    void publishFixedTransforms()
    {
        std::vector<geometry_msgs::TransformStamped> transforms;

        for (auto const& [name, sp] : segments_fixed_) {
            auto tf = tf2::kdlToTransform(sp.segment.pose(0));
            tf.header.stamp = ros::Time::now();
            tf.header.frame_id = sp.root;
            tf.child_frame_id = sp.tip;
            transforms.push_back(tf);
        }

        static_tf_broadcaster_.sendTransform(transforms);
    }

private:
    void addChildren(const KDL::SegmentMap::const_iterator segment)
    {
        const auto& root_name = segment->second.segment.getName();

        for (const auto& child_it : segment->second.children) {
            const auto& seg = child_it->second.segment;
            const auto& jnt = seg.getJoint();

            if (jnt.getType() == KDL::Joint::None) {
                if (model_.getJoint(jnt.getName()) && model_.getJoint(jnt.getName())->type == urdf::Joint::FLOATING) {
                    ROS_INFO("Floating joint. Not adding segment from %s to %s. This TF can not be published based on joint_states info", root_name.c_str(), seg.getName().c_str());
                } else {
                    segments_fixed_.insert({jnt.getName(), SegmentPair(seg, root_name, seg.getName())});
                    ROS_DEBUG("Adding fixed segment from %s to %s", root_name.c_str(), seg.getName().c_str());
                }
            } else {
                segments_.insert({jnt.getName(), SegmentPair(seg, root_name, seg.getName())});
                ROS_DEBUG("Adding moving segment from %s to %s", root_name.c_str(), seg.getName().c_str());
            }

            addChildren(child_it);
        }
    }

    urdf::Model model_;
    KDL::Tree tree_;
    std::unordered_map<std::string, SegmentPair> segments_;
    std::unordered_map<std::string, SegmentPair> segments_fixed_;
    tf2_ros::TransformBroadcaster tf_broadcaster_;
    tf2_ros::StaticTransformBroadcaster static_tf_broadcaster_;
    std::unordered_map<std::string, double> joint_positions_;
};

int main(int argc, char* argv[])
{
    ros::init(argc, argv, "ursurg_state_publisher");
    ros::NodeHandle nh;
    ros::NodeHandle nh_priv("~");
    RobotStatePublisher robot_state_publisher;
    std::list<ros::Subscriber> joint_state_subscribers;

    // Can receive joint state messages from multiple sources
    for (const auto& topic : nh_priv.param<std::vector<std::string>>("joint_state_topics", {})) {
        joint_state_subscribers.push_back(nh.subscribe(topic,
                                                       1,
                                                       &RobotStatePublisher::updateJointPositions,
                                                       &robot_state_publisher,
                                                       ros::TransportHints().udp().tcp().tcpNoDelay()));
    }

    auto timer = nh.createTimer(ros::Duration(1.0 / nh_priv.param("publish_frequency", 50.0)),
                                &RobotStatePublisher::publishTransforms,
                                &robot_state_publisher);
    ros::spin();
    return EXIT_SUCCESS;
}
