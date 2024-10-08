#pragma once

#include "conversions.hpp"

#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/transform.hpp"

#include <kdl/frames.hpp>

// KDL -> ROS
template<>
geometry_msgs::msg::Point convert_to(const KDL::Vector& v)
{
    geometry_msgs::msg::Point m;
    m.x = v.x();
    m.y = v.y();
    m.z = v.z();
    return m;
}

template<>
geometry_msgs::msg::Vector3 convert_to(const KDL::Vector& v)
{
    geometry_msgs::msg::Vector3 m;
    m.x = v.x();
    m.y = v.y();
    m.z = v.z();
    return m;
}

template<>
geometry_msgs::msg::Quaternion convert_to(const KDL::Rotation& rot)
{
    geometry_msgs::msg::Quaternion m;
    rot.GetQuaternion(m.x, m.y, m.z, m.w);
    return m;
}

template<>
geometry_msgs::msg::Pose convert_to(const KDL::Frame& tf)
{
    geometry_msgs::msg::Pose m;
    m.position = convert_to<geometry_msgs::msg::Point>(tf.p);
    m.orientation = convert_to<geometry_msgs::msg::Quaternion>(tf.M);
    return m;
}

template<>
geometry_msgs::msg::Transform convert_to(const KDL::Frame& tf)
{
    geometry_msgs::msg::Transform m;
    m.translation = convert_to<geometry_msgs::msg::Vector3>(tf.p);
    m.rotation = convert_to<geometry_msgs::msg::Quaternion>(tf.M);
    return m;
}

// ROS -> KDL
template<>
KDL::Vector convert_to(const geometry_msgs::msg::Point& m)
{
    return {m.x, m.y, m.z};
}

template<>
KDL::Rotation convert_to(const geometry_msgs::msg::Quaternion& m)
{
    return KDL::Rotation::Quaternion(m.x, m.y, m.z, m.w);
}

template<>
KDL::Frame convert_to(const geometry_msgs::msg::Pose& m)
{
    return {convert_to<KDL::Rotation>(m.orientation), convert_to<KDL::Vector>(m.position)};
}
