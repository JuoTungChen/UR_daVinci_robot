#pragma once

#include "conversions.h"

#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Transform.h>

#include <kdl/frames.hpp>

// KDL -> ROS
template<>
geometry_msgs::Point convert_to(const KDL::Vector& v)
{
    geometry_msgs::Point m;
    m.x = v.x();
    m.y = v.y();
    m.z = v.z();
    return m;
}

template<>
geometry_msgs::Vector3 convert_to(const KDL::Vector& v)
{
    geometry_msgs::Vector3 m;
    m.x = v.x();
    m.y = v.y();
    m.z = v.z();
    return m;
}

template<>
geometry_msgs::Quaternion convert_to(const KDL::Rotation& rot)
{
    geometry_msgs::Quaternion m;
    rot.Quaternion(m.x, m.y, m.z, m.w);
    return m;
}

template<>
geometry_msgs::Pose convert_to(const KDL::Frame& tf)
{
    geometry_msgs::Pose m;
    m.position = convert_to<geometry_msgs::Point>(tf.p);
    m.orientation = convert_to<geometry_msgs::Quaternion>(tf.M);
    return m;
}

template<>
geometry_msgs::Transform convert_to(const KDL::Frame& tf)
{
    geometry_msgs::Transform m;
    m.translation = convert_to<geometry_msgs::Vector3>(tf.p);
    m.rotation = convert_to<geometry_msgs::Quaternion>(tf.M);
    return m;
}

// ROS -> KDL
template<>
KDL::Vector convert_to(const geometry_msgs::Point& m)
{
    return {m.x, m.y, m.z};
}

template<>
KDL::Rotation convert_to(const geometry_msgs::Quaternion& m)
{
    return KDL::Rotation::Quaternion(m.x, m.y, m.z, m.w);
}

template<>
KDL::Frame convert_to(const geometry_msgs::Pose& m)
{
    return {convert_to<KDL::Rotation>(m.orientation), convert_to<KDL::Vector>(m.position)};
}
