#pragma once

#include "conversions.h"

#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Transform.h>
#include <geometry_msgs/Twist.h>

#include <rw/math/Quaternion.hpp>
#include <rw/math/Transform3D.hpp>
#include <rw/math/VelocityScrew6D.hpp>

// RobWork -> ROS
template<>
geometry_msgs::Point convert_to(const rw::math::Vector3D<double>& v)
{
    geometry_msgs::Point m;
    m.x = v[0];
    m.y = v[1];
    m.z = v[2];
    return m;
}

template<>
geometry_msgs::Vector3 convert_to(const rw::math::Vector3D<double>& v)
{
    geometry_msgs::Vector3 m;
    m.x = v[0];
    m.y = v[1];
    m.z = v[2];
    return m;
}

template<>
geometry_msgs::Quaternion convert_to(const rw::math::Quaternion<double>& q)
{
    geometry_msgs::Quaternion m;
    m.x = q.getQx();
    m.y = q.getQy();
    m.z = q.getQz();
    m.w = q.getQw();
    return m;
}

template<>
geometry_msgs::Pose convert_to(const rw::math::Transform3D<double>& tf)
{
    geometry_msgs::Pose m;
    m.position = convert_to<geometry_msgs::Point>(tf.P());
    m.orientation = convert_to<geometry_msgs::Quaternion>(tf.R());
    return m;
}

template<>
geometry_msgs::Transform convert_to(const rw::math::Transform3D<double>& tf)
{
    geometry_msgs::Transform m;
    m.translation = convert_to<geometry_msgs::Vector3>(tf.P());
    m.rotation = convert_to<geometry_msgs::Quaternion>(tf.R());
    return m;
}

template<>
geometry_msgs::Twist convert_to(const rw::math::VelocityScrew6D<double>& v)
{
    geometry_msgs::Twist m;
    m.linear.x = v[0];
    m.linear.y = v[1];
    m.linear.z = v[2];
    m.angular.x = v[3];
    m.angular.y = v[4];
    m.angular.z = v[5];
    return m;
}

// ROS -> RobWork
template<>
rw::math::Vector3D<double> convert_to(const geometry_msgs::Point& m)
{
    return {m.x, m.y, m.z};
}

template<>
rw::math::Quaternion<double> convert_to(const geometry_msgs::Quaternion& m)
{
    return {m.x, m.y, m.z, m.w};
}

template<>
rw::math::Transform3D<double> convert_to(const geometry_msgs::Pose& m)
{
    auto p = convert_to<rw::math::Vector3D<double>>(m.position);
    auto q = convert_to<rw::math::Quaternion<double>>(m.orientation);
    return {p, q.toRotation3D()};
}
