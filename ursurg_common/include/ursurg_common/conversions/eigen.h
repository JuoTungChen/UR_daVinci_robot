#pragma once

#include "conversions.h"

#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Transform.h>
#include <geometry_msgs/Twist.h>

#include <Eigen/Geometry>

// Eigen -> ROS
template<>
geometry_msgs::Point convert_to(const Eigen::Vector3d& v)
{
    geometry_msgs::Point m;
    m.x = v[0];
    m.y = v[1];
    m.z = v[2];
    return m;
}

template<>
geometry_msgs::Vector3 convert_to(const Eigen::Vector3d& v)
{
    geometry_msgs::Vector3 m;
    m.x = v[0];
    m.y = v[1];
    m.z = v[2];
    return m;
}

template<>
geometry_msgs::Quaternion convert_to(const Eigen::Quaterniond& q)
{
    geometry_msgs::Quaternion m;
    m.w = q.w();
    m.x = q.x();
    m.y = q.y();
    m.z = q.z();
    return m;
}

template<>
geometry_msgs::Pose convert_to(const Eigen::Isometry3d& tf)
{
    geometry_msgs::Pose m;
    m.position.x = tf.translation().x();
    m.position.y = tf.translation().y();
    m.position.z = tf.translation().z();
    Eigen::Quaterniond q(tf.linear());
    m.orientation.w = q.w();
    m.orientation.x = q.x();
    m.orientation.y = q.y();
    m.orientation.z = q.z();
    return m;
}

template<>
geometry_msgs::Transform convert_to(const Eigen::Isometry3d& tf)
{
    geometry_msgs::Transform m;
    m.translation.x = tf.translation().x();
    m.translation.y = tf.translation().y();
    m.translation.z = tf.translation().z();
    Eigen::Quaterniond q(tf.linear());
    m.rotation.w = q.w();
    m.rotation.x = q.x();
    m.rotation.y = q.y();
    m.rotation.z = q.z();
    return m;
}

// ROS -> Eigen
template<>
Eigen::Vector3d convert_to(const geometry_msgs::Point& m)
{
    return {m.x, m.y, m.z};
}

template<>
Eigen::Translation3d convert_to(const geometry_msgs::Point& m)
{
    return {m.x, m.y, m.z};
}

template<>
Eigen::Quaterniond convert_to(const geometry_msgs::Quaternion& m)
{
    return {m.w, m.x, m.y, m.z};
}

template<>
Eigen::Isometry3d convert_to(const geometry_msgs::Pose& m)
{
    auto p = convert_to<Eigen::Translation3d>(m.position);
    auto q = convert_to<Eigen::Quaterniond>(m.orientation);
    return q * p;
}
