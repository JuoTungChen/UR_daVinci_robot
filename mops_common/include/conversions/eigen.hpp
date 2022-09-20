#pragma once

#include "conversions.hpp"

#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/transform.hpp"
#include "geometry_msgs/msg/twist.hpp"

#include <Eigen/Geometry>

// Eigen -> ROS
template<>
geometry_msgs::msg::Point convert_to(const Eigen::Vector3d& v)
{
    geometry_msgs::msg::Point m;
    m.x = v[0];
    m.y = v[1];
    m.z = v[2];
    return m;
}

template<>
geometry_msgs::msg::Vector3 convert_to(const Eigen::Vector3d& v)
{
    geometry_msgs::msg::Vector3 m;
    m.x = v[0];
    m.y = v[1];
    m.z = v[2];
    return m;
}

template<>
geometry_msgs::msg::Quaternion convert_to(const Eigen::Quaterniond& q)
{
    geometry_msgs::msg::Quaternion m;
    m.w = q.w();
    m.x = q.x();
    m.y = q.y();
    m.z = q.z();
    return m;
}

template<>
geometry_msgs::msg::Pose convert_to(const Eigen::Isometry3d& tf)
{
    geometry_msgs::msg::Pose m;
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
geometry_msgs::msg::Transform convert_to(const Eigen::Isometry3d& tf)
{
    geometry_msgs::msg::Transform m;
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
Eigen::Vector3d convert_to(const geometry_msgs::msg::Point& m)
{
    return {m.x, m.y, m.z};
}

template<>
Eigen::Translation3d convert_to(const geometry_msgs::msg::Point& m)
{
    return {m.x, m.y, m.z};
}

template<>
Eigen::Vector3d convert_to(const geometry_msgs::msg::Vector3& m)
{
    return {m.x, m.y, m.z};
}

template<>
Eigen::Translation3d convert_to(const geometry_msgs::msg::Vector3& m)
{
    return {m.x, m.y, m.z};
}

template<>
Eigen::Quaterniond convert_to(const geometry_msgs::msg::Quaternion& m)
{
    return {m.w, m.x, m.y, m.z};
}

template<>
Eigen::Isometry3d convert_to(const geometry_msgs::msg::Pose& m)
{
    auto p = convert_to<Eigen::Translation3d>(m.position);
    auto q = convert_to<Eigen::Quaterniond>(m.orientation);
    return p * q;
}

template<>
Eigen::Isometry3d convert_to(const geometry_msgs::msg::Transform& m)
{
    auto p = convert_to<Eigen::Translation3d>(m.translation);
    auto q = convert_to<Eigen::Quaterniond>(m.rotation);
    return p * q;
}
