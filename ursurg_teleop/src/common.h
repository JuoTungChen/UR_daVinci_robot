#pragma once

#include <geometry_msgs/Pose.h>

#include <Eigen/Geometry>

inline Eigen::Isometry3d convert(const geometry_msgs::Pose& m)
{
    return Eigen::Translation3d(m.position.x, m.position.y, m.position.z)
        * Eigen::Quaterniond(m.orientation.w, m.orientation.x, m.orientation.y, m.orientation.z);
}

inline geometry_msgs::Pose convert(const Eigen::Isometry3d& tf)
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
