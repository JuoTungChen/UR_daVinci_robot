#include <ursurg_common/conversions/eigen.h>
#include <ursurg_common/rosutility.h>

#include <geometry_msgs/PoseStamped.h>
#include <std_msgs/Bool.h>

#include <ros/ros.h>

#include <Eigen/Geometry>

struct Pose
{
    Eigen::Translation3d translation;
    Eigen::Quaterniond orientation;

    Pose() = default;

    Pose(const Eigen::Translation3d& tra, const Eigen::Quaterniond& ori)
        : translation(tra)
        , orientation(ori)
    {}

    Pose(Eigen::Translation3d&& tra, Eigen::Quaterniond&& ori)
        : translation(std::move(tra))
        , orientation(std::move(ori))
    {}

    Pose(const Eigen::Isometry3d& tf)
        : translation(tf.translation())
        , orientation(tf.linear())
    {}

    Pose(const geometry_msgs::Pose& pose)
        : translation(convert_to<Eigen::Translation3d>(pose.position))
        , orientation(convert_to<Eigen::Quaterniond>(pose.orientation))
    {}

    Pose inverse() const
    {
        return {translation.inverse(), orientation.inverse()};
    }

    operator Eigen::Isometry3d() const
    {
        return translation * orientation;
    }

    Pose operator*(const Pose& p) const
    {
        return {translation * p.translation, orientation * p.orientation};
    }

    static Pose Identity()
    {
        return {Eigen::Translation3d::Identity(), Eigen::Quaterniond::Identity()};
    }
};

int main(int argc, char* argv[])
{
    ros::init(argc, argv, "teleop");
    ros::NodeHandle nh;
    ros::NodeHandle nh_priv("~");

    bool clutch_engaged = false;

    std::string pose_tcp_frame_id;
    Eigen::Isometry3d t_tcp_current = Eigen::Isometry3d::Identity();
    Eigen::Isometry3d t_tcp_desired = Eigen::Isometry3d::Identity();

    Pose pose_haptic_current = Pose::Identity();
    Pose pose_haptic_last = Pose::Identity();

    // TODO: read this transform from "static TF" topic
    Eigen::Isometry3d t_touch_base = Eigen::Isometry3d::Identity();
    // clang-format off
    t_touch_base.linear() << 1,  0,  0,
                             0,  0,  1,
                             0, -1,  0;
    // clang-format on
    Eigen::Isometry3d t_base_touch = t_touch_base.inverse();

    auto pub_pose_desired = nh.advertise<geometry_msgs::PoseStamped>("tcp_pose_desired", 1);

    std::list<ros::Subscriber> subs{
        mksub<std_msgs::Bool>(
            nh, "clutch_engaged", 4, [&](const auto& m) {
                clutch_engaged = m.data;

                // Initially set desired=current
                if (clutch_engaged)
                    t_tcp_desired = t_tcp_current;
            },
            ros::TransportHints().tcpNoDelay()),
        mksub<geometry_msgs::PoseStamped>(
            nh, "tcp_pose_current", 1, [&](const auto& m) {
                pose_tcp_frame_id = m.header.frame_id;
                t_tcp_current = convert_to<Eigen::Isometry3d>(m.pose);
            },
            ros::TransportHints().tcpNoDelay()),
        mksub<geometry_msgs::PoseStamped>(
            nh, "haptic_pose", 1, [&](const auto& m) {
                pose_haptic_last = std::move(pose_haptic_current);
                pose_haptic_current = m.pose;

                if (clutch_engaged) {
                    // Compute diff
                    Pose diff = pose_haptic_last.inverse() * pose_haptic_current;

                    // Change of basis
                    Eigen::Isometry3d t_incr = t_base_touch * diff * t_touch_base;

                    // Accumulate translation and rotation separately
                    t_tcp_desired.translation() += t_incr.translation();
                    t_tcp_desired.linear() = t_incr.linear() * t_tcp_desired.linear();
                }
            },
            ros::TransportHints().tcpNoDelay()),
    };

    auto timer = nh.createSteadyTimer(
        ros::WallDuration(1.0 / nh_priv.param("publish_rate", 500)),
        [&](const auto&) {
            if (clutch_engaged) {
                geometry_msgs::PoseStamped m;
                m.header.stamp = ros::Time::now();
                m.header.frame_id = pose_tcp_frame_id;
                m.pose = convert_to<geometry_msgs::Pose>(t_tcp_desired);
                pub_pose_desired.publish(m);

                // TODO buttons -> grasper angle
            }
        });

    ros::spin();
    return EXIT_SUCCESS;
}
