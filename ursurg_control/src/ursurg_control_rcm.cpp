#include <ursurg_common/conversions/eigen.h>
#include <ursurg_common/math.h>
#include <ursurg_common/rosutility/subscription.h>

#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <sensor_msgs/JointState.h>
#include <ursurg_msgs/SetRCM.h>

#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/chainiksolverpos_nr_jl.hpp>
#include <kdl/chainiksolvervel_wdls.hpp>
#include <kdl/tree.hpp>
#include <kdl_parser/kdl_parser.hpp>
#include <urdf/model.h>

#include <ros/ros.h>

#include <Eigen/Geometry>

#include <optional>

KDL::Frame eigen2kdl(const Eigen::Matrix3d& R, const Eigen::Vector3d& t)
{
    using MapR = Eigen::Map<Eigen::Matrix<double, 3, 3, Eigen::RowMajor>>;
    using MapV = Eigen::Map<Eigen::Vector3d>;

    KDL::Frame f;
    MapR(f.M.data) = R;
    MapV(f.p.data) = t;
    return f;
}

Eigen::Matrix3d tool_rod_orient_desired(const Eigen::Vector3d& p_rod,
                                        const Eigen::Vector3d& p_rcm)
{
    Eigen::Vector3d z_axis = (p_rod - p_rcm).normalized();
    Eigen::Vector3d y_axis = z_axis.cross(Eigen::Vector3d::UnitX()).normalized();
    Eigen::Vector3d x_axis = y_axis.cross(z_axis).normalized();

    Eigen::Matrix3d R;
    R.block<3, 1>(0, 0) = x_axis;
    R.block<3, 1>(0, 1) = y_axis;
    R.block<3, 1>(0, 2) = z_axis;

    return R;
}

std::vector<double> tool_config_desired(const Eigen::Matrix3d& r_shaft,
                                        const Eigen::Matrix3d& r_tip)
{
    std::vector<double> q(4);

    Eigen::Matrix3d shaftRtip = r_shaft.inverse() * r_tip;

    // Isolate individual joints from this rotation matrix
    q[1] = std::asin(shaftRtip(2, 2));
    double c1 = std::cos(q[1]);
    q[0] = std::atan2(-shaftRtip(1, 2) * c1, -shaftRtip(0, 2) * c1) - math::pi; // ups, quickfix!
    q[2] = std::atan2(-shaftRtip(2, 1) * c1, shaftRtip(2, 0) * c1);

    return q;
}

int main(int argc, char* argv[])
{
    using namespace std::string_literals;

    ros::init(argc, argv, "ursurg_control");
    ros::NodeHandle nh;
    ros::NodeHandle nh_priv("~");

    urdf::Model model;

    if (!model.initParam("robot_description"))
        throw std::runtime_error("Robot model not found (set the 'robot_description' parameter)");

    KDL::Tree tree;

    if (!kdl_parser::treeFromUrdfModel(model, tree))
        throw std::runtime_error("Failed to extract KDL tree from robot description");

    auto ur_prefix = nh_priv.param("ur_prefix", ""s);
    auto tool_prefix = nh_priv.param("tool_prefix", ""s);
    // TODO: std::vector<std::string> ur_joint_names;
    auto tool_joint_names = [&]() {
        std::vector<std::string> names;

        for (const auto& name : {"roll", "pitch", "yaw1", "yaw2"})
            names.push_back(tool_prefix + name);

        return names;
    }();

    auto chain_root = ur_prefix + "base_link";
    auto chain_tip = tool_prefix + "tip_calib";
    KDL::Chain chain;

    if (!tree.getChain(chain_root, chain_tip, chain))
        throw std::runtime_error("Failed to get kinematic chain between links: " + chain_root + ", " + chain_tip);

    chain.addSegment(KDL::Segment(KDL::Joint("rcm_helper_joint"),
                                  KDL::Frame(KDL::Rotation::RotY(math::half_pi))));

    // Map of joint names :-> limits
    const auto joint_limits = [&]() {
        std::unordered_map<std::string, urdf::JointLimits> limits;

        for (auto joint : model.joints_) {
            auto plim = joint.second->limits;

            if (plim)
                limits[joint.second->name] = *plim;
        }

        return limits;
    }();

    // Vector of pointers to the movable joints in the kinematic chain
    auto movable_joints = [&]() {
        std::vector<const KDL::Joint*> segments;

        for (const auto& seg : chain.segments)
            if (seg.getJoint().getType() != KDL::Joint::None)
                segments.push_back(&seg.getJoint());

        return segments;
    }();

    // Chain joint (movable joints) limits for input to IK solver
    auto [q_min, q_max] = [&]() {
        KDL::JntArray lower(chain.getNrOfJoints());
        KDL::JntArray upper(chain.getNrOfJoints());
        unsigned i = 0;

        for (auto joint : movable_joints) {
            lower(i) = joint_limits.at(joint->getName()).lower;
            upper(i) = joint_limits.at(joint->getName()).upper;
            ++i;
        }

        return std::make_tuple(lower, upper);
    }();

    // Forward kinematics
    KDL::ChainFkSolverPos_recursive fk_solver(chain);

    // Inverse velocity kinematics
    KDL::ChainIkSolverVel_wdls ik_solver_vel(chain);

    // Inverse position kinematics
    KDL::ChainIkSolverPos_NR_JL ik_solver_pos(chain,
                                              q_min,
                                              q_max,
                                              fk_solver,
                                              ik_solver_vel,
                                              nh_priv.param("ik_pos_max_nr_itrs", 15),
                                              nh_priv.param("ik_pos_epsilon", 1.0e-4));

    // Current joint state
    KDL::JntArray q_current(chain.getNrOfJoints());

    Eigen::Vector3d p_rcm(0, 0, 0);

    // Desired manipulator state
    Eigen::Isometry3d tcp_pose_desired = Eigen::Isometry3d::Identity();
    double grasp_desired = 0;

    // State signals
    bool init_ur = false;
    bool new_servo_cmd = false;

    // Map to q_current by joint name
    auto q_current_by_name = [&]() {
        std::unordered_map<std::string, double*> dict;
        unsigned i = 0;

        for (auto joint : movable_joints)
            dict.insert({joint->getName(), &q_current(i++)});

        return dict;
    }();

    auto pub_robot_servo_joint = nh.advertise<sensor_msgs::JointState>("ur/servo_joint", 1);
    auto pub_tool_servo_joint = nh.advertise<sensor_msgs::JointState>("tool/servo_joint", 1);
    auto pub_rcm = nh.advertise<sensor_msgs::JointState>("rcm", 1);

    // Solve inverse kinematics for the chain given the desired TCP pose
    auto solve_ik_tcp = [&](const KDL::Frame& pose_desired) -> std::optional<KDL::JntArray> {
        KDL::JntArray q_chain(chain.getNrOfJoints());

        // Use current position as initial guess
        auto retval = ik_solver_pos.CartToJnt(q_current, pose_desired, q_chain);

        if (retval != KDL::ChainIkSolverVel_wdls::E_NOERROR) {
            ROS_WARN_STREAM("IK error: " << ik_solver_pos.strError(retval));
            return {};
        }

        return q_chain;
    };

    std::list<ros::Subscriber> subscribers{
        mksub<sensor_msgs::JointState>(
            nh, "ur/joint_states", 1, [&](const auto& m) {
                // Cache current UR joint angles
                assert(m.name.size() == m.position.size());

                for (std::size_t i = 0; i < m.position.size(); ++i)
                    *q_current_by_name.at(m.name[i]) = m.position[i];

                init_ur = true;
            },
            ros::TransportHints().udp().tcp().tcpNoDelay()),
        mksub<geometry_msgs::PoseStamped>(
            nh, "servo_joint_ik", 1, [&](const auto& m) {
                tcp_pose_desired = convert_to<Eigen::Isometry3d>(m.pose);
                new_servo_cmd = true;
            },
            ros::TransportHints().udp().tcp().tcpNoDelay()),
    };

    auto srv_set_rcm = mksrv<ursurg_msgs::SetRCM>(
        nh, "set_rcm", [&](auto& req, auto&) {
            p_rcm = convert_to<Eigen::Vector3d>(req.point);
            return true;
        });

    // Schedule timer to solve IK and publish servo commands
    auto timer0 = nh.createSteadyTimer(
        ros::WallDuration(1.0 / nh_priv.param("servo_rate", 125)),
        [&](const auto&) {
            if (!new_servo_cmd || !init_ur)
                return;

            // HACK: Hard-coded translation of -18mm along TCP x-axis to
            // arrive at the '#_tool_rotation' frame.
            // FIXME: Use tool-type-specific parameters from URDF
            Eigen::Isometry3d tcp_pose_desired_x = tcp_pose_desired * Eigen::Translation3d(-0.018, 0, 0);
            Eigen::Vector3d p_rod = tcp_pose_desired_x.translation();

            Eigen::Matrix3d r_rod_desired = tool_rod_orient_desired(p_rod, p_rcm);

            // Rotate so that x-axis is along tool shaft
            r_rod_desired = r_rod_desired * Eigen::AngleAxisd(math::pi, Eigen::Vector3d::UnitZ());

            // Solve IK with input target frame '#_tool_calib'
            // FIXME: Here, p_rod should be the projection of 'p_rod' onto the
            // vector along to tool rod (p_rod - p_rcm), ie. the rotation
            // 'r_rod_desired'
            auto q_ur = solve_ik_tcp(eigen2kdl(r_rod_desired, p_rod));
            auto q_tool = tool_config_desired(r_rod_desired, tcp_pose_desired.linear());

            // TODO: check tool joint q against its limits

            if (q_ur) {
                sensor_msgs::JointState m_robot;
                m_robot.header.stamp = ros::Time::now();
                // FIXME joint names
                std::copy(q_ur->data.data(), std::next(q_ur->data.data(), 6), std::back_inserter(m_robot.position));

                sensor_msgs::JointState m_tool;
                m_tool.header.stamp = m_robot.header.stamp;
                m_tool.name = tool_joint_names;
                m_tool.position = q_tool;

                pub_robot_servo_joint.publish(m_robot);
                pub_tool_servo_joint.publish(m_tool);
            }

            new_servo_cmd = false;
        });

    auto timer1 = nh.createSteadyTimer(
        ros::WallDuration(0.1),
        [&](const auto&) {
            geometry_msgs::PointStamped m_rcm;
            m_rcm.header.stamp = ros::Time::now();
            m_rcm.header.frame_id = chain_root;
            m_rcm.point = convert_to<geometry_msgs::Point>(p_rcm);
            pub_rcm.publish(m_rcm);
        });

    ros::spin();

    return EXIT_SUCCESS;
}
