#include <ursurg_common/conversions/kdl.h>
#include <ursurg_common/math.h>
#include <ursurg_common/rosutility/subscription.h>

#include <geometry_msgs/PoseStamped.h>
#include <sensor_msgs/JointState.h>
#include <ursurg_msgs/ToolEndEffectorStateStamped.h>

#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/chainiksolverpos_nr_jl.hpp>
#include <kdl/chainiksolvervel_wdls.hpp>
#include <kdl/tree.hpp>
#include <kdl_parser/kdl_parser.hpp>
#include <urdf/model.h>

#include <ros/ros.h>

#include <range/v3/action/push_back.hpp>
#include <range/v3/algorithm/copy.hpp>
#include <range/v3/view/enumerate.hpp>
#include <range/v3/view/span.hpp>
#include <range/v3/view/zip.hpp>

#include <optional>

struct Init
{
    bool ur;
    bool tool;
};

void append_yaw0(const std::string& prefix, sensor_msgs::JointState& m)
{
    // Find indices of yaw1 and yaw2 joints
    std::ptrdiff_t j = -1;
    std::ptrdiff_t k = -1;

    for (std::ptrdiff_t i = m.name.size() - 1; i >= 0; --i) { // expect yaw1 and yaw2 at end
        if ((j == -1) && (m.name[i].rfind("yaw1") != std::string::npos))
            j = i;
        else if ((k == -1) && (m.name[i].rfind("yaw2") != std::string::npos))
            k = i;
    }

    if (j != -1 && k != -1) {
        // yaw0 angle is between yaw1 and yaw2
        m.name.push_back(prefix + "yaw0");
        m.position.push_back((m.position[j] - m.position[k]) / 2);
    }
};

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

    const auto ur_prefix = nh_priv.param("ur_prefix", ""s);
    const auto tool_prefix = nh_priv.param("tool_prefix", ""s);
    // TODO: std::vector<std::string> ur_joint_names;
    const auto tool_joint_names = [&]() {
        std::vector<std::string> names;

        for (const auto& name : {"roll", "pitch", "yaw1", "yaw2"})
            names.push_back(tool_prefix + name);

        return names;
    }();

    const auto chain_root = ur_prefix + "base_link";
    const auto chain_tip = tool_prefix + "tcp0";
    KDL::Chain chain;

    if (!tree.getChain(chain_root, chain_tip, chain))
        throw std::runtime_error("Failed to get kinematic chain between links: " + chain_root + ", " + chain_tip);

    // Map of joint names :-> limits
    const auto joint_limits = [&]() {
        std::unordered_map<std::string, urdf::JointLimits> limits;

        for (const auto& joint : model.joints_)
            if (joint.second->limits)
                limits[joint.second->name] = *joint.second->limits;

        return limits;
    }();

    // Vector of pointers to the movable joints in the kinematic chain
    const auto movable_joints = [&]() {
        std::vector<const KDL::Joint*> segments;

        for (const auto& seg : chain.segments)
            if (seg.getJoint().getType() != KDL::Joint::None)
                segments.push_back(&seg.getJoint());

        return segments;
    }();

    // Chain joint (movable joints) limits for input to IK solver
    const auto [q_min, q_max] = [&]() {
        KDL::JntArray lower(chain.getNrOfJoints());
        KDL::JntArray upper(chain.getNrOfJoints());

        for (const auto& [i, joint] : movable_joints | ranges::views::enumerate) {
            lower(i) = joint_limits.at(joint->getName()).lower;
            upper(i) = joint_limits.at(joint->getName()).upper;
        }

        return std::make_tuple(lower, upper);
    }();

    // Forward kinematics
    KDL::ChainFkSolverPos_recursive fk_solver(chain);

    // Inverse velocity kinematics
    KDL::ChainIkSolverVel_wdls ik_solver_vel(chain);
    auto weights_vector_js = nh_priv.param("weights_joint_space", std::vector<double>(chain.getNrOfJoints(), 1.0));
    Eigen::MatrixXd Mq = Eigen::VectorXd::Map(weights_vector_js.data(), weights_vector_js.size()).asDiagonal();


    if (auto err = ik_solver_vel.setWeightJS(Mq); err != KDL::ChainIkSolverVel_wdls::E_NOERROR)
        ROS_ERROR_STREAM("Setting IK solver joint space weighting matrix failed: "
                         << ik_solver_vel.strError(err)
                         << "\n" << "Mq =\n" << Mq);

    // Inverse position kinematics
    KDL::ChainIkSolverPos_NR_JL ik_solver_pos(chain,
                                              q_min,
                                              q_max,
                                              fk_solver,
                                              ik_solver_vel,
                                              nh_priv.param("ik_pos_max_nr_itrs", 15),
                                              nh_priv.param("ik_pos_epsilon", 1.0e-4));

    // Current joint state (from robot/tool drivers)
    KDL::JntArray q_current(chain.getNrOfJoints());
    KDL::JntArray q_desired(chain.getNrOfJoints());
    std::array<double, 2> q_yaw_dummy = {0, 0}; // grasper joints (yaw1, yaw2) are not part of 'chain'

    // State signals
    Init init{false, false};

    // Map to q_current by joint name
    auto q_current_by_name = [&]() {
        std::unordered_map<std::string, double*> dict;
        unsigned i = 0;

        for (const auto& joint : movable_joints)
            dict.insert({joint->getName(), &q_current(i++)});

        // Also map grasper joint states (yaw1, yaw2) that are not part of 'chain'
        dict.insert({tool_joint_names[2], &q_yaw_dummy[0]});
        dict.insert({tool_joint_names[3], &q_yaw_dummy[1]});

        return dict;
    }();

    auto pub_robot_move_joint = nh.advertise<sensor_msgs::JointState>("ur/move_joint", 1);
    auto pub_robot_servo_joint = nh.advertise<sensor_msgs::JointState>("ur/servo_joint", 1);
    auto pub_tool_move_joint = nh.advertise<sensor_msgs::JointState>("tool/move_joint", 1);
    auto pub_tool_servo_joint = nh.advertise<sensor_msgs::JointState>("tool/servo_joint", 1);

    // Get IK solution as joint states to push to robot and tool drivers, respectively
    auto get_ik_solution = [&](const auto& pose_desired, double grasp_desired)
            -> std::optional<std::pair<sensor_msgs::JointState, sensor_msgs::JointState>> {
        // If q_current and q_desired (the previous IK solution) are close,
        // we use q_desired as the seed for the IK algorithm
        auto q_init = ((q_desired.data - q_current.data).norm() > math::pi / 8) ? q_current : q_desired;

        // Find configuration of joints in 'chain'
        if (auto err = ik_solver_pos.CartToJnt(q_init, pose_desired, q_desired);
                err != KDL::ChainIkSolverVel_wdls::E_NOERROR) {
            ROS_WARN_STREAM("IK error: " << ik_solver_pos.strError(err));
            return {};
        }

        // yaw1, yaw2 configuration given a desired grasper opening angle
        double q_yaw1 = q_desired(8) + grasp_desired / 2;
        double q_yaw2 = -q_desired(8) + grasp_desired / 2;

        // Enforce joint limits for yaw1, yaw2
        const auto& lim_yaw1 = joint_limits.at(tool_joint_names[2]);
        q_yaw1 = std::clamp(q_yaw1, lim_yaw1.lower, lim_yaw1.upper);
        const auto& lim_yaw2 = joint_limits.at(tool_joint_names[3]);
        q_yaw2 = std::clamp(q_yaw2, lim_yaw2.lower, lim_yaw2.upper);

        // The first 6 elements of the solution vector is the robot configuration
        sensor_msgs::JointState m_robot;
        m_robot.header.stamp = ros::Time::now();
        // FIXME joint names
        m_robot.position |= ranges::actions::push_back(ranges::span{q_desired.data.data(), 6});

        // and the last 4 is the tool configuration
        sensor_msgs::JointState m_tool;
        m_tool.header.stamp = m_robot.header.stamp;
        m_tool.name = tool_joint_names;
        m_tool.position.push_back(q_desired(6)); // roll
        m_tool.position.push_back(q_desired(7)); // pitch (wrist)
        m_tool.position.push_back(q_yaw1);       // yaw1 (jaw1)
        m_tool.position.push_back(q_yaw2);       // yaw2 (jaw2)

        return std::make_pair(m_robot, m_tool);
    };

    std::list<ros::Subscriber> subscribers{
        mksub<sensor_msgs::JointState>(
            nh, "ur/joint_states", 1, [&](const auto& m) {
                // Cache current UR joint angles
                for (auto [n, q] : ranges::views::zip(m.name, m.position))
                    *q_current_by_name[n] = q;

                init.ur = true;
            }, ros::TransportHints().udp().tcp().tcpNoDelay()),
        mksub<sensor_msgs::JointState>(
            nh, "tool/joint_states", 1, [&](auto m) {
                append_yaw0(tool_prefix, m);

                // Cache current tool joint angles
                for (auto [n, q] : ranges::views::zip(m.name, m.position))
                    *q_current_by_name[n] = q;

                init.tool = true;
            }, ros::TransportHints().udp().tcp().tcpNoDelay()),
        mksub<ursurg_msgs::ToolEndEffectorState>(
            nh, "servo_joint_ik", 1, [&](const auto& m) {
                if (!init.ur || !init.tool)
                    return;

                auto sol = get_ik_solution(convert_to<KDL::Frame>(m.pose), m.grasper_angle);

                if (sol) {
                    pub_robot_servo_joint.publish(sol->first);
                    pub_tool_servo_joint.publish(sol->second);
                }
            }, ros::TransportHints().udp().tcp().tcpNoDelay()),
        mksub<ursurg_msgs::ToolEndEffectorStateStamped>(
            nh, "move_joint_ik", 1, [&](const auto& m) {
                if (!init.ur || !init.tool)
                    return;

                auto sol = get_ik_solution(convert_to<KDL::Frame>(m.ee.pose), m.ee.grasper_angle);

                if (sol) {
                    pub_robot_move_joint.publish(sol->first);
                    pub_tool_move_joint.publish(sol->second);
                }
            }, ros::TransportHints().tcp().tcpNoDelay()),
    };

    ros::spin();

    return EXIT_SUCCESS;
}
