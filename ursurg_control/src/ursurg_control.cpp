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

struct Init
{
    bool ur;
    bool tool;
};

void appendYaw0(const std::string& prefix, sensor_msgs::JointState& m)
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
    auto chain_tip = tool_prefix + "tcp0";
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

        for (const auto& joint : movable_joints) {
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
    auto weights_vector_js = nh_priv.param("weights_joint_space", std::vector<double>(chain.getNrOfJoints(), 1.0));
    Eigen::MatrixXd Mq = Eigen::VectorXd::Map(weights_vector_js.data(), weights_vector_js.size()).asDiagonal();
    auto retval = ik_solver_vel.setWeightJS(Mq);

    if (retval != KDL::ChainIkSolverVel_wdls::E_NOERROR)
        ROS_ERROR_STREAM("Setting IK solver joint space weighting matrix failed: "
                         << ik_solver_vel.strError(retval)
                         << "\n" << "Mq = \n" << Mq);

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

        for (auto joint : movable_joints)
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

    // Solve for two tool yaw joint angles
    auto solve_ik_yaw = [&](double tcp0, double grasp_desired) {
        std::array<double, 2> q_yaw{
            tcp0 + grasp_desired / 2,
            -tcp0 + grasp_desired / 2,
        };

        // Enforce joint limits
        auto limitx = [](auto& x, const auto& limits) {
            if (x > limits.upper)
                x = limits.upper;

            if (x < limits.lower)
                x = limits.lower;
        };

        limitx(q_yaw[0], joint_limits.at(tool_joint_names[2]));
        limitx(q_yaw[1], joint_limits.at(tool_joint_names[3]));

        return q_yaw;
    };

    auto q_to_msgs = [&](const auto& q_chain, const auto& q_yaw) {
        // The first 6 elements of the solution vector is the robot configuration
        sensor_msgs::JointState m_robot;
        m_robot.header.stamp = ros::Time::now();
        // FIXME joint names
        std::copy(q_chain.data.data(), std::next(q_chain.data.data(), 6), std::back_inserter(m_robot.position));

        // and the last 4 is the tool configuration
        sensor_msgs::JointState m_tool;
        m_tool.header.stamp = m_robot.header.stamp;
        m_tool.name = tool_joint_names;
        m_tool.position.push_back(q_chain(6)); // roll
        m_tool.position.push_back(q_chain(7)); // pitch (wrist)
        m_tool.position.push_back(q_yaw[0]);   // yaw1 (jaw1)
        m_tool.position.push_back(q_yaw[1]);   // yaw2 (jaw2)

        return std::make_tuple(m_robot, m_tool);
    };

    std::list<ros::Subscriber> subscribers{
        mksub<sensor_msgs::JointState>(
            nh, "ur/joint_states", 1, [&](const auto& m) {
                // Cache current UR joint angles
                for (std::size_t i = 0; i < m.position.size(); ++i)
                    *q_current_by_name.at(m.name[i]) = m.position[i];

                init.ur = true;
            },
            ros::TransportHints().udp().tcp().tcpNoDelay()),
        mksub<sensor_msgs::JointState>(
            nh, "tool/joint_states", 1, [&](auto m) {
                appendYaw0(tool_prefix, m);

                // Cache current tool joint angles
                for (std::size_t i = 0; i < m.name.size(); ++i)
                    *q_current_by_name.at(m.name[i]) = m.position[i];

                init.tool = true;
            },
            ros::TransportHints().udp().tcp().tcpNoDelay()),
        mksub<ursurg_msgs::ToolEndEffectorState>(
            nh, "servo_joint_ik", 1, [&](const auto& m) {
                if (!init.ur || !init.tool)
                    return;

                // Desired manipulator state (target for IK solution)
                auto pose_desired = convert_to<KDL::Frame>(m.pose);
                auto grasp_desired = m.grasper_angle;

                // If q_current and q_desired (the previous IK solution) are close,
                // we use q_desired as the seed for the IK algorithm
                const auto& q_init = ((q_desired.data - q_current.data).norm() > math::half_pi) ? q_current : q_desired;

                // Find configuration of joints in 'chain'
                if (auto err = ik_solver_pos.CartToJnt(q_init, pose_desired, q_desired);
                    err != KDL::ChainIkSolverVel_wdls::E_NOERROR) {
                    ROS_WARN_STREAM("IK error: " << ik_solver_pos.strError(err));
                    return;
                }

                // Find joint configuration for yaw1, yaw2
                auto q_yaw = solve_ik_yaw(q_desired(8), grasp_desired);

                auto [m_robot, m_tool] = q_to_msgs(q_desired, q_yaw);
                pub_robot_servo_joint.publish(m_robot);
                pub_tool_servo_joint.publish(m_tool);

            },
            ros::TransportHints().udp().tcp().tcpNoDelay()),
    };

    ros::spin();

    return EXIT_SUCCESS;
}
