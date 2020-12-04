#include <ursurg_common/conversions/kdl.h>
#include <ursurg_common/math.h>
#include <ursurg_common/rosutility.h>

#include <geometry_msgs/PoseStamped.h>
#include <sensor_msgs/JointState.h>

#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/chainiksolverpos_nr_jl.hpp>
#include <kdl/chainiksolvervel_wdls.hpp>
#include <kdl/tree.hpp>
#include <kdl_parser/kdl_parser.hpp>
#include <urdf/model.h>

#include <ros/ros.h>

#include <optional>

struct Init
{
    bool ur;
    bool tool;
    bool fk;
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
    auto chain_tip = tool_prefix + "jaw0";
    KDL::Chain chain;

    if (!tree.getChain(chain_root, chain_tip, chain))
        throw std::runtime_error("Failed to get kinematic chain between links: " + chain_root + ", " + chain_tip);

    // Add a virtual TCP frame to be placed between the grasper jaws
    // TODO: Adjust offset depending on tool type (9mm for LND)
    chain_tip = ur_prefix + "tcp0";
    chain.addSegment(KDL::Segment(KDL::Joint(chain_tip, KDL::Joint::RotZ),
                                  KDL::Frame(KDL::Vector(0.009, 0, 0))));

    // Map of joint names :-> limits
    const auto joint_limits = [&]() {
        std::unordered_map<std::string, urdf::JointLimits> limits;

        for (auto joint : model.joints_) {
            auto plim = joint.second->limits;

            if (plim)
                limits[joint.second->name] = *plim;
        }

        // Virtual TCP joint
        // TODO verify yaw1/yaw2 lower/upper corresponds to tcp0 -/+ direction
        limits[chain_tip] = urdf::JointLimits();
        limits[chain_tip].lower = limits.at(tool_prefix + "yaw1").lower;
        limits[chain_tip].upper = limits.at(tool_prefix + "yaw2").upper;

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

    // Current joint state
    KDL::JntArray q_current(chain.getNrOfJoints());
    std::array<double, 2> q_current_yaw = {0, 0}; // grasper joints (yaw1, yaw2) are not part of 'chain'

    // Desired manipulator state
    KDL::Frame tcp_pose_desired = KDL::Frame::Identity();
    double grasp_desired = 0;

    // State signals
    Init init{false, false, false};
    bool new_servo_cmd = false;

    // Map to q_current by joint name
    auto q_current_by_name = [&]() {
        std::unordered_map<std::string, double*> dict;
        unsigned i = 0;

        for (auto joint : movable_joints)
            dict.insert({joint->getName(), &q_current(i++)});

        // Also map grasper joint states (yaw1, yaw2) that are not part of 'chain'
        dict.insert({tool_joint_names[2], &q_current_yaw[0]});
        dict.insert({tool_joint_names[3], &q_current_yaw[1]});

        return dict;
    }();

    auto pub_pose_current = nh.advertise<geometry_msgs::PoseStamped>("tcp_pose_current", 1);
    auto pub_pose_desired = nh.advertise<geometry_msgs::PoseStamped>("tcp_pose_desired", 1);
    auto pub_grasp_current = nh.advertise<sensor_msgs::JointState>("grasp_current", 1);
    auto pub_grasp_desired = nh.advertise<sensor_msgs::JointState>("grasp_desired", 1);
    auto pub_robot_move_joint = nh.advertise<sensor_msgs::JointState>("ur/move_joint", 1);
    auto pub_robot_servo_joint = nh.advertise<sensor_msgs::JointState>("ur/servo_joint", 1);
    auto pub_tool_move_joint = nh.advertise<sensor_msgs::JointState>("tool/move_joint", 1);
    auto pub_tool_servo_joint = nh.advertise<sensor_msgs::JointState>("tool/servo_joint", 1);

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

    // Solve for two tool yaw joint angles
    auto solve_ik_yaw = [&](double tcp0, double grasp_desired) {
        std::array<double, 2> q_yaw{
            tcp0 + grasp_desired / 2,
            -tcp0 + grasp_desired / 2
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
        m_tool.position.push_back(q_chain(6));
        m_tool.position.push_back(q_chain(7));
        m_tool.position.push_back(q_yaw[0]);
        m_tool.position.push_back(q_yaw[1]);

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
            nh, "tool/joint_states", 1, [&](const auto& m) {
                // Cache current tool joint angles
                for (std::size_t i = 0; i < m.position.size(); ++i)
                    *q_current_by_name.at(m.name[i]) = m.position[i];

                // TCP is between the two grasper jaws
                *q_current_by_name.at(chain_tip) = (q_current_yaw[0] - q_current_yaw[1]) / 2;

                init.tool = true;
            },
            ros::TransportHints().udp().tcp().tcpNoDelay()),
        mksub<geometry_msgs::PoseStamped>(
            nh, "move_joint_ik", 1, [&](const auto& m) {
                if (!init.fk)
                    return;

                // TODO: Maybe interpolate path if set-point is far from current position?
                tcp_pose_desired = convert_to<KDL::Frame>(m.pose);
                auto q_chain = solve_ik_tcp(tcp_pose_desired);

                if (q_chain) {
                    auto q_yaw = solve_ik_yaw((*q_chain)(8), grasp_desired);
                    auto [m_robot, m_tool] = q_to_msgs(*q_chain, q_yaw);
                    pub_robot_move_joint.publish(m_robot);
                    pub_tool_move_joint.publish(m_tool);
                }
            },
            ros::TransportHints().tcp().tcpNoDelay()),
        mksub<sensor_msgs::JointState>(
            nh, "move_grasp", 1, [&](const auto& m) {
                if (!init.fk)
                    return;

                grasp_desired = m.position.front();
                auto q_chain = solve_ik_tcp(tcp_pose_desired);

                if (q_chain) {
                    auto q_yaw = solve_ik_yaw((*q_chain)(8), grasp_desired);
                    auto [m_robot, m_tool] = q_to_msgs(*q_chain, q_yaw);
                    pub_robot_move_joint.publish(m_robot);
                    pub_tool_move_joint.publish(m_tool);
                }
            },
            ros::TransportHints().tcp().tcpNoDelay()),
        mksub<geometry_msgs::PoseStamped>(
            nh, "servo_joint_ik", 1, [&](const auto& m) {
                if (!init.fk)
                    return;

                tcp_pose_desired = convert_to<KDL::Frame>(m.pose);
                new_servo_cmd = true;
            },
            ros::TransportHints().udp().tcp().tcpNoDelay()),
        mksub<sensor_msgs::JointState>(
            nh, "servo_grasp", 1, [&](const auto& m) {
                if (!init.fk)
                    return;

                grasp_desired = m.position.front();
                new_servo_cmd = true;
            },
            ros::TransportHints().udp().tcp().tcpNoDelay()),
    };

    // Schedule timer to solve IK and publish servo commands
    auto timer0 = nh.createSteadyTimer(
        ros::WallDuration(1.0 / nh_priv.param("servo_rate", 125)),
        [&](const auto&) {
            if (!new_servo_cmd || !init.fk)
                return;

            auto q_chain = solve_ik_tcp(tcp_pose_desired);

            if (q_chain) {
                auto q_yaw = solve_ik_yaw((*q_chain)(8), grasp_desired);
                auto [m_robot, m_tool] = q_to_msgs(*q_chain, q_yaw);
                pub_robot_servo_joint.publish(m_robot);
                pub_tool_servo_joint.publish(m_tool);
            }

            new_servo_cmd = false;
        });

    // Schedule timer to compute and publish forward kinematics
    auto timer1 = nh.createSteadyTimer(
        ros::WallDuration(1.0 / nh_priv.param("publish_rate", 125)),
        [&](const auto&) {
            if (!init.ur || !init.tool)
                return;

            // Compute forward kinematics
            KDL::Frame tcp_pose_current;
            fk_solver.JntToCart(q_current, tcp_pose_current);
            double grasp_current = q_current_yaw[0] + q_current_yaw[1];

            // Initially: desired = current
            if (!init.fk) {
                tcp_pose_desired = tcp_pose_current;
                grasp_desired = grasp_current;
                init.fk = true;
            }

            geometry_msgs::PoseStamped m_pose;
            m_pose.header.stamp = ros::Time::now();
            m_pose.header.frame_id = chain_root;
            m_pose.pose = convert_to<geometry_msgs::Pose>(tcp_pose_current);
            pub_pose_current.publish(m_pose);

            m_pose.pose = convert_to<geometry_msgs::Pose>(tcp_pose_desired);
            pub_pose_desired.publish(m_pose);

            sensor_msgs::JointState m_grasp;
            m_grasp.header.stamp = ros::Time::now();
            m_grasp.position.push_back(grasp_current);
            pub_grasp_current.publish(m_grasp);

            m_grasp.position.back() = grasp_desired;
            pub_grasp_desired.publish(m_grasp);
        });

    ros::spin();

    return EXIT_SUCCESS;
}
