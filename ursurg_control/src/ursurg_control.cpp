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
    chain_tip = ur_prefix + "tcp0";
    chain.addSegment(KDL::Segment(KDL::Joint(chain_tip, KDL::Joint::RotZ),
                                  KDL::Frame(KDL::Vector(0.009, 0, 0))));

    // Vector of pointers to the movable joints of the kinematic chain
    auto movable_joints = [&]() {
        std::vector<const KDL::Joint*> segments;

        for (const auto& seg : chain.segments)
            if (seg.getJoint().getType() != KDL::Joint::None)
                segments.push_back(&seg.getJoint());

        return segments;
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

    // Find joint limits from URDF model
    auto [q_min, q_max] = [&]() {
        KDL::JntArray lower(chain.getNrOfJoints());
        KDL::JntArray upper(chain.getNrOfJoints());
        unsigned i = 0;

        for (auto joint : movable_joints) {
            auto model_joint = model.getJoint(joint->getName());

            if (model_joint) {
                // Joint was found in URDF model
                auto limits = model_joint->limits;
                lower(i) = limits->lower;
                upper(i) = limits->upper;
            } else {
                // Hacky-hack: TCP joint
                if (joint->getName() == chain_tip) {
                    lower(i) = math::radians(-45);
                    upper(i) = math::radians(45);
                } else {
                    throw std::runtime_error("Joint '" + joint->getName() + "' has no limits!?");
                }
            }

            ++i;
        }

        return std::make_tuple(lower, upper);
    }();

    // Inverse position kinematics
    KDL::ChainIkSolverPos_NR_JL ik_solver_pos(chain,
                                              q_min,
                                              q_max,
                                              fk_solver,
                                              ik_solver_vel,
                                              nh_priv.param("ik_pos_max_nr_itrs", 15),
                                              nh_priv.param("ik_pos_epsilon", 1.0e-4));

    // Current state
    KDL::JntArray q_current(chain.getNrOfJoints());
    std::array<double, 2> q_current_yaw = {0, 0}; // grasper joints (yaw1, yaw2) are not part of 'chain'
    double grasp_current = 0;

    // Desired state
    KDL::JntArray q_desired = q_current;
    double grasp_desired = grasp_current;

    // Map to q_current by joint name
    auto q_current_by_name = [&]() {
        std::unordered_map<std::string, double*> dict;
        unsigned i = 0;

        for (auto joint : movable_joints)
            dict.emplace(joint->getName(), &q_current(i++));

        // Also map grasper joint states (yaw1, yaw2) that are not part of 'chain'
        dict.emplace(tool_joint_names[2], &q_current_yaw[0]);
        dict.emplace(tool_joint_names[3], &q_current_yaw[1]);

        return dict;
    }();

    auto pub_pose = nh.advertise<geometry_msgs::PoseStamped>("tcp_pose_current", 1);
    auto pub_grasp = nh.advertise<sensor_msgs::JointState>("grasp_current", 1);
    auto pub_robot_move_joint = nh.advertise<sensor_msgs::JointState>("ur/move_joint", 1);
    auto pub_robot_servo_joint = nh.advertise<sensor_msgs::JointState>("ur/servo_joint", 1);
    auto pub_tool_move_joint = nh.advertise<sensor_msgs::JointState>("tool/move_joint", 1);
    auto pub_tool_servo_joint = nh.advertise<sensor_msgs::JointState>("tool/servo_joint", 1);

    auto solve_ik = [&](const auto& m) {
        // If q_desired is close to q_current we use that as the initial guess
        const KDL::JntArray& q_init = ((q_current.data - q_desired.data).norm() < 0.1) ? q_desired : q_current;
        KDL::JntArray q_solution(chain.getNrOfJoints());
        auto retval = ik_solver_pos.CartToJnt(q_init, convert_to<KDL::Frame>(m.pose), q_solution);

        if (retval != KDL::ChainIkSolverVel_wdls::E_NOERROR) {
            ROS_WARN_STREAM("IK error: " << ik_solver_pos.strError(retval));
            return false;
        }

        q_desired = q_solution;
        return true;
    };

    auto make_msgs = [&](const KDL::JntArray& q, double grasp) {
        // The first 6 elements of the solution vector is the robot configuration
        sensor_msgs::JointState m_robot;
        m_robot.header.stamp = ros::Time::now();
        // FIXME joint names
        std::copy(q.data.data(), std::next(q.data.data(), 6), std::back_inserter(m_robot.position));

        // and the last 4 is the tool configuration
        sensor_msgs::JointState m_tool;
        m_tool.header.stamp = m_robot.header.stamp;
        m_tool.name = tool_joint_names;
        m_tool.position.push_back(q(6));
        m_tool.position.push_back(q(7));
        m_tool.position.push_back(q(8) + grasp / 2);
        m_tool.position.push_back(-q(8) + grasp / 2);

        return std::make_tuple(m_robot, m_tool);
    };

    std::list<ros::Subscriber> subscribers{
        mksub<sensor_msgs::JointState>(
            nh, "ur/joint_states", 1, [&](const auto& m) {
                // Cache current UR joint angles
                for (std::size_t i = 0; i < m.position.size(); ++i)
                    *q_current_by_name.at(m.name[i]) = m.position[i];
            },
            ros::TransportHints().tcpNoDelay()),
        mksub<sensor_msgs::JointState>(
            nh, "tool/joint_states", 1, [&](const auto& m) {
                // Cache current tool joint angles
                for (std::size_t i = 0; i < m.position.size(); ++i)
                    *q_current_by_name.at(m.name[i]) = m.position[i];

                // TCP is between the two grasper jaws
                *q_current_by_name.at(chain_tip) = (q_current_yaw[0] - q_current_yaw[1]) / 2;

                // Grasper opening angle
                grasp_current = q_current_yaw[0] + q_current_yaw[1];
            },
            ros::TransportHints().tcpNoDelay()),
        mksub<geometry_msgs::PoseStamped>(
            nh, "move_joint_ik", 1, [&](const auto& m) {
                // TODO: Maybe interpolate path if set-point is far from current position?
                if (solve_ik(m)) {
                    auto [m_robot, m_tool] = make_msgs(q_desired, grasp_desired);
                    pub_robot_move_joint.publish(m_robot);
                    pub_tool_move_joint.publish(m_tool);
                }
            },
            ros::TransportHints().tcpNoDelay()),
        mksub<geometry_msgs::PoseStamped>(
            nh, "servo_joint_ik", 1, [&](const auto& m) {
                if (solve_ik(m)) {
                    auto [m_robot, m_tool] = make_msgs(q_desired, grasp_desired);
                    pub_robot_servo_joint.publish(m_robot);
                    pub_tool_servo_joint.publish(m_tool);
                }
            },
            ros::TransportHints().tcpNoDelay()),
        mksub<sensor_msgs::JointState>(
            nh, "servo_grasp", 1, [&](const auto& m) {
                grasp_desired = m.position.front();
            },
            ros::TransportHints().tcpNoDelay()),
    };

    // Schedule timer to publish TCP pose
    auto timer = nh.createSteadyTimer(
        ros::WallDuration(1.0 / 100),
        [&](const auto&) {
            KDL::Frame f;
            fk_solver.JntToCart(q_current, f);

            geometry_msgs::PoseStamped m_pose;
            m_pose.header.stamp = ros::Time::now();
            m_pose.header.frame_id = chain_root;
            m_pose.pose = convert_to<geometry_msgs::Pose>(f);
            pub_pose.publish(m_pose);

            sensor_msgs::JointState m_grasp;
            m_grasp.header.stamp = ros::Time::now();
            m_grasp.position.push_back(grasp_current);
            pub_grasp.publish(m_grasp);
        });

    ros::spin();

    return EXIT_SUCCESS;
}
