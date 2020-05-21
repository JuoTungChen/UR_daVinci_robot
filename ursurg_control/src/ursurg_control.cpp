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
        throw std::runtime_error("Robot model not found (set the 'robot_description' parameter");

    KDL::Tree tree;

    if (!kdl_parser::treeFromUrdfModel(model, tree))
        throw std::runtime_error("Failed to extract KDL tree from robot description");

    auto chain_root = nh_priv.param("ur_prefix", ""s) + "base_link";
    auto chain_tip = nh_priv.param("tool_prefix", ""s) + "jaw0";
    KDL::Chain chain;

    if (!tree.getChain(chain_root, chain_tip, chain))
        throw std::runtime_error("Failed to get kinematic chain between links: " + chain_root + ", " + chain_tip);

    // Forward kinematics
    KDL::ChainFkSolverPos_recursive fk_solver(chain);

    // Inverse velocity kinematics
    KDL::ChainIkSolverVel_wdls ik_solver_vel(chain);
    auto weights_js = nh_priv.param("weights_joint_space", std::vector<double>(chain.getNrOfJoints(), 1.0));
    ik_solver_vel.setWeightJS(Eigen::VectorXd::Map(weights_js.data(), weights_js.size()).asDiagonal());

    // Find joint limits from URDF model
    auto [q_min, q_max] = [&]() {
        KDL::JntArray lower(chain.getNrOfJoints());
        KDL::JntArray upper(chain.getNrOfJoints());
        unsigned i = 0;

        for (const auto& seg : chain.segments) {
            if (seg.getJoint().getType() != KDL::Joint::None) { // A moving joint
                auto limits = model.getJoint(seg.getJoint().getName())->limits;
                lower(i) = limits->lower;
                upper(i) = limits->upper;
                ++i;
            }
        }

        return std::make_tuple(lower, upper);
    }();

    // Inverse position kinematics
    KDL::ChainIkSolverPos_NR_JL ik_solver_pos(chain, q_min, q_max, fk_solver, ik_solver_vel);

    // Current joint state
    KDL::JntArray q_current(chain.getNrOfJoints());

    // Map to q_current by joint name
    auto q_current_by_name = [&]() {
        std::unordered_map<std::string, double*> dict;
        unsigned i = 0;

        for (const auto& seg : chain.segments) {
            if (seg.getJoint().getType() != KDL::Joint::None) { // A moving joint
                dict.emplace(seg.getJoint().getName(), &q_current(i++));
            }
        }

        return dict;
    }();

    auto pub_pose = nh.advertise<geometry_msgs::PoseStamped>("tcp_pose_current", 1);
    auto pub_robot_move_joint = nh.advertise<sensor_msgs::JointState>("ur/move_joint", 1);
    auto pub_robot_servo_joint = nh.advertise<sensor_msgs::JointState>("ur/servo_joint", 1);
    auto pub_tool_move_joint = nh.advertise<sensor_msgs::JointState>("tool/move_joint", 1);
    auto pub_tool_servo_joint = nh.advertise<sensor_msgs::JointState>("tool/servo_joint", 1);

    auto solve_and_make_msgs = [&](const auto& m) -> std::optional<std::pair<sensor_msgs::JointState, sensor_msgs::JointState>> {
        KDL::JntArray q(chain.getNrOfJoints());
        auto error = ik_solver_pos.CartToJnt(q_current, convert_to<KDL::Frame>(m.pose), q);

        if (error < 0) {
            ROS_WARN_STREAM("IK failed: " << ik_solver_pos.strError(error));
            return {};
        }

        // TODO: TCP between grasper jaws, compute jaw joint angles

        auto dptr = q.data.data();

        // The first 6 elements of the solution vector is the robot configuration
        sensor_msgs::JointState m_robot;
        m_robot.header.stamp = ros::Time::now();
        std::copy(dptr, std::next(dptr, 6), std::back_inserter(m_robot.position));

        // and the last 4 is the tool configuration
        sensor_msgs::JointState m_tool;
        m_tool.header.stamp = m_robot.header.stamp;
        m_tool.name = {"roll", "wrist", "jaw1", "jaw2"}; // names matter to the eua_control node
        std::copy(std::next(dptr, 6), std::next(dptr, 9), std::back_inserter(m_tool.position));

        m_tool.position.push_back(math::radians(10)); // FIXME: Compute real grasper jaw angle

        return std::make_pair(m_robot, m_tool);
    };

    auto set_q = [&](const auto& m) {
        assert(!m.name.empty() && !m.position.empty());
        assert(m.name.size() == m.position.size());

        for (std::size_t i = 0; i < m.position.size(); ++i) {
            // *q_current_map.at(m.name[i]) = m.position[i];

            // FIXME: Ignore "unknown" joint names for now because 'tool_jaw2'
            // is ignored for now
            auto it = q_current_by_name.find(m.name[i]);

            if (it != q_current_by_name.end())
                *it->second = m.position[i];
        }
    };

    std::list<ros::Subscriber> subscribers{
        mksub<sensor_msgs::JointState>(nh, "ur/joint_states", 1, set_q, ros::TransportHints().tcpNoDelay()),
        mksub<sensor_msgs::JointState>(nh, "tool/joint_states", 1, set_q, ros::TransportHints().tcpNoDelay()),
        mksub<geometry_msgs::PoseStamped>(
            nh, "move_joint_ik", 1, [&](const auto& m) {
                // TODO: Interpolate path if set-point is far from current position
                auto s = solve_and_make_msgs(m);

                if (s) {
                    pub_robot_move_joint.publish(s->first);
                    pub_tool_move_joint.publish(s->second);
                }
            },
            ros::TransportHints().tcpNoDelay()),
        mksub<geometry_msgs::PoseStamped>(
            nh, "servo_joint_ik", 1, [&](const auto& m) {
                auto s = solve_and_make_msgs(m);

                if (s) {
                    pub_robot_servo_joint.publish(s->first);
                    pub_tool_servo_joint.publish(s->second);
                }
            },
            ros::TransportHints().tcpNoDelay()),
    };

    // Schedule timer to publish robot state
    auto timer = nh.createSteadyTimer(
        ros::WallDuration(1.0 / 100),
        [&](const auto&) {
            KDL::Frame f;
            fk_solver.JntToCart(q_current, f);

            geometry_msgs::PoseStamped m;
            m.header.stamp = ros::Time::now();
            m.header.frame_id = chain_root;
            m.pose = convert_to<geometry_msgs::Pose>(f);

            pub_pose.publish(m);
        });

    ros::spin();

    return EXIT_SUCCESS;
}
