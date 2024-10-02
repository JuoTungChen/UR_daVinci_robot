#include "geometry_msgs/msg/pose_stamped.hpp"
#include <geometry_msgs/msg/pose.hpp>

#include "sensor_msgs/msg/joint_state.hpp"
#include "mops_msgs/msg/tool_end_effector_state_stamped.hpp"
#include "mops_common/conversions/kdl.hpp"
#include "mops_common/mathutility.hpp"
#include "kdl/chainfksolverpos_recursive.hpp"
#include "kdl/chainiksolverpos_nr_jl.hpp"
#include "kdl/chainiksolvervel_wdls.hpp"
#include "kdl/tree.hpp"
#include "kdl_parser/kdl_parser.hpp"
#include "urdf/model.h"
#include "rclcpp/rclcpp.hpp"

#include <range/v3/action/push_back.hpp>
#include <range/v3/algorithm/copy.hpp>
#include <range/v3/view/enumerate.hpp>
#include <range/v3/view/span.hpp>
#include <range/v3/view/zip.hpp>

#include <optional>

#include <rcm/RCMSolver.hpp>

using namespace std::string_literals;

namespace mops_control {

struct Init
{
    bool ur;
    bool tool;
};

KDL::JntArray 
VectorToJnt(const Eigen::VectorXd& eigen_vector,int size) {
    KDL::JntArray jnt_array(size);

    for (int i = 0; i < size; ++i) {
        jnt_array(i) = eigen_vector(i);
    }

    return jnt_array;
}

Eigen::VectorXd
JntToVector(const KDL::JntArray& jnt_array, int size) {
    Eigen::VectorXd eigen_vector(size);
    for (int i = 0; i < size; ++i) {
        eigen_vector(i) = jnt_array(i);
    }
    return eigen_vector;
}

geometry_msgs::msg::Pose 
FrameToPoseMsg(const KDL::Frame& frame)
{
    geometry_msgs::msg::Pose pose_msg;

    // Extract translation components
    pose_msg.position.x = frame.p.x();
    pose_msg.position.y = frame.p.y();
    pose_msg.position.z = frame.p.z();

    // Extract orientation components
    double qx, qy, qz, qw;
    frame.M.GetQuaternion(qx, qy, qz, qw);
    pose_msg.orientation.x = qx;
    pose_msg.orientation.y = qy;
    pose_msg.orientation.z = qz;
    pose_msg.orientation.w = qw;

    return pose_msg;
}

KDL::Frame 
PoseMsgToFrame(const geometry_msgs::msg::Pose& pose_msg)
{
    // Extract translation components
    KDL::Vector translation(pose_msg.position.x, pose_msg.position.y, pose_msg.position.z);

    // Extract orientation components
    KDL::Rotation orientation;
    orientation.Quaternion(pose_msg.orientation.x, pose_msg.orientation.y, pose_msg.orientation.z, pose_msg.orientation.w);

    // Construct KDL::Frame
    KDL::Frame frame(orientation, translation);
    
    return frame;
}

void append_yaw0(const std::string& prefix, sensor_msgs::msg::JointState& m)
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

auto split_joint_state(const sensor_msgs::msg::JointState& m)
{
    sensor_msgs::msg::JointState m_ur;
    sensor_msgs::msg::JointState m_tool;

    m_ur.header.stamp = m.header.stamp;
    m_tool.header.stamp = m.header.stamp;

    for (auto [n, q] : ranges::views::zip(m.name, m.position)) {
        if (n.find("ur") != std::string::npos) {
            m_ur.name.push_back(n);
            m_ur.position.push_back(q);
        } else if (n.find("tool") != std::string::npos) {
            m_tool.name.push_back(n);
            m_tool.position.push_back(q);
        }
    }

    return std::make_pair(m_ur, m_tool);
}

class MopsControlNode : public rclcpp::Node
{
public:
    explicit MopsControlNode(const rclcpp::NodeOptions& options)
        : rclcpp::Node("mops_control", options)
    {
        auto xml = declare_parameter("robot_description", ""s);
        double k_task = declare_parameter("k_task", 1.5);
        double k_rcm = declare_parameter("k_rcm", 5.0);
        double k_p = declare_parameter("k_p", 1.0);
        double k_pos = declare_parameter("k_pos", 1.5);
        double k_ori = declare_parameter("k_ori", 1.5);
        double dt__ = declare_parameter("dt", 0.2);
        double thres = declare_parameter("thres", 1e-3);
        int max_iter__ = declare_parameter("max_iter", 1);
        double offset_ = declare_parameter("offset", 0.0);
        
        if (xml.empty())
            throw std::runtime_error("The 'robot_description' parameter is empty");

        if (!model_.initString(xml))
            throw std::runtime_error("Failed to initialize urdf::Model");


        if (!kdl_parser::treeFromUrdfModel(model_, tree_))
            throw std::runtime_error("Failed to initialize KDL::Tree");

        const auto ur_prefix = declare_parameter("ur_prefix", ""s);
        const auto tool_prefix = declare_parameter("tool_prefix", ""s);
        enable_rcm = declare_parameter("enable_rcm", false);
        // TODO: std::vector<std::string> ur_joint_names;

        for (const auto& name : {"roll", "pitch", "yaw1", "yaw2"})
            tool_joint_names_.push_back(tool_prefix + name);

        const auto chain_root = ur_prefix + "base_link";
        const auto chain_tip = tool_prefix + "tcp0";

        if (!tree_.getChain(chain_root, chain_tip, chain_))
            throw std::runtime_error("Failed to get kinematic chain between links: " + chain_root + ", " + chain_tip);

        // Map of joint names :-> limits
        for (const auto& joint : model_.joints_)
            if (joint.second->limits)
                joint_limits_[joint.second->name] = *joint.second->limits;

        // Vector of pointers to the movable joints in the kinematic chain
        const auto movable_joints = [&]() {
            std::vector<const KDL::Joint*> segments;

            for (const auto& seg : chain_.segments)
                if (seg.getJoint().getType() != KDL::Joint::None)
                    segments.push_back(&seg.getJoint());

            return segments;
        }();

        // Chain joint (movable joints) limits for input to IK solver
        const auto [q_min, q_max] = [&]() {
            KDL::JntArray lower(chain_.getNrOfJoints());
            KDL::JntArray upper(chain_.getNrOfJoints());

            for (const auto& [i, joint] : movable_joints | ranges::views::enumerate) {
                lower(i) = joint_limits_.at(joint->getName()).lower;
                upper(i) = joint_limits_.at(joint->getName()).upper;
            }

            return std::make_tuple(lower, upper);
        }();

        // Forward kinematics
        fk_solver_ = std::make_unique<KDL::ChainFkSolverPos_recursive>(chain_);

        // Inverse velocity kinematics
        ik_solver_vel_ = std::make_unique<KDL::ChainIkSolverVel_wdls>(chain_);

        auto weights_vector_js = declare_parameter("weights_joint_space", std::vector<double>(chain_.getNrOfJoints(), 1.0));
        Eigen::MatrixXd Mq = Eigen::VectorXd::Map(weights_vector_js.data(), weights_vector_js.size()).asDiagonal();

        if (auto err = ik_solver_vel_->setWeightJS(Mq); err != KDL::ChainIkSolverVel_wdls::E_NOERROR)
            RCLCPP_ERROR_STREAM(get_logger(), "Setting IK solver joint space weighting matrix failed: "
                                << ik_solver_vel_->strError(err)
                                << "\n" << "Mq =\n" << Mq);

        // Inverse position kinematics
        ik_solver_pos_ = std::make_unique<KDL::ChainIkSolverPos_NR_JL>(
            chain_,
            q_min,
            q_max,
            *fk_solver_,
            *ik_solver_vel_,
            declare_parameter("ik_pos_max_nr_itrs", 15),
            declare_parameter("ik_pos_epsilon", 1.0e-4));

        q_current_.resize(chain_.getNrOfJoints());
        q_desired_.resize(chain_.getNrOfJoints());

        init_ = {false, false};

        // Map to q_current by joint name
        for (const auto& [i, joint] : movable_joints | ranges::views::enumerate)
            q_current_by_name_.insert({joint->getName(), &q_current_(i)});
        
        // Also map grasper joint states (yaw1, yaw2) that are not part of 'chain'
        q_current_by_name_.insert({tool_joint_names_[2], &q_yaw_dummy_[0]});
        q_current_by_name_.insert({tool_joint_names_[3], &q_yaw_dummy_[1]});

        pub_robot_move_joint_ = create_publisher<sensor_msgs::msg::JointState>("ur/move_joint", 1);
        pub_tool_move_joint_ = create_publisher<sensor_msgs::msg::JointState>("tool/move_joint", 1);
        pub_robot_servo_joint_ = create_publisher<sensor_msgs::msg::JointState>("ur/servo_joint", 1);
        pub_tool_servo_joint_ = create_publisher<sensor_msgs::msg::JointState>("tool/servo_joint", 1);

        subscribers_ = {
            create_subscription<sensor_msgs::msg::JointState>("ur/joint_states", rclcpp::QoS(1).best_effort(),
                [this](sensor_msgs::msg::JointState::ConstSharedPtr m) {
                    // Cache current UR joint angles
                    for (auto [n, q] : ranges::views::zip(m->name, m->position))
                        *q_current_by_name_[n] = q;

                    init_.ur = true;
                }),
            create_subscription<sensor_msgs::msg::JointState>("tool/joint_states", rclcpp::QoS(1).best_effort(),
                [this, tool_prefix](sensor_msgs::msg::JointState::ConstSharedPtr m) {
                    sensor_msgs::msg::JointState m2 = *m;
                    append_yaw0(tool_prefix, m2);

                    // Cache current tool joint angles
                    for (auto [n, q] : ranges::views::zip(m2.name, m2.position))
                        *q_current_by_name_[n] = q;

                    init_.tool = true;
                }),
            create_subscription<sensor_msgs::msg::JointState>("servo_joint", rclcpp::QoS(1).best_effort(),
                [this](sensor_msgs::msg::JointState::ConstSharedPtr m) {
                    auto [m_ur, m_tool] = split_joint_state(*m);

                    if (!m_ur.name.empty())
                        pub_robot_servo_joint_->publish(m_ur);

                    if (!m_tool.name.empty())
                        pub_tool_servo_joint_->publish(m_tool);
                }),
            create_subscription<mops_msgs::msg::ToolEndEffectorState>("servo_joint_ik", rclcpp::QoS(1).best_effort(),
                [this](mops_msgs::msg::ToolEndEffectorState::ConstSharedPtr m) {
                    if (!init_.ur || !init_.tool)
                        return;

                    if (!initialized){
                        rcm_solver = std::make_shared<RCM::RCMSolver>(shared_from_this(), "robot_description");

                        Eigen::VectorXd q_vec = JntToVector(q_current_, chain_.getNrOfJoints());
                        rcm_solver->setDefaultRCM(q_vec);
                        initialized = true;
                    }
                    
                    if (enable_rcm){
                        auto sol = get_ik_solution_rcm(convert_to<KDL::Frame>(m->pose), m->grasper_angle, rcm_pose_);
                        if (sol) {
                            pub_robot_servo_joint_->publish(sol->first);
                            pub_tool_servo_joint_->publish(sol->second);
                        }
                    }
                    else{
                        auto sol = get_ik_solution(convert_to<KDL::Frame>(m->pose), m->grasper_angle);
                        if (sol) {
                            pub_robot_servo_joint_->publish(sol->first);
                            pub_tool_servo_joint_->publish(sol->second);
                        }
                    }
                }),
            create_subscription<sensor_msgs::msg::JointState>("move_joint", 1,
                [this](sensor_msgs::msg::JointState::ConstSharedPtr m) {
                    auto [m_ur, m_tool] = split_joint_state(*m);

                    if (!m_ur.name.empty())
                        pub_robot_move_joint_->publish(m_ur);

                    if (!m_tool.name.empty())
                        pub_tool_move_joint_->publish(m_tool);
                }),
            create_subscription<mops_msgs::msg::ToolEndEffectorStateStamped>("move_joint_ik", 1,
                [this](mops_msgs::msg::ToolEndEffectorStateStamped::ConstSharedPtr m) {
                    if (!init_.ur || !init_.tool)
                        return;

                    auto sol = get_ik_solution(convert_to<KDL::Frame>(m->ee.pose), m->ee.grasper_angle);

                    if (sol) {
                        pub_robot_move_joint_->publish(sol->first);
                        pub_tool_move_joint_->publish(sol->second);
                    }
                }),
            create_subscription<geometry_msgs::msg::Pose>("rcm_pose", rclcpp::QoS(1).best_effort(),
                [this](geometry_msgs::msg::Pose m) {
                    rcm_pose_ = m;
                }),
        };
    }

    // Get IK solution as joint states to push to robot and tool drivers, respectively
    std::optional<std::pair<sensor_msgs::msg::JointState, sensor_msgs::msg::JointState>>
    get_ik_solution(const KDL::Frame& pose_desired, double grasp_desired)
    {
        // If q_current and q_desired (the previous IK solution) are close,
        // we use q_desired as the seed for the IK algorithm
        auto q_init = ((q_desired_.data - q_current_.data).norm() > math::pi / 8) ? q_current_ : q_desired_;

        // Find configuration of joints in 'chain'
        if (auto err = ik_solver_pos_->CartToJnt(q_init, pose_desired, q_desired_);
                err != KDL::ChainIkSolverVel_wdls::E_NOERROR) {
            RCLCPP_WARN_STREAM(get_logger(), "IK error: " << ik_solver_pos_->strError(err));
            return {};
        }

        // yaw1, yaw2 configuration given a desired grasper opening angle
        double q_yaw1 = q_desired_(8) + grasp_desired / 2;
        double q_yaw2 = -q_desired_(8) + grasp_desired / 2;

        // Enforce joint limits for yaw1, yaw2
        const auto& lim_yaw1 = joint_limits_.at(tool_joint_names_[2]);
        q_yaw1 = std::clamp(q_yaw1, lim_yaw1.lower, lim_yaw1.upper);
        const auto& lim_yaw2 = joint_limits_.at(tool_joint_names_[3]);
        q_yaw2 = std::clamp(q_yaw2, lim_yaw2.lower, lim_yaw2.upper);

        // The first 6 elements of the solution vector is the robot configuration
        sensor_msgs::msg::JointState m_robot;
        m_robot.header.stamp = now();
        // FIXME joint names
        m_robot.position |= ranges::actions::push_back(ranges::span{q_desired_.data.data(), 6});

        // and the last 4 is the tool configuration
        sensor_msgs::msg::JointState m_tool;
        m_tool.header.stamp = m_robot.header.stamp;
        m_tool.name = tool_joint_names_;
        m_tool.position.push_back(q_desired_(6)); // roll
        m_tool.position.push_back(q_desired_(7)); // pitch (wrist)
        m_tool.position.push_back(q_yaw1);        // yaw1 (jaw1)
        m_tool.position.push_back(q_yaw2);        // yaw2 (jaw2)

        return std::make_pair(m_robot, m_tool);
    }

    std::optional<std::pair<sensor_msgs::msg::JointState, sensor_msgs::msg::JointState>>
    get_ik_solution_rcm(const KDL::Frame& pose_desired, double grasp_desired, const geometry_msgs::msg::Pose& rcm_pose)
    {
        // If q_current and q_desired (the previous IK solution) are close,
        // we use q_desired as the seed for the IK algorithm
        auto q_init = ((q_desired_.data - q_current_.data).norm() > math::pi / 8) ? q_current_ : q_desired_;

        // initialize q_vec and q_new
        Eigen::VectorXd q_vec = JntToVector(q_init, chain_.getNrOfJoints());   // convert JntArrary to VectorXd
        Eigen::VectorXd q_new(9);

        geometry_msgs::msg::Pose target_pose = FrameToPoseMsg(pose_desired);   // convert KDL::Frame to pose msg

        rcm_solver->setDesiredPose(target_pose); // retrieve target_pose from geometery_msgs
        q_new = rcm_solver->solveIK(q_vec);

        q_desired_ = VectorToJnt(q_new, 9);      // convert VectorXd (q_new) back to JntArrary (q_desired_)

        // yaw1, yaw2 configuration given a desired grasper opening angle
        double q_yaw1 = q_desired_(8) + grasp_desired / 2;
        double q_yaw2 = -q_desired_(8) + grasp_desired / 2;

        // Enforce joint limits for yaw1, yaw2
        const auto& lim_yaw1 = joint_limits_.at(tool_joint_names_[2]);
        q_yaw1 = std::clamp(q_yaw1, lim_yaw1.lower, lim_yaw1.upper);
        const auto& lim_yaw2 = joint_limits_.at(tool_joint_names_[3]);
        q_yaw2 = std::clamp(q_yaw2, lim_yaw2.lower, lim_yaw2.upper);

        // The first 6 elements of the solution vector is the robot configuration
        sensor_msgs::msg::JointState m_robot;
        m_robot.header.stamp = now();
        m_robot.position |= ranges::actions::push_back(ranges::span{q_desired_.data.data(), 6});

        // and the last 4 is the tool configuration
        sensor_msgs::msg::JointState m_tool;
        m_tool.header.stamp = m_robot.header.stamp;
        m_tool.name = tool_joint_names_;
        m_tool.position.push_back(q_desired_(6)); // roll
        m_tool.position.push_back(q_desired_(7)); // pitch (wrist)
        m_tool.position.push_back(q_yaw1);        // yaw1 (jaw1)
        m_tool.position.push_back(q_yaw2);        // yaw2 (jaw2)

        return std::make_pair(m_robot, m_tool);
    }


private:
    bool initialized = false;
    urdf::Model model_;
    KDL::Tree tree_;
    KDL::Chain chain_;
    std::vector<std::string> tool_joint_names_;
    std::unordered_map<std::string, urdf::JointLimits> joint_limits_;
    std::unordered_map<std::string, double*> q_current_by_name_;

    std::unique_ptr<KDL::ChainFkSolverPos_recursive> fk_solver_;
    std::unique_ptr<KDL::ChainIkSolverVel_wdls> ik_solver_vel_;
    std::unique_ptr<KDL::ChainIkSolverPos_NR_JL> ik_solver_pos_;
    std::shared_ptr<RCM::RCMSolver> rcm_solver;
    // Current joint state (from robot/tool drivers)
    KDL::JntArray q_current_;
    KDL::JntArray q_desired_;
    std::array<double, 2> q_yaw_dummy_; // grasper joints (yaw1, yaw2) are not part of 'chain'
    bool enable_rcm;
    // State signals
    Init init_;

    // rcm constraint pose
    geometry_msgs::msg::Pose rcm_pose_;

    std::list<rclcpp::SubscriptionBase::SharedPtr> subscribers_;
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr pub_robot_move_joint_;
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr pub_robot_servo_joint_;
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr pub_tool_move_joint_;
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr pub_tool_servo_joint_;
};

} // namespace mops_control

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(mops_control::MopsControlNode)
