#include "common.h"
#include "gui.h"
#include "haptics.h"
#include "robot.h"

#include <ursurg_common/math.h>
#include <ursurg_common/synchronized.h>

#include <geometry_msgs/PoseStamped.h>

#include <ros/ros.h>

#include <QApplication>

#include <Eigen/Geometry>

#include <chrono>
#include <csignal>
#include <optional>
#include <thread>

using namespace std::string_literals;

// Class to join wrapped std::thread upon destruction
struct Thread
{
    template<typename Callable, typename... Args>
    explicit Thread(Callable&& f, Args&&... args)
        : thread_(std::forward<Callable>(f), std::forward<Args>(args)...)
    {
    }

    ~Thread()
    {
        if (thread_.joinable())
            thread_.join();
    }

    std::thread thread_;
};

class TeleopController
{
public:
    using haptics_state_getter_t = std::function<HapticsState()>;
    using robot_state_getter_t = std::function<RobotState()>;

    TeleopController(haptics_state_getter_t a,
                     robot_state_getter_t b,
                     double grasp_rate,
                     double scaling_factor)
        : get_haptics_state_(a)
        , get_robot_state_(b)
        , clutched_(false)
        , grasp_rate_(grasp_rate)
        , translation_scaling_factor_(scaling_factor)
    {
        t_hd_base = Eigen::Isometry3d::Identity();
        // clang-format off
        t_hd_base.linear() << 1,  0,  0,
                              0,  0,  1,
                              0, -1,  0;
        // clang-format on
        t_base_hd = t_hd_base.inverse();
    }

    void engage()
    {
        haptics_state_curr_ = get_haptics_state_();
        robot_state_desired_ = get_robot_state_();
        clutched_ = true;
    }

    void disengage()
    {
        clutched_ = false;
    }

    std::optional<Eigen::Isometry3d> step()
    {
        if (!clutched_)
            return {};

        haptics_state_prev_ = haptics_state_curr_;
        haptics_state_curr_ = get_haptics_state_();

        HapticsStateDiff diff = haptics_state_curr_ - haptics_state_prev_;

        Eigen::Isometry3d t_incr = t_base_hd * diff.tf * t_hd_base;
        t_incr.translation() *= translation_scaling_factor_;
        robot_state_desired_.tf.translation() += t_incr.translation();
        robot_state_desired_.tf.linear() = t_incr.linear() * robot_state_desired_.tf.linear();

        // TODO grasp angle get/set
        double grasp_incr = 0;

        if ((haptics_state_curr_.button_state.grey == ButtonState::PRESSED)
            && (haptics_state_curr_.button_state.white == ButtonState::RELEASED)) {
            // Decrease grasp angle
            grasp_incr = -grasp_rate_ * diff.dt.count();
        } else if ((haptics_state_curr_.button_state.grey == ButtonState::RELEASED)
                   && (haptics_state_curr_.button_state.white == ButtonState::PRESSED)) {
            // Increase grasp angle
            grasp_incr = grasp_rate_ * diff.dt.count();
        }

        return t_incr;
    }

private:
    haptics_state_getter_t get_haptics_state_;
    robot_state_getter_t get_robot_state_;

    std::atomic<bool> clutched_; // set asynchronously

    double grasp_rate_;
    double translation_scaling_factor_;

    Eigen::Isometry3d t_hd_base;
    Eigen::Isometry3d t_base_hd;

    RobotState robot_state_desired_;
    HapticsState haptics_state_curr_;
    HapticsState haptics_state_prev_;
};

int main(int argc, char* argv[])
{
    using namespace std::string_literals;

    QApplication app(argc, argv);

    ros::init(argc, argv, "ursurg_teleop", ros::init_options::NoSigintHandler);

    // SIGINT quits the Qt main event loop
    std::signal(SIGINT, [](int) { QApplication::quit(); });

    // Shutdown ROS stuff when Qt is quitting the main event loop
    QObject::connect(&app, &QApplication::aboutToQuit, ros::shutdown);

    ros::NodeHandle nh;
    ros::NodeHandle nh_priv("~");

    ros::NodeHandle nh_left(nh, nh_priv.param("ns_left", "left"s));
    ros::NodeHandle nh_right(nh, nh_priv.param("ns_right", "right"s));

    HapticsStateReader haptics(nh_priv.param("device_name_left", "left"s),
                               nh_priv.param("device_name_right", "right"s),
                               nh_priv.param("scheduler_rate", 500));

    RobotStateReader robot(nh_left, nh_right);

    ros::Publisher pub_servo_left = nh_left.advertise<geometry_msgs::PoseStamped>("servo_j_ik", 1);
    ros::Publisher pub_servo_right = nh_right.advertise<geometry_msgs::PoseStamped>("servo_j_ik", 1);

    ros::Publisher pub_haptics_left = nh_left.advertise<geometry_msgs::PoseStamped>("pose_haptics_current", 1);
    ros::Publisher pub_haptics_right = nh_right.advertise<geometry_msgs::PoseStamped>("pose_haptics_current", 1);

    TeleopController ctrl_left([&]() { return haptics.currentState(L); },
                               [&]() { return robot.currentState(L); },
                               nh_priv.param("grasp_rate", math::pi / 4),
                               nh_priv.param("translation_scaling_factor", 1.0));
    TeleopController ctrl_right([&]() { return haptics.currentState(R); },
                                [&]() { return robot.currentState(R); },
                                nh_priv.param("grasp_rate", math::pi / 4),
                                nh_priv.param("translation_scaling_factor", 1.0));

    ClutchWidget clutch_widget;

    QObject::connect(&clutch_widget, &ClutchWidget::engaged, [&]() {
        ctrl_right.engage();
        ctrl_left.engage();
    });
    QObject::connect(&clutch_widget, &ClutchWidget::disengaged, [&]() {
        ctrl_left.disengage();
        ctrl_right.disengage();
    });

    std::list<ros::SteadyTimer> timers{
        nh.createSteadyTimer(ros::WallDuration(1.0 / 500),
                             [&](const auto&) {
                                 auto ret = ctrl_left.step();

                                 if (!ret)
                                     return;

                                 geometry_msgs::PoseStamped m;
                                 m.header.stamp = ros::Time::now();
                                 m.pose = convert(*ret);
                                 pub_servo_left.publish(m);
                             }),
        nh.createSteadyTimer(ros::WallDuration(1.0 / 125),
                             [&](const auto&) {
                                 auto ret = ctrl_right.step();

                                 if (!ret)
                                     return;

                                 geometry_msgs::PoseStamped m;
                                 m.header.stamp = ros::Time::now();
                                 m.pose = convert(*ret);
                                 pub_servo_right.publish(m);
                             }),
    };

    if (nh_priv.param("publish_haptics_poses", false)) {
        timers.push_back(
            nh.createSteadyTimer(ros::WallDuration(1.0 / 50),
                                 [&](const auto&) {
                                     geometry_msgs::PoseStamped m;
                                     m.header.stamp = ros::Time::now();
                                     m.pose = convert(haptics.currentState(L).tf);
                                     pub_haptics_left.publish(m);
                                 }));
        timers.push_back(
            nh.createSteadyTimer(ros::WallDuration(1.0 / 50),
                                 [&](const auto&) {
                                     geometry_msgs::PoseStamped m;
                                     m.header.stamp = ros::Time::now();
                                     m.pose = convert(haptics.currentState(R).tf);
                                     pub_haptics_right.publish(m);
                                 }));
    }

    clutch_widget.show();

    Thread ros_spin_thread([] { ros::spin(); });

    return app.exec(); // Qt event loop
}
