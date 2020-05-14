#include "footcontrolwidget.h"

#include <std_msgs/Bool.h>

#include <ros/ros.h>

#include <QApplication>

#include <csignal>

std_msgs::Bool boolMsg(bool value)
{
    std_msgs::Bool m;
    m.data = value;
    return m;
}

int main(int argc, char* argv[])
{
    QApplication app(argc, argv);

    ros::init(argc, argv, "footcontrol", ros::init_options::NoSigintHandler);
    ros::NodeHandle nh;

    // SIGINT quits the Qt event loop
    std::signal(SIGINT, [](int) { QApplication::quit(); });

    auto pub = nh.advertise<std_msgs::Bool>("clutch_engaged", 4, true);
    pub.publish(boolMsg(false)); // Publish initial value (latched topic)

    FootControlWidget widget;
    QObject::connect(&widget, &FootControlWidget::clutchEngagedChanged, [&](bool engaged) {
        pub.publish(boolMsg(engaged));
    });
    widget.show();

    return app.exec();
}
