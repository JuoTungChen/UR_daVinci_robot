/*
 * Raven2 - Software for the Raven II surgical robot
 * Copyright (C) 2016-2017 Kim Lindberg Schwaner <kils@mmmi.sdu.dk>
 *
 * This file is part of Raven2.
 *
 * Raven2 is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Lesser General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * Raven2 is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public License
 * along with Raven2.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <std_msgs/Bool.h>

#include <ros/ros.h>

#include <QApplication>
#include <QWidget>
#include <QKeyEvent>

#include <csignal>

class Window : public QWidget
{
    Q_OBJECT

public:
    Window(QWidget* parent = nullptr)
        : QWidget(parent)
        , nh_()
        , pub_clutch_(nh_.advertise<std_msgs::Bool>("clutch", 1, true))
    {
        setWindowTitle("Clutch");
        setFocusPolicy(Qt::StrongFocus); // For widget to process keyboard events
        disengage();
    }

private:
    virtual void keyPressEvent(QKeyEvent* event) override
    {
        if (!event->isAutoRepeat()) {
            if (event->key() == Qt::Key_D) {
                disengage();
            } else if (event->key() == Qt::Key_E) {
                engage();
            }
        }

        QWidget::keyPressEvent(event);
    }

    virtual void focusOutEvent(QFocusEvent* event) override
    {
        disengage();
        QWidget::focusOutEvent(event);
    }

    void setBackgroundColor(Qt::GlobalColor color)
    {
        QPalette pal = palette();
        pal.setColor(QPalette::Background, color);
        setPalette(pal);
    }

    void engage()
    {
        std_msgs::Bool m;
        m.data = true;
        pub_clutch_.publish(m);
        setBackgroundColor(Qt::green);
    }

    void disengage()
    {
        std_msgs::Bool m;
        m.data = false;
        pub_clutch_.publish(m);
        setBackgroundColor(Qt::yellow);
    }

private:
    ros::NodeHandle nh_;
    ros::Publisher pub_clutch_;
};

int main(int argc, char* argv[])
{
    QApplication app(argc, argv);
    ros::init(argc, argv, "footpedal", ros::init_options::NoSigintHandler);

    // SIGINT quits the Qt app
    std::signal(SIGINT, [](int) {
        ros::shutdown();
        QApplication::quit();
    });

    Window window;
    window.show();

    return app.exec(); // Qt event loop
}

#include "footpedal.moc"
