#include "std_msgs/msg/bool.hpp"
#include "rclcpp/rclcpp.hpp"

#include <QApplication>
#include <QHBoxLayout>
#include <QKeyEvent>
#include <QLabel>
#include <QWidget>

#include <csignal>

std_msgs::msg::Bool boolMsg(bool value)
{
    std_msgs::msg::Bool m;
    m.data = value;
    return m;
}

enum Pedal : std::size_t
{
    Left = 0,
    Middle,
    Right,
};

class FootControlWidget : public QWidget
{
    Q_OBJECT

public:
    FootControlWidget(QWidget* parent = nullptr)
        : QWidget(parent)
    {
        setWindowTitle("Foot pedals");
        setFocusPolicy(Qt::StrongFocus); // For widget to process keyboard events
        engaged_.fill(false);
        auto layout = new QHBoxLayout();

        for (auto& l : label_) {
            l = new QLabel();
            l->setStyleSheet("background: red");
            layout->addWidget(l);
        }

        setLayout(layout);
        resize(300, 180);
    }

private:
    virtual void keyPressEvent(QKeyEvent* event) override
    {
        switch (event->key()) {
        case Qt::Key_A:
            setPedalEngaged(Pedal::Left, true);
            break;

        case Qt::Key_X:
            setPedalEngaged(Pedal::Left, false);
            break;

        case Qt::Key_B:
            setPedalEngaged(Pedal::Middle, true);
            break;

        case Qt::Key_Y:
            setPedalEngaged(Pedal::Middle, false);
            break;

        case Qt::Key_C:
            setPedalEngaged(Pedal::Right, true);
            break;

        case Qt::Key_Z:
            setPedalEngaged(Pedal::Right, false);
            break;

        default:
            QWidget::keyPressEvent(event);
            break;
        }
    }
    virtual void focusOutEvent(QFocusEvent* event) override
    {
        for (auto i : {Left, Middle, Right})
            setPedalEngaged(i, false);

        QWidget::focusOutEvent(event);
    }

signals:
    void pedalEngagedChanged(Pedal i, bool engaged);

private:
    void setPedalEngaged(Pedal i, bool engaged)
    {
        if (engaged_[i] == engaged)
            return;

        engaged_[i] = engaged;
        emit pedalEngagedChanged(i, engaged);

        if (engaged) {
            label_[i]->setStyleSheet("background: green");
        } else {
            label_[i]->setStyleSheet("background: red");
        }
    }

    std::array<bool, 3> engaged_;
    std::array<QLabel*, 3> label_;
};

int main(int argc, char* argv[])
{
    QApplication app(argc, argv);

    rclcpp::init(argc, argv);
    auto node = std::make_shared<rclcpp::Node>("footcontrol");

    // SIGINT quits the Qt event loop
    std::signal(SIGINT, [](int) { QApplication::quit(); });

    std::array<rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr, 3> publishers = {
        node->create_publisher<std_msgs::msg::Bool>("left", rclcpp::QoS(10).transient_local()),
        node->create_publisher<std_msgs::msg::Bool>("middle", rclcpp::QoS(10).transient_local()),
        node->create_publisher<std_msgs::msg::Bool>("right", rclcpp::QoS(10).transient_local()),
    };

    // Publish initial value (latched topic)
    for (auto& pub : publishers)
        pub->publish(boolMsg(false));

    FootControlWidget widget;
    QObject::connect(&widget, &FootControlWidget::pedalEngagedChanged, [&](Pedal i, bool engaged) {
        publishers[i]->publish(boolMsg(engaged));
    });
    widget.show();

    return app.exec();
}

#include "footcontrol.moc"
