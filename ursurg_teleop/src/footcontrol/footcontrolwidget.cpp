#include "footcontrolwidget.h"

#include <QHBoxLayout>
#include <QKeyEvent>
#include <QLabel>

FootControlWidget::FootControlWidget(QWidget* parent)
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
    resize(800, 500);
}

void FootControlWidget::keyPressEvent(QKeyEvent* event)
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

void FootControlWidget::focusOutEvent(QFocusEvent* event)
{
    for (auto i : {Left, Middle, Right})
        setPedalEngaged(i, false);

    QWidget::focusOutEvent(event);
}

void FootControlWidget::setPedalEngaged(Pedal i, bool engaged)
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
