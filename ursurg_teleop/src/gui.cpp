
#include "gui.h"

ClutchWidget::ClutchWidget(QWidget* parent)
    : QWidget(parent)
    , clutch_engaged_(false)
{
    setWindowTitle("Clutch");
    setBackgroundColor(Qt::red);
    setFocusPolicy(Qt::StrongFocus); // For widget to process keyboard events
}

void ClutchWidget::keyPressEvent(QKeyEvent* event)
{
    if (event->key() == Qt::Key_D && clutch_engaged_) {
        clutch_engaged_ = false;
        setClutchDisengaged();
    } else if (event->key() == Qt::Key_E && !clutch_engaged_) {
        clutch_engaged_ = true;
        setClutchEngaged();
    }

    QWidget::keyPressEvent(event);
}

void ClutchWidget::focusOutEvent(QFocusEvent* event)
{
    QWidget::focusOutEvent(event);
    setClutchDisengaged();
}

void ClutchWidget::setBackgroundColor(Qt::GlobalColor color)
{
    QPalette pal = palette();
    pal.setColor(QPalette::Background, color);
    setPalette(pal);
}

void ClutchWidget::setClutchEngaged()
{
    emit engaged();
    setBackgroundColor(Qt::green);
}

void ClutchWidget::setClutchDisengaged()
{
    emit disengaged();
    setBackgroundColor(Qt::red);
}
