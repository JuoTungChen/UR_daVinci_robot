
#include "gui.h"

ClutchWidget::ClutchWidget(QWidget* parent)
    : QWidget(parent)
    , clutched_(false)
{
    setWindowTitle("Clutch");
    setBackgroundColor(Qt::red);
    setFocusPolicy(Qt::StrongFocus); // For widget to process keyboard events
}

void ClutchWidget::keyPressEvent(QKeyEvent* event)
{
    if (event->key() == Qt::Key_D && clutched_) {
        clutched_ = false;
        setDisengaged();
    } else if (event->key() == Qt::Key_E && !clutched_) {
        clutched_ = true;
        setEngaged();
    }

    QWidget::keyPressEvent(event);
}

void ClutchWidget::focusOutEvent(QFocusEvent* event)
{
    QWidget::focusOutEvent(event);
    setDisengaged();
}

void ClutchWidget::setBackgroundColor(Qt::GlobalColor color)
{
    QPalette pal = palette();
    pal.setColor(QPalette::Background, color);
    setPalette(pal);
}

void ClutchWidget::setEngaged()
{
    emit engaged();
    setBackgroundColor(Qt::green);
}

void ClutchWidget::setDisengaged()
{
    emit disengaged();
    setBackgroundColor(Qt::red);
}
