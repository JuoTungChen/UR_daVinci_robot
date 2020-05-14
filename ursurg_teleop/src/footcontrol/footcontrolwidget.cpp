#include "footcontrolwidget.h"

#include <QKeyEvent>

FootControlWidget::FootControlWidget(QWidget* parent)
    : QWidget(parent)
    , clutch_engaged_(false)
{
    setWindowTitle("Foot control");
    setBackgroundColor(Qt::red);
    setFocusPolicy(Qt::StrongFocus); // For widget to process keyboard events
}

void FootControlWidget::keyPressEvent(QKeyEvent* event)
{
    if (event->key() == Qt::Key_D && clutch_engaged_)
        setClutchDisengaged();
    else if (event->key() == Qt::Key_E && !clutch_engaged_)
        setClutchEngaged();

    QWidget::keyPressEvent(event);
}

void FootControlWidget::focusOutEvent(QFocusEvent* event)
{
    QWidget::focusOutEvent(event);
    setClutchDisengaged();
}

void FootControlWidget::setBackgroundColor(Qt::GlobalColor color)
{
    QPalette pal = palette();
    pal.setColor(QPalette::Background, color);
    setPalette(pal);
}

void FootControlWidget::setClutchEngaged()
{
    clutch_engaged_ = true;
    emit clutchEngagedChanged(true);
    setBackgroundColor(Qt::green);
}

void FootControlWidget::setClutchDisengaged()
{
    clutch_engaged_ = false;
    emit clutchEngagedChanged(false);
    setBackgroundColor(Qt::red);
}
