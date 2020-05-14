#pragma once

#include <QWidget>

class QKeyEvent;
class QFocusEvent;

class FootControlWidget : public QWidget
{
    Q_OBJECT

public:
    FootControlWidget(QWidget* parent = nullptr);

private:
    virtual void keyPressEvent(QKeyEvent* event) override;
    virtual void focusOutEvent(QFocusEvent* event) override;
    void setBackgroundColor(Qt::GlobalColor color);

signals:
    void clutchEngagedChanged(bool engaged);

private:
    void setClutchEngaged();
    void setClutchDisengaged();

private:
    bool clutch_engaged_;
};
