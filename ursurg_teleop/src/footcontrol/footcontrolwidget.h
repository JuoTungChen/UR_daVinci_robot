#pragma once

#include <QWidget>

class QKeyEvent;
class QFocusEvent;
class QLabel;

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
    FootControlWidget(QWidget* parent = nullptr);

private:
    virtual void keyPressEvent(QKeyEvent* event) override;
    virtual void keyReleaseEvent(QKeyEvent* event) override;
    virtual void focusOutEvent(QFocusEvent* event) override;

signals:
    void pedalEngagedChanged(Pedal i, bool engaged);

private:
    void setPedalEngaged(Pedal i, bool engaged);

    std::array<bool, 3> engaged_;
    std::array<QLabel*, 3> label_;
};

