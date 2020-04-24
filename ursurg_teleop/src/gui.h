#pragma once

#include <QWidget>
#include <QKeyEvent>

#include <atomic>

class ClutchWidget : public QWidget
{
    Q_OBJECT

public:
    ClutchWidget(QWidget* parent = nullptr);

private:
    virtual void keyPressEvent(QKeyEvent* event) override;
    virtual void focusOutEvent(QFocusEvent* event) override;
    void setBackgroundColor(Qt::GlobalColor color);

signals:
    void engaged();
    void disengaged();

private:
    void setEngaged();
    void setDisengaged();

private:
    bool clutched_;
};
