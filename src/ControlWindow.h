#ifndef CONTROL_WINDOW_H
#define CONTROL_WINDOW_H
#include <QMainWindow>
#include <QList>
#include <QFileDialog>
#include <QMessageBox>
#include <QtAlgorithms>
#include <QPushButton>
#include <QString>
#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QLabel>
#include <QTextStream>
#include <QLineEdit>
#include <QIcon>
#include "RobotThread.h"

namespace server{
#define PI 3.14159265359

class ControlWindow : public QWidget
{
    Q_OBJECT

public:
    ControlWindow(int argc, char** argv, QWidget * parent = 0);

    Q_SLOT void updatePoseDisplay(double x, double y, double theta);
private:
    Q_SLOT void goForward();
    Q_SLOT void goBackward();
    Q_SLOT void goLeft();
    Q_SLOT void goRight();
    Q_SLOT void halt();

    QPushButton *p_upButton;
    QPushButton *p_downButton;
    QPushButton *p_leftButton;
    QPushButton *p_rightButton;
    QPushButton *p_stopButton;
    QPushButton *p_quitButton;

    QVBoxLayout *rightLayout;
    QHBoxLayout *layout;
    QHBoxLayout *layout2;
    QHBoxLayout *layout3;
    QHBoxLayout *layout4;

    QVBoxLayout *leftLayout;
    QHBoxLayout *p_xLayout;
    QHBoxLayout *p_yLayout;
    QHBoxLayout *p_aLayout;

    QLabel *p_xLabel;
    QLabel *p_yLabel;
    QLabel *p_aLabel;
    QLabel *p_scanLabel;

    QLineEdit *p_xDisplay;
    QLineEdit *p_yDisplay;
    QLineEdit *p_aDisplay;

    QHBoxLayout *mainLayout;
    QPushButton *closeButton;

    RobotThread m_RobotThread;
};//end class ControlWindow
}//end namespace
#endif

