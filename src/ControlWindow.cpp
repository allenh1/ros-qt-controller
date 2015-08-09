#include "ControlWindow.h"

namespace server{
ControlWindow::ControlWindow(int argc, char **argv, QWidget *parent)
    : QWidget(parent),
      m_RobotThread(argc, argv)
{
    /** Set up the Controls **/
    p_upButton = new QPushButton();
    p_downButton = new QPushButton();
    p_leftButton = new QPushButton();
    p_rightButton = new QPushButton();
    p_stopButton = new QPushButton(tr("&Stop"));
    p_quitButton = new QPushButton(tr("&Quit"));

    QPalette palette = p_rightButton->palette();
    palette.setColor(QPalette::Button,QColor(255,255,255));
    p_rightButton->setAutoFillBackground(true);
    p_rightButton->setFlat(true);
    p_rightButton->setPalette(palette);
    p_rightButton->setIcon(QIcon(":/images/right.xpm"));
    p_rightButton->setIconSize(QSize(50, 50));

    palette = p_leftButton->palette();
    palette.setColor(QPalette::Button,QColor(255,255,255));
    p_leftButton->setAutoFillBackground(true);
    p_leftButton->setFlat(true);
    p_leftButton->setPalette(palette);
    p_leftButton->setIcon(QIcon(":/images/left.xpm"));
    p_leftButton->setIconSize(QSize(50, 50));

    palette = p_upButton->palette();
    palette.setColor(QPalette::Button,QColor(255,255,255));
    p_upButton->setAutoFillBackground(true);
    p_upButton->setFlat(true);
    p_upButton->setPalette(palette);
    p_upButton->setIcon(QIcon(":/images/up.xpm"));
    p_upButton->setIconSize(QSize(50, 50));

    palette = p_downButton->palette();
    palette.setColor(QPalette::Button,QColor(255,255,255));
    p_downButton->setAutoFillBackground(true);
    p_downButton->setFlat(true);
    p_downButton->setPalette(palette);
    p_downButton->setIcon(QIcon(":/images/down.xpm"));
    p_downButton->setIconSize(QSize(50, 50));

    /** Set up the Position Display **/
    leftLayout = new QVBoxLayout();
    p_xLayout = new QHBoxLayout();
    p_yLayout = new QHBoxLayout();
    p_aLayout = new QHBoxLayout();

    p_xLabel = new QLabel();
    p_xLabel->setText(tr("X:"));
    p_xDisplay = new QLineEdit();
    p_xDisplay->setText(tr("0.0"));

    p_yLabel = new QLabel();
    p_yLabel->setText(tr("Y:"));
    p_yDisplay = new QLineEdit();
    p_yDisplay->setText(tr("0.0"));

    p_aLabel = new QLabel();
    p_aLabel->setText(tr("Theta: "));
    p_aDisplay = new QLineEdit();
    p_aDisplay->setText(tr("0.0"));

    p_xLayout->addWidget(p_xLabel);
    p_xLayout->addWidget(p_xDisplay);
    p_yLayout->addWidget(p_yLabel);
    p_yLayout->addWidget(p_yDisplay);
    p_aLayout->addWidget(p_aLabel);
    p_aLayout->addWidget(p_aDisplay);

    leftLayout->addLayout(p_xLayout);
    leftLayout->addLayout(p_yLayout);
    leftLayout->addLayout(p_aLayout);

    /** Set up the Layouts **/
    rightLayout = new QVBoxLayout();
    layout = new QHBoxLayout();
    layout2 = new QHBoxLayout();
    layout3 = new QHBoxLayout();
    layout4 = new QHBoxLayout();

    mainLayout = new QHBoxLayout();

    layout->addWidget(p_upButton);
    layout2->addWidget(p_leftButton);
    layout2->addWidget(p_stopButton);
    layout2->addWidget(p_rightButton);
    layout3->addWidget(p_downButton);
    layout4->addWidget(p_quitButton, 6);

    rightLayout->addLayout(layout);
    rightLayout->addLayout(layout2);
    rightLayout->addLayout(layout3);
    rightLayout->addLayout(layout4);

    mainLayout->addLayout(leftLayout);
    mainLayout->addLayout(rightLayout);
    setLayout(mainLayout);

    show();

    setWindowTitle(tr("Control Window"));

    connect(p_quitButton,  &QPushButton::clicked, this, &ControlWindow::close);
    connect(p_upButton,    &QPushButton::clicked, this, &ControlWindow::goForward);
    connect(p_leftButton,  &QPushButton::clicked, this, &ControlWindow::goLeft);
    connect(p_rightButton, &QPushButton::clicked, this, &ControlWindow::goRight);
    connect(p_downButton,  &QPushButton::clicked, this, &ControlWindow::goBackward);
    connect(p_stopButton,  &QPushButton::clicked, this, &ControlWindow::halt);
    
    connect(&m_RobotThread, &RobotThread::newPose, this, &ControlWindow::updatePoseDisplay);
    m_RobotThread.init();
}//end constructor

void ControlWindow::goForward(){m_RobotThread.SetSpeed(0.25, 0);}
void ControlWindow::goBackward(){m_RobotThread.SetSpeed(-0.25, 0);}
void ControlWindow::goRight(){m_RobotThread.SetSpeed(0, -PI / 6.0);}
void ControlWindow::goLeft(){m_RobotThread.SetSpeed(0, PI / 6.0);}
void ControlWindow::halt(){ m_RobotThread.SetSpeed(0, 0); }

void ControlWindow::updatePoseDisplay(double x, double y, double theta)
{
    QString xPose, yPose, aPose;
    xPose.setNum(x);
    yPose.setNum(y);
    aPose.setNum(theta);

    p_xDisplay->setText(xPose);
    p_yDisplay->setText(yPose);
    p_aDisplay->setText(aPose);
}//update the display.
}//end namespace

