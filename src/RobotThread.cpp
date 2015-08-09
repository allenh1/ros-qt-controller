#include "RobotThread.h"

RobotThread::RobotThread(int argc, char** pArgv, const char * topic)
    :	m_Init_argc(argc),
        m_pInit_argv(pArgv),
        m_topic(topic)
{/** Constructor for the robot thread **/}

RobotThread::~RobotThread()
{
    if (ros::isStarted())
    {
        ros::shutdown();
        ros::waitForShutdown();
    }//end if

    m_pThread->wait();
}//end destructor

bool RobotThread::init()
{
    m_pThread = new QThread();
    this->moveToThread(m_pThread);

    connect(m_pThread, &QThread::started, this, &RobotThread::run);
    ros::init(m_Init_argc, m_pInit_argv, "gui_command");

    if (!ros::master::check())
        return false;//do not start without ros.

    ros::start();
    ros::Time::init();
    ros::NodeHandle nh;
    sim_velocity  = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 100);
    pose_listener = nh.subscribe(m_topic, 10, &RobotThread::poseCallback, this);

    m_pThread->start();
    return true;
}//set up the thread

void RobotThread::poseCallback(const nav_msgs::Odometry & msg)
{
    QMutex * pMutex = new QMutex();

    pMutex->lock();
    m_xPos = msg.pose.pose.position.x;
    m_yPos = msg.pose.pose.position.y;
    m_aPos = msg.pose.pose.orientation.w;
    pMutex->unlock();

    delete pMutex;
    Q_EMIT newPose(m_xPos, m_yPos, m_aPos);
}//callback method to update the robot's position.

void RobotThread::run()
{
    ros::Rate loop_rate(100);
    QMutex * pMutex;
    while (ros::ok())
    {
        pMutex = new QMutex();

        geometry_msgs::Twist cmd_msg;
        pMutex->lock();
        cmd_msg.linear.x = m_speed;
        cmd_msg.angular.z = m_angle;
        pMutex->unlock();

        sim_velocity.publish(cmd_msg);
        ros::spinOnce();
        loop_rate.sleep();
        delete pMutex;
    }//do ros things.
}

void RobotThread::SetSpeed(double speed, double angle)
{
    QMutex * pMutex = new QMutex();
    pMutex->lock();
    m_speed = speed;
    m_angle = angle;
    pMutex->unlock();

    delete pMutex;
}//set the speed of the robot.

double RobotThread::getXSpeed(){ return m_speed; }
double RobotThread::getASpeed(){ return m_angle; }

double RobotThread::getXPos(){ return m_xPos; }
double RobotThread::getYPos(){ return m_yPos; }
double RobotThread::getAPos(){ return m_aPos; }

