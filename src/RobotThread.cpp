#include "RobotThread.h"

namespace server {
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

    wait();
}//end destructor

bool RobotThread::init()
{
    ros::init(m_Init_argc, m_pInit_argv, "gui_command");

    if (!ros::master::check())
        return false;//do not start without ros.

    ros::start();
    ros::Time::init();
    ros::NodeHandle nh;
    sim_velocity  = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 100);
    pose_listener = nh.subscribe(m_topic, 10, &RobotThread::poseCallback, this);
    return true;
}//set up the ros toys.

/** For a real robot **/
void RobotThread::poseCallback(nav_msgs::Odometry msg)
{
    m_xPos = msg.pose.pose.position.x;
    m_yPos = msg.pose.pose.position.y;
    m_aPos = msg.pose.pose.orientation.w;

    Q_EMIT newPose(m_xPos, m_yPos, m_aPos);
}//callback method to update the robot's position.

void RobotThread::run()
{
    ros::Rate loop_rate(100);
    while (ros::ok())
    {
        geometry_msgs::Twist cmd_msg;
        cmd_msg.linear.x = m_speed;
        cmd_msg.angular.z = m_angle;

        sim_velocity.publish(cmd_msg);
        ros::spinOnce();
        loop_rate.sleep();
    }//do ros things.
}

void RobotThread::SetSpeed(double speed, double angle)
{
    m_speed = speed;
    m_angle = angle;
}//set the speed of the robot.

double RobotThread::getXSpeed(){ return m_speed; }
double RobotThread::getASpeed(){ return m_angle; }

double RobotThread::getXPos(){ return m_xPos; }
double RobotThread::getYPos(){ return m_yPos; }
double RobotThread::getAPos(){ return m_aPos; }
}//end namespace

