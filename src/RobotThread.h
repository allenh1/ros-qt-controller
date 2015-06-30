#ifndef ___ROBOTTHREAD_H___
#define ___ROBOTTHREAD_H___

#include <QThread>
#include <QObject>
#include <QStringList>
#include <stdlib.h>
#include <iostream>
#include "assert.h"

#include <ros/ros.h>
#include <ros/network.h>
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>

namespace server {

class RobotThread : public QThread {
	Q_OBJECT
public:
    RobotThread(int argc, char **pArgv, const char * topic  = "/odom");
    virtual ~RobotThread();

    double getXPos();
    double getXSpeed();
    double getASpeed();
    double getYPos();
    double getAPos();

    bool init();

    void poseCallback(const nav_msgs::Odometry & msg);

	void SetSpeed(double speed, double angle);
    void setPose(QList<double> to_set);
    void run();

    Q_SIGNAL void newPose(double,double,double);
private:
    int m_Init_argc;
    char** m_pInit_argv;
    const char * m_topic;

    double m_speed;
    double m_angle;

    double m_xPos;
    double m_yPos;
    double m_aPos;

    double m_maxRange;
    double m_minRange;

    QList<double> ranges;

    ros::Subscriber pose_listener;
    ros::Subscriber scan_listener;
    ros::Publisher  sim_velocity;
};
}//end namespace
#endif

