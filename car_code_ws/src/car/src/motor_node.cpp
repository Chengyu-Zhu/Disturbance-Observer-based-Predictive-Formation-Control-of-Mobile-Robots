#include <ros/ros.h>
#include "serial/serial.h"
#include <stdio.h>
#include <geometry_msgs/Wrench.h>
#include <iostream>
#include "geometry_msgs/Twist.h"
#include "motor.h"
#include "std_msgs/String.h"
using namespace std;
using namespace motor;

CMotorInitialized* motor_r= new CMotorInitialized(0);
CMotorInitialized* motor_l= new CMotorInitialized(1);

void call_back(const geometry_msgs::Twist::ConstPtr &p)
{
    float v = p->linear.x;
    float w = p->angular.z; 
    //cout << "tor " << tor_l << "   " << tor_r  << endl;
    float v_left = v + 0.30/2*w;
    float v_right = v - 0.30/2*w;
    motor_l->setVel(-v_left);
    motor_r->setVel(+v_right);
    static int count = 0;
    count++;
    ROS_INFO("次数%d",count);
    
    //motor->setVel(0.171,-0.229);
};


void motor_back(const std_msgs::String::ConstPtr& msg)
{
     ROS_INFO("I heard: [%s]", msg->data.c_str());
}

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "hch_Gust");
    ros::NodeHandle nh;
    ros::Subscriber sub = nh.subscribe<geometry_msgs::Twist>("/motor_control", 1000, call_back);
    //::Subscriber sub = nh.subscribe<std_msgs::String>("/motor_control", 1000, motor_back);
    motor_l->setMotorMode(2);
    motor_r->setMotorMode(2);
    std::cout << "start" << std::endl;
    ros::Rate loop_rate(50);
    while (ros::ok())
    {
        ros::spinOnce();
        //loop_rate.sleep();
    }
    delete motor_l;
    //delete motor_r;
    std::cout << "close" << std::endl;
    return 0;
}