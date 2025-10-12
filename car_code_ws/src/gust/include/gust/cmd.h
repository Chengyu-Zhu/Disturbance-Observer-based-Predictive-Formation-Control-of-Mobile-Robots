
#ifndef __CMD_H_
#define __CMD_H_

#include <geometry_msgs/Twist.h>

#include <tf/tf.h>


class Cmd
{
    public:
        Cmd();
        ~Cmd();
        void cmd_move_control();
        double rightValue;
        double leftValue;

    private:
        ros::NodeHandle cmd_;
        ros::Subscriber cmd_sub = cmd_.subscribe<geometry_msgs::Twist>("/motor_control", 100, &Cmd::cmd_callback,this);
        
        void cmd_callback(const geometry_msgs::Twist::ConstPtr &msg);
        
        //cmd
        float cmd_xlinear, cmd_ylinear, cmd_zangular, cmd_degree;

        float vel_left = 15000;
        float vel_right = 15000;//左右轮串口校正值
    
};

#endif