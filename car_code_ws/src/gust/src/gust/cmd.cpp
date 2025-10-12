#include "gust/cmd.h"


Cmd::Cmd()
{
    cmd_xlinear = 0;
    cmd_ylinear = 0;
    cmd_zangular = 0;

}

Cmd::~Cmd()
{
    cmd_xlinear = 0;
    cmd_ylinear = 0;
    cmd_zangular = 0;

}


void Cmd::cmd_callback(const geometry_msgs::Twist::ConstPtr &msg)
{
    cmd_xlinear = msg->linear.x/0.96;
    cmd_ylinear = msg->linear.y;
    cmd_zangular = msg->angular.z*0.875;
}


void Cmd::cmd_move_control()
{
    leftValue =  (cmd_xlinear  - cmd_zangular * 0.2 ) * vel_left;
    rightValue = (cmd_xlinear  + cmd_zangular * 0.2 ) * vel_right;
}

