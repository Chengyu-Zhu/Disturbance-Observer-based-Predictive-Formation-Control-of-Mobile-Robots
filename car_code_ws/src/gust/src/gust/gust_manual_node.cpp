
#include <ros/ros.h>
#include <ros/time.h>
#include <thread>

#include "can/motor_init.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "Gust"); 
    
    motor_init mi;

    int a;

    ros::Rate loop_rate(10);
    while (ros::ok())
    {  
        
        mi.Set_Velocity(2600,2600);
        
        
        
        a = a + 1 ;
        if(a == 24)
        {
            break;
        }
        
        ros::spinOnce();
        loop_rate.sleep();

    }

    return 0;
}