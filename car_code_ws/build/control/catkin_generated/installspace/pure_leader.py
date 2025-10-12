#! /usr/bin/env python3

import casadi as ca
import numpy as np
import matplotlib.pyplot as plt
import rospy
import time
from geometry_msgs.msg import Twist
from std_msgs.msg import String
import datetime, time

if __name__ == "__main__":
    
        
    # ROS节点初始化
    rospy.init_node('control', anonymous=True)
    rate = rospy.Rate(10)    
    turtle_vel_pub = rospy.Publisher('/motor_control', Twist, queue_size=1)
    vel_msg = Twist()
    vel_msg.linear.x =  0.0
    vel_msg.angular.z = 0.21
    total_timestart = rospy.Time.now()
    

    # 发布消息
    while(True):
        
        total_timenow = rospy.Time.now() 
        total_timelapse = total_timenow - total_timestart
        if total_timelapse < rospy.Duration(secs=314):
            vel_msg.linear.x =  0.5
            vel_msg.angular.z = 0.0
            turtle_vel_pub.publish(vel_msg)
        # elif total_timelapse < rospy.Duration(secs=91.5):
        #     vel_msg.linear.x =  0.25
        #     vel_msg.angular.z = -0.1
        #     turtle_vel_pub.publish(vel_msg)
        # elif total_timelapse < rospy.Duration(secs=121.7):
        #     vel_msg.linear.x =  0.25
        #     vel_msg.angular.z = -0.1
        #     turtle_vel_pub.publish(vel_msg)
        else:
            vel_msg.linear.x =  0.0
            vel_msg.angular.z = 0.0
            turtle_vel_pub.publish(vel_msg)
            break
        #turtle_vel_pub.publish(vel_msg)
        rospy.loginfo("Publsh turtle velocity command[%0.2f m/s, %0.2f rad/s]", vel_msg.linear.x, vel_msg.angular.z)
        rate.sleep()    

