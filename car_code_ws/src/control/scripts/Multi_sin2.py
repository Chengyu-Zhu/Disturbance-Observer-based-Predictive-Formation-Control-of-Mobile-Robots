#! /usr/bin/env python3

import casadi as ca
import numpy as np
from dualmode_mpc import *
from plot import *
import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry



def doMsg(msg):
    global a
    global b
    global c
    a = msg.pose.pose.position.x
    b = msg.pose.pose.position.y
    c = msg.pose.pose.position.z/360*2*3.1415926535

rospy.init_node('control', anonymous=True)
rate = rospy.Rate(10)
turtle_vel_pub = rospy.Publisher('/motor_control', Twist, queue_size=1)
sub = rospy.Subscriber("/odomuwb",Odometry,doMsg,queue_size=10)

def main():
    state_seq = np.zeros((3,0))
    input_seq = np.zeros((2,0))
    # simulation time n
    n = 1000

    mpc1 = dualmode_mpc()

    mpc1.sys_para()

    input_seq = np.zeros((2,0))
    true_input = np.zeros((2,0))

    # tao init
    tao = 0
    k_i = 0.1
    Z = np.array([0, 1])

    X_o_d = np.array([0.1*tao,2*np.sin(0.1*tao)+3,np.arctan(2*0.1*np.cos(0.1*tao)/0.1)])


    # initial error state
    error_state_seq = np.zeros(3)
    mpc1.error_state = error_state_seq
    true_state_seq = mpc1.error_state + X_o_d

    mpc1.sys_init()
    mpc1.update_ref(tao)

    desire_input = np.array([mpc1.v, mpc1.w])
    desire_state = np.array([X_o_d[0], X_o_d[1], X_o_d[2]])

    mpc1.constraint_init(X_o_d[0], X_o_d[1], X_o_d[2])

    # free parameter
    c = np.zeros((2,0))
    c_seq = np.zeros((10,0))

    c_opt = mpc1.set_opt()

    a = X_o_d[0]
    b =  X_o_d[1]
    c = X_o_d[2]

    while True:
        total_timestart = rospy.Time.now()
        one_timestart = rospy.Time.now()

        state = np.array([a,b,c])

        mpc1.error_state =  desire_state - state

        total_timenow = rospy.Time.now()
        total_timelapse = total_timenow - total_timestart

        sol = mpc1.opti.solve()

        cc = sol.value(c_opt)

        mpc1.error_state = mpc1.Cal_error_state(cc)
        np.savetxt("error_state.txt",mpc1.error_state)

        alpha = mpc1.Cal_alpha(cc)

        X_o_d = np.array([0.1*tao,2*np.sin(0.1*tao)+3,np.arctan(2*0.1*np.cos(0.1*tao)/0.1)])

        mpc1.update_ref(tao)

        desire_input = np.array([mpc1.v, mpc1.w])
        desire_state = np.array([X_o_d[0], X_o_d[1], X_o_d[2]])

        mpc1.constraint_init(X_o_d[0], X_o_d[1], X_o_d[2])

        mpc1.update_opt()

        tao = tao + 0.1

        input = mpc1.Cal_true_input(desire_input)

        print(input)
    
        # 初始化geometry_msgs::Twist类型的消息
        vel_msg = Twist()
        vel_msg.linear.x =  input[0]
        vel_msg.angular.z = input[1]

        state_seq = np.append(state_seq,state)
        input_seq = np.append(input_seq,input)
        
        np.savetxt('state.txt',state)
        np.savetxt('control.txt',input_seq)

        # 发布消息
        turtle_vel_pub.publish(vel_msg)
        rospy.loginfo("Publsh turtle velocity command[%0.2f m/s, %0.2f rad/s]", vel_msg.linear.x, vel_msg.angular.z)
        rate.sleep()
        one_timestop = rospy.Time.now()
        one_timeduration = one_timestart - one_timestop
        if one_timeduration < rospy.Duration(secs=0.09):
            delay_time = rospy.Duration(secs=0.09) - one_timeduration
            rospy.sleep(delay_time)
            print(1)
        else:
            pass

    rospy.spin() 


if __name__ == "__main__":
    main()
