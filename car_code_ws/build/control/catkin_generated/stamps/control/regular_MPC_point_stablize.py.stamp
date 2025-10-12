#! /usr/bin/env python3

import casadi as ca
import numpy as np
import matplotlib.pyplot as plt
import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import String

# sampling time T
T = 0.1
# simulation time n
n = 500
# prediction horizon
Np = 5
# system parameters
A = np.array([[1,0],[0,1]])
B = np.array([[0.023,0.1063],[-0.023,-0.0063]])
# terminal matrix
P = np.array([[15.4202058796341,	2.46566037137417],[2.46566037137418,	38.5816035568261]])  
# weight matrix
Q = 30*np.eye(2)
R = 1*np.eye(2)
# matrix initialization
At = np.zeros((0,0))
Bt = np.zeros((0,0))
Kt = np.zeros((0,0))
Phit = np.zeros((0,0))
Dis = np.zeros((0,0))
temp = np.zeros((0,0))
Qt = np.zeros((0,0))
Rt = np.zeros((0,0))
Pt = np.zeros((0,0))
Psit = np.zeros((0,0))
# 加权矩阵的计算过程，以及推导方程矩阵的叠加过程
for i in range(1,Np+1):

    At = np.append(At,np.linalg.matrix_power(A, i)).reshape(2*i,2)

    Bt = np.hstack((Bt, np.zeros((np.size(Bt,0),np.size(B,1)))))
    if i==1:
        Bt = np.vstack((Bt, np.linalg.matrix_power(A, i-1)@B))
        temp = np.linalg.matrix_power(A, i-1)@B
    else:
        Bt = np.vstack((Bt, np.hstack((np.linalg.matrix_power(A, i-1)@B,temp))))
        temp = np.hstack((np.linalg.matrix_power(A, i-1)@B,temp))
    
    Qt = np.hstack((Qt, np.zeros((np.size(Qt,0),np.size(Q,1)))))
    Qt = np.vstack((Qt,np.hstack((np.zeros((np.size(Q,0),np.size(Qt,0))),Q))))

    Rt = np.hstack((Rt, np.zeros(( np.size(Rt,0),np.size(R,1)))))
    Rt = np.vstack((Rt,np.hstack((np.zeros((np.size(R,0),np.size(Rt,0))),R))))

    Pt = np.hstack((Pt, np.zeros(( np.size(Pt,0),np.size(P,1)))))
    Pt = np.vstack((Pt,np.hstack((np.zeros((np.size(P,0),np.size(Pt,0))),P))))

# state model
state_space = lambda x_, u_: A@x_ + B@u_[0:2]

desire_input = np.array([0,0])
desire_input_seq = np.array([0,0,0,0,0,0,0,0,0,0])

# control vector init
error = 0.03*np.ones(10)
x_ub = np.array([0.27, 0.64, 0.27, 0.64, 0.27, 0.64, 0.27, 0.64, 0.27, 0.64]) - error
x_lb = np.array([-0.77, -0.14, -0.77, -0.14, -0.77, -0.14, -0.77, -0.14, -0.77, -0.14]) + error
u_ub = 0.5*np.ones(10) - desire_input_seq
u_lb = -0.5*np.ones(10) - desire_input_seq

# initial state
desire_state = np.array([0.25, -0.25])

opti = ca.Opti()
u_opt = opti.variable(10)

x = opti.parameter(2)
opti.set_value(x,desire_state)
opti.minimize( (At@x + Bt@u_opt).T@Qt@(At@x + Bt@u_opt) +  u_opt.T@Rt@u_opt)
opti.subject_to( At@x + Bt@u_opt >= x_lb )
opti.subject_to( At@x + Bt@u_opt <= x_ub )
opti.subject_to( opti.bounded(u_lb,u_opt,u_ub) )
opts_setting = {'ipopt.max_iter':100, 'ipopt.print_level':0, 'print_time':0, 'ipopt.acceptable_tol':1e-8, 'ipopt.acceptable_obj_change_tol':1e-6}
opti.solver('ipopt',opts_setting)


def doMsg(msg):
    mmsg = msg.data
    mmsg = mmsg.split(',')
    a = float(mmsg[0])
    b = float(mmsg[1])
    error_state = np.array([a, b]) - desire_state
    print(error_state)

    sol = opti.solve()
    cc = sol.value(u_opt)

    error_state = state_space(error_state, cc)

    opti.set_value(x,error_state)

    input = cc[0:2] + desire_input

    # 初始化geometry_msgs::Twist类型的消息
    vel_msg = Twist()
    vel_msg.linear.x = input[0]
    vel_msg.angular.z = input[1]

    # 发布消息
    turtle_vel_pub.publish(vel_msg)
    rospy.loginfo("Publsh turtle velocity command[%0.2f m/s, %0.2f rad/s]", vel_msg.linear.x, vel_msg.angular.z)

    # # 按照循环频率延时
    # rate.sleep()

if __name__ == "__main__":
    #设置循环的频率
    # ROS节点初始化
    rospy.init_node('control', anonymous=True)
    rate = rospy.Rate(10)
    turtle_vel_pub = rospy.Publisher('/cmd_vel_mux/input/navi', Twist, queue_size=10)
    sub = rospy.Subscriber("/image",String,doMsg,queue_size=10)

    rospy.spin()
