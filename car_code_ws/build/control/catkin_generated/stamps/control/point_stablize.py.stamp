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
n = 200
# prediction horizon
Np = 5
# system parameters
A = np.array([[1,0],[0,1]])
B = np.array([[0.023,0.1063],[-0.023,-0.0063]])
# terminal matrix
P = np.array([[15.4202058796341,	2.46566037137417],[2.46566037137418,	38.5816035568261]])  
# feedback gain
K = np.array([[-0.699768631114821,	6.30721934780593],[-5.82955894566514,	-0.867781233548494]])
# weight matrix
Q = 5*np.eye(2)
R = 0.1*np.eye(2)
# stable constraint init
alpha = 100000
# stable constant
beta = 0.01
# feedback matrix
Phi = A + B@K
# control parameter matrix
Psi = R + B.T@P@B
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

    Phit = np.append(Phit,np.linalg.matrix_power(Phi, i)).reshape(2*i,2)

    Dis = np.append(Dis,np.linalg.matrix_power(Phi, i-1)).reshape(2*i,2)

    Bt = np.hstack((Bt, np.zeros((np.size(Bt,0),np.size(B,1)))))
    if i==1:
        Bt = np.vstack((Bt, np.linalg.matrix_power(A, i-1)@B))
        temp = np.linalg.matrix_power(A, i-1)@B
    else:
        Bt = np.vstack((Bt, np.hstack((np.linalg.matrix_power(A, i-1)@B,temp))))
        temp = np.hstack((np.linalg.matrix_power(A, i-1)@B,temp))
    
    Kt = np.hstack((Kt, np.zeros((np.size(Kt,0),np.size(K,1)))))
    Kt = np.vstack((Kt,np.hstack((np.zeros((np.size(K,0),np.size(Kt,0))),K))))

    Psit = np.hstack((Psit, np.zeros((np.size(Psit,0),np.size(Psi,1)))))
    Psit = np.vstack((Psit,np.hstack((np.zeros((np.size(Psi,0),np.size(Psit,0))),Psi))))

    Qt = np.hstack((Qt, np.zeros((np.size(Qt,0),np.size(Q,1)))))
    Qt = np.vstack((Qt,np.hstack((np.zeros((np.size(Q,0),np.size(Qt,0))),Q))))

    Rt = np.hstack((Rt, np.zeros(( np.size(Rt,0),np.size(R,1)))))
    Rt = np.vstack((Rt,np.hstack((np.zeros((np.size(R,0),np.size(Rt,0))),R))))

    Pt = np.hstack((Pt, np.zeros(( np.size(Pt,0),np.size(P,1)))))
    Pt = np.vstack((Pt,np.hstack((np.zeros((np.size(P,0),np.size(Pt,0))),P))))

# calculate alpha
def Cal_alpha(c_opt,Psit,Psi,beta):
    sc = c_opt.T@Psit@c_opt
    alpaha = sc - beta*c_opt[0:2].T@Psi@c_opt[0:2]
    return alpaha

# state model
state_space = lambda x_, c_: Phi@x_ + B@c_[0:2]

desire_input = np.array([0,0])
desire_state = np.array([0.25,-0.25])
desire_input_seq = np.array([0,0,0,0,0,0,0,0,0,0])

# control vector init
error = 0.05*np.ones(10)
x_ub = np.array([0.27, 0.64, 0.27, 0.64, 0.27, 0.64, 0.27, 0.64, 0.27, 0.64]) - error
x_lb = np.array([-0.77, -0.14, -0.77, -0.14, -0.77, -0.14, -0.77, -0.14, -0.77, -0.14]) + error
u_ub = 0.5*(np.array([1,1.8,1,1.8,1,1.8,1,1.8,1,1.8]) - desire_input_seq)
u_lb = -0.5*(np.array([1,1.8,1,1.8,1,1.8,1,1.8,1,1.8]) - desire_input_seq)

# initial state
state_seq = np.array([0.0,0.05])
state = state_seq

opti = ca.Opti()
c_opt = opti.variable(10)

x = opti.parameter(2)
alp = opti.parameter(1)
opti.set_value(x,state_seq)
opti.set_value(alp,alpha)
opti.minimize( c_opt.T@Psit@c_opt)
opti.subject_to( Phit@x + Bt@c_opt >= x_lb )
opti.subject_to( Phit@x + Bt@c_opt <= x_ub )
opti.subject_to( K@x + c_opt[0:2] <= u_ub[0:2] )
opti.subject_to( K@x + c_opt[0:2] >= u_lb[0:2] )
opti.subject_to( Kt@(Phit@x + Bt@c_opt) + c_opt <= u_ub )
opti.subject_to( Kt@(Phit@x + Bt@c_opt) + c_opt >= u_lb )
#opti.subject_to( c_opt.T@Psit@c_opt <= alp )
opts_setting = {'ipopt.max_iter':1000, 'ipopt.print_level':0, 'print_time':0, 'ipopt.acceptable_tol':1e-8, 'ipopt.acceptable_obj_change_tol':1e-6}
opti.solver('ipopt',opts_setting)


def doMsg(msg):
    mmsg = msg.data
    mmsg = mmsg.split(',')
    a = float(mmsg[0])
    b = float(mmsg[1])
    if a ==1000 and b ==1000:
        state   = np.loadtxt('predict_state.txt')
        print('predicted:')     
    else:
        state = np.array([a,b]) - desire_state

    print(state)
    # try:
    #     c_mem = np.zeros(10)
    #     sol = opti.solve()
    #     opti.debug.value    
    #     cc = sol.value(c_opt)
    #     c_mem = sol.value(c_opt)
    #     print(cc)
    # except:
    #     print("error")
    #     cc = c_mem
#    try:
#        sol = opti.solve()
#        cc = sol.value(c_opt)
#        constant = 1
#    except:
#        print('error')
#        cc= np.zeros(10)
#        constant = 0.5
    constant=1
    sol = opti.solve()
    cc = sol.value(c_opt)  

    state = state_space(state, cc)
    np.savetxt('predict_state.txt',state)

    opti.set_value(x,state)

    alpha = Cal_alpha(cc,Psit,Psi,beta)

    #opti.set_value(alp,alpha)

    input = K@state + cc[0:2] + desire_input

    # 初始化geometry_msgs::Twist类型的消息
    vel_msg = Twist()
    vel_msg.linear.x =  constant*input[0]
    vel_msg.angular.z = constant*input[1]
 
    # 发布消息
    turtle_vel_pub.publish(vel_msg)
    rospy.loginfo("Publsh turtle velocity command[%0.2f m/s, %0.2f rad/s]", vel_msg.linear.x, vel_msg.angular.z)

    # # 按照循环频率延时
    rate.sleep()

if __name__ == "__main__":
    # ROS节点初始化
    rospy.init_node('control', anonymous=True)
    rate = rospy.Rate(10)    
    turtle_vel_pub = rospy.Publisher('/cmd_vel_mux/input/navi', Twist, queue_size=1)
    sub = rospy.Subscriber("/image",String,doMsg,queue_size=10)

    rospy.spin()
