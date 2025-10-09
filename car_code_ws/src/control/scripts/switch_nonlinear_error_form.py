#! /usr/bin/env python3

import casadi as ca
import numpy as np
import matplotlib.pyplot as plt
import datetime, time
import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import String

# sampling time T
T = 0.1
# simulation time n
n = 1000
# prediction horizon
Np = 5
# system parameters
A = np.array([[1,0],[0.004,1]])
B = np.array([[0,0.1],[0.1,0]])
# # terminal matrix
# P = np.array([[62.6020,	-14.1888],[-12.4907, 159.8502]])
# # feedback gain
# K = np.array([[-0.3405,	2.2968],[-4.2941, 1.0880]])
# terminal matrix
P = np.array([[12.31889111,  0.03094296],[0.03094296, 12.31868356]])

# feedback gain
K = np.array([[-0.02589772, -5.02853059],[-5.02859592, -0.00578362]])
# weight matrix
Q = 5*np.eye(2)
R = 0.1*np.eye(2)
# relative height = camera height - feature height
height = 0.04
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

desire_input = np.array([0.2,0.2])
desire_input_seq = np.array([desire_input[0],desire_input[1],desire_input[0],desire_input[1],desire_input[0],
                             desire_input[1],desire_input[0],desire_input[1],desire_input[0],desire_input[1]])

# initial state
state_seq = np.array([0,0])
state = state_seq

# desire state define
desire_state = np.array([0, 0.2])
true_state = state + desire_state

# control vector init
desire_state_seq = np.array([desire_state[0],desire_state[1],desire_state[0],desire_state[1],desire_state[0],
                                desire_state[1],desire_state[0],desire_state[1],desire_state[0],desire_state[1]])
x_ub = np.array([0.52, 0.39, 0.52, 0.39, 0.52, 0.39, 0.52, 0.39, 0.52, 0.39]) - desire_state_seq
x_lb = -1*np.array([0.52, 0.39, 0.52, 0.39, 0.52, 0.39, 0.52, 0.39, 0.52, 0.39]) - desire_state_seq
u_ub = 1*np.ones(10) - desire_input_seq
u_lb = -1*np.ones(10) - desire_input_seq

# dicerete nonlinear error state model
fxe = lambda x, u, x_r, u_r: T*(-u_r[1]*(x_r[0]**2 + 1) + (u[1]+u_r[1])*((x[0]+x_r[0])**2 +1) - x[0]*x[1]*u_r[0]/height - desire_input[0]/height*(-x_r[0]*x_r[1]*np.cos(x_r[0])-x_r[1]*np.sin(x_r[0])) 
                                + desire_input[0]/height*(-(x[0]+x_r[0])*(x[1]+x_r[1])*np.cos(x[0])-(x[1]+x_r[1])*np.sin(x[0])) + (x[0]+x_r[0])*(x[1]+x_r[1])*(u[0]+u_r[0])/height) + x[0]
gxe = lambda x, u, x_r, u_r: T*(-x_r[0]*x_r[1]*u_r[1] + (x[0]+x_r[0])*(x[1]+x_r[1])*(u[1]+u_r[1]) - x_r[1]**2*u_r[0]/height + x_r[1]**2*u_r[0]*np.cos(x_r[0])/height
                                 - (x[1]+x_r[1])**2*np.cos(x_r[0])*desire_input[0]/height + (x[1]+x_r[1])**2*(u[0]+u_r[0])/height) + x[1]
state_space = lambda x_, u_, x_r, u_r: np.array([fxe(x_,u_,x_r,u_r),gxe(x_,u_,x_r,u_r)])

# casadi dicerete nonlinear error state model
ffxe = lambda x, u, x_r, u_r: T*(-u_r[1]*(x_r[0]**2 + 1) + (u[1]+u_r[1])*((x[0]+x_r[0])**2 +1) - x[0]*x[1]*u_r[0]/height - desire_input[0]/height*(-x_r[0]*x_r[1]*ca.cos(x_r[0])-x_r[1]*ca.sin(x_r[0])) 
                                + desire_input[0]/height*(-(x[0]+x_r[0])*(x[1]+x_r[1])*ca.cos(x[0])-(x[1]+x_r[1])*ca.sin(x[0])) + (x[0]+x_r[0])*(x[1]+x_r[1])*(u[0]+u_r[0])/height) + x[0]
ggxe = lambda x, u, x_r, u_r: T*(-x_r[0]*x_r[1]*u_r[1] + (x[0]+x_r[0])*(x[1]+x_r[1])*(u[1]+u_r[1]) - x_r[1]**2*u_r[0]/height + x_r[1]**2*u_r[0]*ca.cos(x_r[0])/height
                                 - (x[1]+x_r[1])**2*ca.cos(x_r[0])*desire_input[0]/height + (x[1]+x_r[1])**2*(u[0]+u_r[0])/height) + x[1]
ca_state_space = lambda x_, u_, x_r, u_r: ca.vertcat(ffxe(x_,u_,x_r,u_r),ggxe(x_,u_,x_r,u_r))

# linear state model
linear_state_space = lambda x_, c_: Phi@x_ + B@c_[0:2]

# calculate upper tao
def ca_upper_tao(x):
    tao = ca.if_else(ca.le(ca.norm_2(x),1), 1, ca.exp( -1*ca.norm_2(x) + 1))
    return tao

# calculate upper tao
def upper_tao(x):
    if np.linalg.norm(x) <= 1:
        tao = 1
    else:
        tao = np.exp(-1*np.linalg.norm(x) + 1)
    return tao

# prediction
def prediction(x,c_opt,desire_input,desire_state):
    c_count=0
    x_seq = np.zeros(0)
    for i in range(5):
        u = K@x + ca_upper_tao(x)*c_opt[c_count:c_count+2]
        x = ca_state_space(x,u,desire_input,desire_state)
        x_seq = ca.vertcat(x_seq,x)
        c_count = c_count + 2
    return x_seq

def control_sequence(x,c_opt,desire_input,desire_state):
    c_count=0
    u_seq = np.zeros(0)
    for i in range(5):
        u = K@x + ca_upper_tao(x)*c_opt[c_count:c_count+2]
        x = ca_state_space(x,u,desire_input,desire_state)
        u_seq = ca.vertcat(u_seq,u)
        c_count = c_count + 2
    return u_seq

# free parameter
c = np.zeros((2,0))
c_seq = np.zeros((10,0))

# nonlinear solver
nlp_opti = ca.Opti()
c_opt = nlp_opti.variable(10)

x = nlp_opti.parameter(2)
alp = nlp_opti.parameter(1)
nlp_opti.set_value(x,state)
nlp_opti.set_value(alp,alpha)
nlp_opti.minimize( c_opt.T@Psit@c_opt )
nlp_opti.subject_to( prediction(x,c_opt,desire_input,desire_state) >= x_lb )
nlp_opti.subject_to( prediction(x,c_opt,desire_input,desire_state) <= x_ub )
nlp_opti.subject_to( control_sequence(x,c_opt,desire_input,desire_state) <= u_ub )
nlp_opti.subject_to( control_sequence(x,c_opt,desire_input,desire_state) >= u_lb )
nlp_opti.subject_to( c_opt.T@Psit@c_opt <= alp )
opts_setting = {'ipopt.max_iter':100, 'ipopt.print_level':2, 'print_time':0, 'ipopt.acceptable_tol':1e-8, 'ipopt.acceptable_obj_change_tol':1e-6}
nlp_opti.solver('ipopt',opts_setting)

# linear solver
opti = ca.Opti()
c_opt_linear = opti.variable(10)

x_linear = opti.parameter(2)
alp_linear = opti.parameter(1)
opti.set_value(x_linear,state)
opti.set_value(alp_linear,alpha)
opti.minimize( c_opt_linear.T@Psit@c_opt_linear )
opti.subject_to( Phit@x_linear + Bt@c_opt_linear >= x_lb )
opti.subject_to( Phit@x_linear + Bt@c_opt_linear <= x_ub )
opti.subject_to( K@x_linear + c_opt_linear[0:2] <= u_ub[0:2] )
opti.subject_to( K@x_linear + c_opt_linear[0:2] >= u_lb[0:2] )
opti.subject_to( Kt@(Phit@x_linear + Bt@c_opt_linear) + c_opt_linear <= u_ub )
opti.subject_to( Kt@(Phit@x_linear + Bt@c_opt_linear) + c_opt_linear >= u_lb )
opti.subject_to( c_opt_linear.T@Psit@c_opt_linear <= alp_linear )
opts_setting = {'ipopt.max_iter':100, 'ipopt.print_level':0, 'print_time':0, 'ipopt.acceptable_tol':1e-8, 'ipopt.acceptable_obj_change_tol':1e-6}
opti.solver('ipopt',opts_setting)

# hh
mmsg = '0,0.2,0'

def doMsg(msg):
    global mmsg
    mmsg = msg.data
    #rospy.loginfo(rospy.get_caller_id() + "I heard %s", msg.data)

rospy.init_node('control', anonymous=True)
rate = rospy.Rate(15)    
turtle_vel_pub = rospy.Publisher('/motor_control', Twist, queue_size=1)
sub = rospy.Subscriber("/image",String,doMsg,queue_size=10)

def main():
    while True:
        point = mmsg.split(',')
        a = float(point[0])
        b = float(point[1])
        # icount = float(mmsg[2])
        if a ==1000 and b ==1000:
            state   = np.loadtxt('predict_state.txt')
            print('predicted:')
        else:
            state = np.array([a,b]) - desire_state
        #print(point)

        if np.linalg.norm(state) >-1.0:
            sol = nlp_opti.solve()
            cc = sol.value(c_opt)

            up_tao = upper_tao(state)

            delta_input = K@state + up_tao*cc[0:2]
            #print(delta_input)
            print(state)
            state = state_space(state, delta_input,desire_input,desire_state)
            np.savetxt('predict_state.txt',state)            

            nlp_opti.set_value(x,state)
            opti.set_value(x_linear,state)            

            alpha = Cal_alpha(cc,Psit,Psi,beta)
            
            nlp_opti.set_value(alp,alpha)

            input = delta_input + desire_input

            cc = cc
        else:
            sol = opti.solve()
            cc = sol.value(c_opt_linear)

            #print(delta_input)
            print(state)
            state = linear_state_space(state, cc)
            np.savetxt('predict_state.txt',state)            

            opti.set_value(x_linear,state)

            alpha = Cal_alpha(cc,Psit,Psi,beta)
            
            opti.set_value(alp_linear,alpha)

            input = delta_input + desire_input

            cc = cc

        # 初始化geometry_msgs::Twist类型的消息
        vel_msg = Twist()
        vel_msg.linear.x =  input[0]
        vel_msg.angular.z = input[1]
    
        # 发布消息
        turtle_vel_pub.publish(vel_msg)
        rospy.loginfo("Publsh turtle velocity command[%0.2f m/s, %0.2f rad/s]", vel_msg.linear.x, vel_msg.angular.z)
        #rate.sleep()

    rospy.spin()

if __name__ == "__main__":
    main()
