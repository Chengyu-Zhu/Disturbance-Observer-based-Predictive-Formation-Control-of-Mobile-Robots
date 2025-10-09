import casadi as ca
import numpy as np
import matplotlib.pyplot as plt
import rospy
from geometry_msgs.msg import Twist

# sampling time T
T = 0.1

# simulation time n
n = 1000

# prediction horizon
Np = 5

# system parameters
A = np.array([[1,0],[0,1]])
B = np.array([[0.023,0.1063],[0.023,0.0063]])

# terminal matrix
P = np.array([[57.2,4.8],[6.23, 164.53]])

# feedback gain
K = np.array([[-0.7175,-2.49],[-3.887,0.15127]])

# weight matrix
Q = 10*np.eye(2)
R = 1*np.eye(2)

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

# memorier of state vector init
x_hat = np.zeros((2,1))

www = np.eye(10)
# control vector init
x_ub = np.array([0.27, 0.64, 0.27, 0.64, 0.27, 0.64, 0.27, 0.64, 0.27, 0.64])
x_lb = np.array([-0.77, -0.14, -0.77, -0.14, -0.77, -0.14, -0.77, -0.14, -0.77, -0.14])
u_ub = np.array([0.5,1,0.5,1,0.5,1,0.5,1,0.5,1])
u_lb = -np.array([0.5,1,0.5,1,0.5,1,0.5,1,0.5,1])
u_u = np.array([0.5,1,0.5,1,0.5,1,0.5,1,0.5,1])
u_l = -np.array([0.5,1,0.5,1,0.5,1,0.5,1,0.5,1])

# free parameter
c = np.zeros((2,0))
c_seq = np.zeros((10,0))

desire_input = np.array([0.2,0])
input_seq = np.zeros((2,0))
true_input = np.zeros((2,0))

# initial state
state_seq = np.array([0.25,-0.2025])
state = state_seq

# desire state define
desire_state = np.array([0.25, 0.3375])
true_state = state + desire_state
# formation error
formation_error = np.array([-2, 1])

# absolute error
f_error = np.linalg.norm(formation_error)

opti = ca.Opti()
c_opt = opti.variable(10)

x = opti.parameter(2)
opti.set_value(x,state_seq)
opti.minimize( c_opt.T@Psit@c_opt)
opti.subject_to( Phit@x + Bt@c_opt >= x_lb )
opti.subject_to( Phit@x + Bt@c_opt <= x_ub )
opti.subject_to( K@x + c_opt[0:2] <= u_ub[0:2] )
opti.subject_to( K@x + c_opt[0:2] >= u_lb[0:2] )
opti.subject_to( Kt@(Phit@x + Bt@c_opt) + c_opt <= u_ub )
opti.subject_to( Kt@(Phit@x + Bt@c_opt) + c_opt >= u_lb )
opti.subject_to( www@(Kt@(Phit@x + Bt@c_opt) + c_opt )<= u_ub )
opti.subject_to( www@(Kt@(Phit@x + Bt@c_opt) + c_opt )>= u_lb )
opti.subject_to( c_opt.T@Psit@c_opt <= alpha )
opti.solver('ipopt')


def main():
    # ROS节点初始化
    rospy.init_node('control', anonymous=True)

	# 创建一个Publisher，发布名为/cmd_vel的topic，消息类型为geometry_msgs::Twist，队列长度10
    turtle_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)

	#设置循环的频率
    rate = rospy.Rate(10) 
    i = 0
    while not rospy.is_shutdown():
    #for i in range(n):
        i = i + 1

        sol = opti.solve()

        state = state_space(state, sol.value(c_opt))

        opti.set_value(x,state)

        input = K@state + sol.value(c_opt[0:2])

        alpha = Cal_alpha(sol.value(c_opt),Psit,Psi,beta)

        # storage of vectors
        state_seq = np.append(state_seq,state).reshape(i+2,2)

        true_state = np.append(true_state,state + desire_state).reshape(i+2,2)

        c_seq = np.append(c_seq,sol.value(c_opt)).reshape(i+1,10)
        
        input_seq = np.append(input_seq,input).reshape(i+1,2)

        true_input = np.append(true_input,K@state + sol.value(c_opt[0:2]) + desire_input).reshape(i+1,2)

        # 初始化geometry_msgs::Twist类型的消息
        vel_msg = Twist()
        vel_msg.linear.x = input[0]
        vel_msg.angular.z = input[1]

		# 发布消息
        turtle_vel_pub.publish(vel_msg)
        rospy.loginfo("Publsh turtle velocity command[%0.2f m/s, %0.2f rad/s]", vel_msg.linear.x, vel_msg.angular.z)

        # 按照循环频率延时
        rate.sleep()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
         pass
    finally:
        pass
