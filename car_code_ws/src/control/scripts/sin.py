import casadi as ca
import numpy as np
import matplotlib.pyplot as plt

# sampling time T
T = 0.1
# simulation time n
n = 1000
# prediction horizon
Np = 5
# system parameters
A = np.array([[0, T, 0],[T, 0, T],[0, 0, 0]])
B = np.array([[T, 0],[0, 0],[0, T]])
A1 = A@np.diag([0, 0, 0.051]) + np.eye(3)
# terminal matrix
P = np.array([[63.1806,  -9.4404, -3.2900],[-5.8, 2072.7,64.644],[0.8785, 56.5868, 58.6687]])
# feedback gain
K = np.array([[-3.6621, 0.3378, -0.0517],[-0.3904, -3.3302, -3.4047]])
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

def sys_init(A,B,Q,R,Phi,Psi):
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

        At = np.append(At,np.linalg.matrix_power(A, i)).reshape(np.size(A,0)*i,np.size(A,1))

        Phit = np.append(Phit,np.linalg.matrix_power(Phi, i)).reshape(np.size(A,0)*i,np.size(A,1))

        Dis = np.append(Dis,np.linalg.matrix_power(Phi, i-1)).reshape(np.size(A,0)*i,np.size(A,1))

        Bt = np.hstack((Bt, np.zeros((np.size(Bt,0),np.size(B,1)))))
        if i==1:
            Bt = np.vstack((Bt, np.linalg.matrix_power(A, i-1)@B))
            temp = np.linalg.matrix_power(A, i-1)@B
        else:
            Bt = np.vstack((Bt, np.hstack((np.linalg.matrix_power(A, i-1)@B,temp))))
            temp = np.hstack((np.linalg.matrix_power(A, i-1)@B,temp))

        Kt = np.hstack((Kt, np.zeros((np.size(Kt,0),np.size(K,1)))))
        Kt = np.vstack((Kt,np.hstack((np.zeros((np.size(K,0),np.size(Kt,1)-np.size(K,1))),K))))

        Psit = np.hstack((Psit, np.zeros((np.size(Psit,0),np.size(Psi,1)))))
        Psit = np.vstack((Psit,np.hstack((np.zeros((np.size(Psi,0),np.size(Psit,0))),Psi))))

        Qt = np.hstack((Qt, np.zeros((np.size(Qt,0),np.size(Q,1)))))
        Qt = np.vstack((Qt,np.hstack((np.zeros((np.size(Q,0),np.size(Qt,0))),Q))))

        Rt = np.hstack((Rt, np.zeros(( np.size(Rt,0),np.size(R,1)))))
        Rt = np.vstack((Rt,np.hstack((np.zeros((np.size(R,0),np.size(Rt,0))),R))))

        Pt = np.hstack((Pt, np.zeros(( np.size(Pt,0),np.size(P,1)))))
        Pt = np.vstack((Pt,np.hstack((np.zeros((np.size(P,0),np.size(Pt,0))),P))))

    return At, Phit, Dis, Bt, Kt, Psit, Qt, Rt, Pt

# calculate alpha
def Cal_alpha(c_opt,Psit,Psi,beta):
    sc = c_opt.T@Psit@c_opt
    alpaha = sc - beta*c_opt[0:2].T@Psi@c_opt[0:2]
    return alpaha

def constranit_init(Phi,B,Np,desire_v,desire_w,desire_x,desire_y,desire_phi):
    # state model
    state_space = lambda x_, c_: Phi@x_ + B@c_[0:2]

    desire_input = np.array([desire_v, desire_w])
    desire_state = np.array([desire_x, desire_y, desire_phi])

    desire_input_seq = np.array([desire_input[0],desire_input[1],
                                 desire_input[0],desire_input[1],
                                 desire_input[0],desire_input[1],
                                 desire_input[0],desire_input[1],
                                 desire_input[0],desire_input[1]])
    # control vector init
    error = 0.0*np.ones(15)
    x_desire_state = np.array([desire_state[0],desire_state[1],desire_state[2],
                               desire_state[0],desire_state[1],desire_state[2],
                               desire_state[0],desire_state[1],desire_state[2],
                               desire_state[0],desire_state[1],desire_state[2],
                               desire_state[0],desire_state[1],desire_state[2]])
    x_ub = 10*np.ones(Np*np.size(Phi,1)) - error - x_desire_state
    x_lb = -10*np.ones(Np*np.size(Phi,1)) + error - x_desire_state
    u_ub = 0.6*np.ones(Np*np.size(B,1)) - desire_input_seq
    u_lb = -0.6*np.ones(Np*np.size(B,1)) - desire_input_seq
    return state_space, x_ub, x_lb, u_ub, u_lb

def update_ref(tao,Np,A,At,B,K):
    x = 0.1*tao
    y = 2*np.sin(0.1*tao)
    difx1_tao = 0.1
    difx2_tao = 0
    dify1_tao = 2*0.1*np.cos(0.1*tao)
    dify2_tao = 2*0.1*0.1*np.sin(0.1*tao)
    v = np.sqrt(difx1_tao**2 + dify1_tao**2)
    w = np.sqrt(difx2_tao*dify2_tao - difx2_tao*dify1_tao)/v**2

    ur = np.array([v,w])

    A_i = A*np.diag([-w,w,v]) + np.eye(3)

    phi = dify1_tao/difx1_tao

    Phi = A_i + B@K

    At_i = np.zeros((0,0))
    Phit = np.zeros((0,0))
    Dis = np.zeros((0,0))

    for i in range(1,Np+1):
        At_i = np.append(At_i,np.linalg.matrix_power(A_i, i)).reshape(np.size(A,0)*i,np.size(A,1))

        Phit = np.append(Phit,np.linalg.matrix_power(Phi, i)).reshape(np.size(A,0)*i,np.size(A,1))

        Dis = np.append(Dis,np.linalg.matrix_power(Phi, i-1)).reshape(np.size(A,0)*i,np.size(A,1))


    return A_i, At_i, v, w, ur, phi, Phi, Phit, Dis

def Cal_rotation(phi):
    rotation = np.array([np.cos(phi), -np.sin(phi), 0],
                        [np.sin(phi),  np.cos(phi), 0],
                        [0,0,1])
    return rotation


input_seq = np.zeros((2,0))
true_input = np.zeros((2,0))

# tao init
tao = 0
k_i = 0.1;
Z = np.array([0, 1]);

X_o_d = np.array([0.1*tao,2*np.sin(0.1*tao),np.arctan(np.cos(0.1*tao)/np.sin(0.1*tao))])

# initial error state
error_state_seq = np.zeros(3) - X_o_d
error_state = error_state_seq
true_state_seq = error_state + X_o_d

At, Phit, Dis, Bt, Kt, Psit, Qt, Rt, Pt = sys_init(A, B, Q, R, Phi, Psi)

print(Kt)

A_i, At_i, v, w, ur, phi, Phi, Phit, Dis = update_ref(tao,Np,A,At,B,K)

desire_input = np.array([v, w])
desire_state = np.array([X_o_d[0], X_o_d[1], X_o_d[2]])

state_space, x_ub, x_lb, u_ub, u_lb = constranit_init(Phi,B,Np,v, w, X_o_d[0], X_o_d[1], X_o_d[2])

state_space = lambda x_, c_: Phi@x_ + B@c_[0:2]

# free parameter
c = np.zeros((2,0))
c_seq = np.zeros((10,0))

opti = ca.Opti()
c_opt = opti.variable(10)

x = opti.parameter(3)
Phitx = opti.parameter(15,3)
Disx = opti.parameter(15,3)
x_lbx = opti.parameter(15,1)
x_ubx = opti.parameter(15,1)
u_lbx = opti.parameter(10,1)
u_ubx = opti.parameter(10,1)
alp = opti.parameter(1)
opti.set_value(x,error_state)
opti.set_value(Phitx,Phit)
opti.set_value(Disx,Dis)
opti.set_value(x_lbx,x_lb)
opti.set_value(x_ubx,x_ub)
opti.set_value(u_lbx,u_lb)
opti.set_value(u_ubx,u_ub)
opti.set_value(alp,alpha)
opti.minimize( c_opt.T@Psit@c_opt )
opti.subject_to( Phitx@x + Bt@c_opt >= x_lbx )
opti.subject_to( Phitx@x + Bt@c_opt <= x_ubx )
opti.subject_to( K@x + c_opt[0:2] <= u_ub[0:2] )
opti.subject_to( K@x + c_opt[0:2] >= u_lb[0:2] )
# opti.subject_to( Kt@(Disx@x + Bt@c_opt) + c_opt <= u_ub )
# opti.subject_to( Kt@(Disx@x + Bt@c_opt) + c_opt >= u_lb )
opti.subject_to( c_opt.T@Psit@c_opt <= alp )
opts_setting = {'ipopt.max_iter':1000, 'ipopt.print_level':0, 'print_time':0, 'ipopt.acceptable_tol':1e-6, 'ipopt.acceptable_obj_change_tol':1e-6}
opti.solver('ipopt',opts_setting)

for i in range(n):
    print(i)
    sol = opti.solve()

    cc = sol.value(c_opt)

    error_state = state_space(error_state, cc)

    alpha = Cal_alpha(cc,Psit,Psi,beta)

    X_o_d = np.array([0.1*tao,2*np.sin(0.1*tao),np.arctan(np.cos(0.1*tao)/np.sin(0.1*tao))])

    A_i, At_i, v, w, ur, phi, Phi, Phit, Dis = update_ref(tao,Np,A,At,B,K)

    desire_input = np.array([v, w])
    desire_state = np.array([X_o_d[0], X_o_d[1], X_o_d[2]])

    state_space, x_ub, x_lb, u_ub, u_lb = constranit_init(Phi,B,Np,v, w, X_o_d[0], X_o_d[1], X_o_d[2])

    opti.set_value(Phitx,Phit)
    opti.set_value(Disx,Dis)
    opti.set_value(x_lbx,x_lb)
    opti.set_value(x_ubx,x_ub)
    opti.set_value(u_lbx,u_lb)
    opti.set_value(u_ubx,u_ub)
    opti.set_value(x,error_state)
    opti.set_value(alp,alpha)

    error_input = K@error_state + cc[0:2]

    true_state = error_state + X_o_d

    tao = tao + 0.1

    input = K@error_state + cc[0:2] + desire_input

    cc = cc

    print(error_state)

    # storage of vectors
    error_state_seq = np.append(error_state_seq,error_state).reshape(i+2,3)

    true_state_seq = np.append(true_state_seq,true_state).reshape(i+2,3)

    c_seq = np.append(c_seq,cc).reshape(i+1,10)
    
    input_seq = np.append(input_seq,error_input).reshape(i+1,2)

    true_input = np.append(true_input,input).reshape(i+1,2)


# make data
time = np.linspace(0, n*T, n)
# 画图
fig = plt.figure()
# plot
plt.xlabel('x',fontsize=20)
plt.ylabel('y',fontsize=20)
plt.grid(True)
plt.plot(true_state_seq[:,0], true_state_seq[:,1], linewidth=2.0)

fig = plt.figure()
# plot
plt.xlabel('time/s',fontsize=20)
plt.ylabel('x',fontsize=20)
plt.grid(True)
plt.plot(time, true_state_seq[0:n,0], linewidth=2.0)

fig = plt.figure()
# plot
plt.xlabel('time/s',fontsize=20)
plt.ylabel('y',fontsize=20)
plt.grid(True)
plt.plot(time, true_state_seq[0:n,1], linewidth=2.0)

fig = plt.figure()
# plot
plt.xlabel('time/s',fontsize=20)
plt.ylabel('phi',fontsize=20)
plt.grid(True)
plt.plot(time, true_state_seq[0:n,2], linewidth=2.0)

fig = plt.figure()
# plot
plt.xlabel('time/s',fontsize=20)
plt.ylabel('Velocity',fontsize=20)
plt.grid(True)
plt.plot(time, true_input[:,0], linewidth=2.0)


fig = plt.figure()
# plot
plt.xlabel('time/s',fontsize=20)
plt.ylabel('Angular Velocity',fontsize=20)
plt.grid(True)
plt.plot(time, true_input[:,1], linewidth=2.0)

# fig = plt.figure()
# # plot
# plt.xlabel('time/s',fontsize=20)
# plt.ylabel('c Velocity',fontsize=20)
# plt.grid(True)
# plt.plot(time, c_seq[:,1], linewidth=2.0)

# fig = plt.figure()
# # plot
# plt.xlabel('time/s',fontsize=20)
# plt.ylabel('c Angular Velocity',fontsize=20)
# plt.grid(True)
# plt.plot(time, c_seq[:,1], linewidth=2.0)

plt.show()

