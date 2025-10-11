import numpy as np
from sqp_qcqp import solve_qcqp

class DualMPC:
    def __init__(self, T=0.1, Np=5):
        self.T = T
        self.Np = Np
        self.sys_para()
        self.sys_init()
        self.error_state = np.zeros(3)

    def sys_para(self):
        self.A = np.array([[0, self.T, 0],[self.T, 0, self.T],[0, 0, 0]])
        self.B = np.array([[self.T, 0],[0, 0],[0, self.T]])
        self.A1 = self.A@np.diag([0, 0, 0.051]) + np.eye(3)
        self.P = np.array([[63.1806,  -9.4404, -3.2900],[-5.8, 2072.7,64.644],[0.8785, 56.5868, 58.6687]])
        self.K = np.array([[-3.6621, 0.3378, -0.0517],[-0.3904, -3.3302, -3.4047]])
        self.Q = 10*np.eye(2)
        self.R = 1*np.eye(2)
        self.alpha = 100000
        self.beta = 0.01
        self.Phi = self.A + self.B@self.K
        self.Psi = self.R + self.B.T@self.P@self.B

    def sys_init(self):
        Np = self.Np
        A, B, K, Phi, Q, R, P, Psi = self.A, self.B, self.K, self.Phi, self.Q, self.R, self.P, self.Psi
        self.At = np.zeros((0,0))
        self.Bt = np.zeros((0,0))
        self.Kt = np.zeros((0,0))
        self.Phit = np.zeros((0,0))
        self.Dis = np.zeros((0,0))
        temp = np.zeros((0,0))
        self.Qt = np.zeros((0,0))
        self.Rt = np.zeros((0,0))
        self.Pt = np.zeros((0,0))
        self.Psit = np.zeros((0,0))
        for i in range(1,Np+1):
            self.At = np.append(self.At,np.linalg.matrix_power(A, i)).reshape(np.size(A,0)*i,np.size(A,1))
            self.Phit = np.append(self.Phit,np.linalg.matrix_power(Phi, i)).reshape(np.size(A,0)*i,np.size(A,1))
            self.Dis = np.append(self.Dis,np.linalg.matrix_power(Phi, i-1)).reshape(np.size(A,0)*i,np.size(A,1))
            self.Bt = np.hstack((self.Bt, np.zeros((np.size(self.Bt,0),np.size(B,1)))))
            if i==1:
                self.Bt = np.vstack((self.Bt, np.linalg.matrix_power(A, i-1)@B))
                temp = np.linalg.matrix_power(A, i-1)@B
            else:
                self.Bt = np.vstack((self.Bt, np.hstack((np.linalg.matrix_power(A, i-1)@B,temp))))
                temp = np.hstack((np.linalg.matrix_power(A, i-1)@B,temp))
            self.Kt = np.hstack((self.Kt, np.zeros((np.size(self.Kt,0),np.size(K,1)))))
            self.Kt = np.vstack((self.Kt,np.hstack((np.zeros((np.size(K,0),np.size(self.Kt,1)-np.size(K,1))),K))))
            self.Psit = np.hstack((self.Psit, np.zeros((np.size(self.Psit,0),np.size(Psi,1)))))
            self.Psit = np.vstack((self.Psit,np.hstack((np.zeros((np.size(Psi,0),np.size(self.Psit,0))),Psi))))
            self.Qt = np.hstack((self.Qt, np.zeros((np.size(self.Qt,0),np.size(Q,1)))))
            self.Qt = np.vstack((self.Qt,np.hstack((np.zeros((np.size(Q,0),np.size(self.Qt,0))),Q))))
            self.Rt = np.hstack((self.Rt, np.zeros(( np.size(self.Rt,0),np.size(R,1)))))
            self.Rt = np.vstack((self.Rt,np.hstack((np.zeros((np.size(R,0),np.size(self.Rt,0))),R))))
            self.Pt = np.hstack((self.Pt, np.zeros(( np.size(self.Pt,0),np.size(P,1)))))
            self.Pt = np.vstack((self.Pt,np.hstack((np.zeros((np.size(P,0),np.size(self.Pt,0))),P))))

    def constraint_init(self,desire_v,desire_w,desire_x,desire_y,desire_phi):
        Phi, B, Np = self.Phi, self.B, self.Np
        state_space = lambda x_, c_: Phi@x_ + B@c_[0:2]
        desire_input = np.array([desire_v, desire_w])
        desire_state = np.array([desire_x, desire_y, desire_phi])
        desire_input_seq = np.array([desire_input[0],desire_input[1],
                                     desire_input[0],desire_input[1],
                                     desire_input[0],desire_input[1],
                                     desire_input[0],desire_input[1],
                                     desire_input[0],desire_input[1]])
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

    def mpc_step(self, error_state, Phit, Bt, Psit, K, x_lb, x_ub, u_lb, u_ub):
        H = Psit
        f = np.zeros(10)
        A_list = []
        lb_list = []
        ub_list = []
        if Bt.shape[0] > 0:
            A_list.append(Bt)
            lb_list.append(x_lb - Phit @ error_state)
            ub_list.append(x_ub - Phit @ error_state)
        A_in = np.zeros((4,10))
        A_in[0,0] = 1
        A_in[1,1] = 1
        A_in[2,0] = -1
        A_in[3,1] = -1
        b_in = np.hstack([u_ub[0:2] - K @ error_state, -(u_lb[0:2] - K @ error_state)])
        A_list.append(A_in)
        lb_list.append(np.full(4, -np.inf))
        ub_list.append(b_in)
        if len(A_list) > 0:
            A_qp = np.vstack(A_list)
            lb_qp = np.hstack(lb_list)
            ub_qp = np.hstack(ub_list)
            # 只用线性约束
            Q_list = []
            a_list = []
            b_list = []
        else:
            Q_list = []
            a_list = []
            b_list = []
        x0 = np.zeros(10)
        x_opt, fval_opt, _ = solve_qcqp(H, f, Q_list, a_list, b_list, x0=x0, max_iter=20, tol=1e-6, plot_history=False)
        return x_opt
