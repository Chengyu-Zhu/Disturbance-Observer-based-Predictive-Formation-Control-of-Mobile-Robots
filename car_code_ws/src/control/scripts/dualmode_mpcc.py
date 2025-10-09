import casadi as ca
import numpy as np

class dualmode_mpc:
    # sampling time T
    T = 0.1
    # prediction horizon
    Np = 5

    opti = ca.Opti()

    error_state = np.zeros(3)

    # system parameters
    def sys_para(self):
        self.A = np.array([[0, self.T, 0],[self.T, 0, self.T],[0, 0, 0]])
        self.B = np.array([[self.T, 0],[0, 0],[0, self.T]])
        self.A1 = self.A@np.diag([0, 0, 0.051]) + np.eye(3)
        # terminal matrix
        self.P = np.array([[63.1806,  -9.4404, -3.2900],[-5.8, 2072.7,64.644],[0.8785, 56.5868, 58.6687]])
        # feedback gain
        self.K = np.array([[-3.6621, 0.3378, -0.0517],[-0.3904, -3.3302, -3.4047]])
        # weight matrix
        self.Q = 10*np.eye(2)
        self.R = 1*np.eye(2)
        # stable constraint init
        self.alpha = 100000
        # stable constant
        self.beta = 0.01
        # phi
        self.phi = 0
        # feedback matrix
        self.Phi = self.A + self.B@self.K
        # control parameter matrix
        self.Psi = self.R + self.B.T@self.P@self.B
        # tongxun
        self.k_i = 0.1
        self.Z = np.array([0, 1])
        self.tao = 0

        self.W = 0.1*np.eye(1)

    #Linear augmented matrix
    def sys_init(self):
        # matrix initialization
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

        # 加权矩阵的计算过程，以及推导方程矩阵的叠加过程
        for i in range(1,self.Np+1):

            self.At = np.append(self.At,np.linalg.matrix_power(self.A, i)).reshape(np.size(self.A,0)*i,np.size(self.A,1))

            self.Phit = np.append(self.Phit,np.linalg.matrix_power(self.Phi, i)).reshape(np.size(self.A,0)*i,np.size(self.A,1))

            self.Dis = np.append(self.Dis,np.linalg.matrix_power(self.Phi, i-1)).reshape(np.size(self.A,0)*i,np.size(self.A,1))

            self.Bt = np.hstack((self.Bt, np.zeros((np.size(self.Bt,0),np.size(self.B,1)))))
            if i==1:
                self.Bt = np.vstack((self.Bt, np.linalg.matrix_power(self.A, i-1)@self.B))
                temp = np.linalg.matrix_power(self.A, i-1)@self.B
            else:
                self.Bt = np.vstack((self.Bt, np.hstack((np.linalg.matrix_power(self.A, i-1)@self.B,temp))))
                temp = np.hstack((np.linalg.matrix_power(self.A, i-1)@self.B,temp))

            self.Kt = np.hstack((self.Kt, np.zeros((np.size(self.Kt,0),np.size(self.K,1)))))
            self.Kt = np.vstack((self.Kt,np.hstack((np.zeros((np.size(self.K,0),np.size(self.Kt,1)-np.size(self.K,1))),self.K))))

            self.Psit = np.hstack((self.Psit, np.zeros((np.size(self.Psit,0),np.size(self.Psi,1)))))
            self.Psit = np.vstack((self.Psit,np.hstack((np.zeros((np.size(self.Psi,0),np.size(self.Psit,0))),self.Psi))))

            self.Qt = np.hstack((self.Qt, np.zeros((np.size(self.Qt,0),np.size(self.Q,1)))))
            self.Qt = np.vstack((self.Qt,np.hstack((np.zeros((np.size(self.Q,0),np.size(self.Qt,0))),self.Q))))

            self.Rt = np.hstack((self.Rt, np.zeros(( np.size(self.Rt,0),np.size(self.R,1)))))
            self.Rt = np.vstack((self.Rt,np.hstack((np.zeros((np.size(self.R,0),np.size(self.Rt,0))),self.R))))

            self.Pt = np.hstack((self.Pt, np.zeros(( np.size(self.Pt,0),np.size(self.P,1)))))
            self.Pt = np.vstack((self.Pt,np.hstack((np.zeros((np.size(self.P,0),np.size(self.Pt,0))),self.P))))

        #return self.At, self.Phit, self.Dis, self.Bt, self.Kt, self.Psit, self.Qt, self.Rt, self.Pt
    
    # calculate alpha
    def Cal_alpha(self,c_opt):
        sc = c_opt.T@self.Psit@c_opt
        self.alpha = sc - self.beta*c_opt[0:2].T@self.Psi@c_opt[0:2]
        return self.alpha
    
    def constraint_init(self,desire_x,desire_y,desire_phi):
        # state model
        self.state_space = lambda x_, c_: self.Phi@x_ + self.B@c_[0:2]

        desire_input = np.array([self.v, self.w])
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
        self.x_ub = 20*np.ones(self.Np*np.size(self.Phi,1)) - error - x_desire_state
        self.x_lb = -10*np.ones(self.Np*np.size(self.Phi,1)) + error - x_desire_state
        self.u_ub = 0.35*np.ones(self.Np*np.size(self.B,1)) - desire_input_seq
        self.u_lb = -0.35*np.ones(self.Np*np.size(self.B,1)) - desire_input_seq
        # return self.state_space, self.x_ub, self.x_lb, self.u_ub, self.u_lb
    
    def update_ref(self,tao):
        x = 0.05*tao
        y = 2*np.sin(0.05*tao)
        difx1_tao = 0.05
        difx2_tao = 0
        dify1_tao = 2*0.05*np.cos(0.05*tao)
        dify2_tao = -2*0.05*0.05*np.sin(0.05*tao)
        self.v = np.sqrt(difx1_tao**2 + dify1_tao**2)
        self.w = (difx1_tao*dify2_tao - difx2_tao*dify1_tao)/self.v**2

        ur = np.array([self.v,self.w])

        self.A_i = self.A*np.diag([-self.w,self.w,self.v]) + np.eye(3)

        self.phi = np.arctan(dify1_tao/difx1_tao)

        self.Phi = self.A_i + self.B@self.K

        self.At_i = np.zeros((0,0))
        self.Phit = np.zeros((0,0))
        self.Dis = np.zeros((0,0))

        for i in range(1,self.Np+1):
            self.At_i = np.append(self.At_i,np.linalg.matrix_power(self.A_i, i)).reshape(np.size(self.A,0)*i,np.size(self.A,1))

            self.Phit = np.append(self.Phit,np.linalg.matrix_power(self.Phi, i)).reshape(np.size(self.A,0)*i,np.size(self.A,1))

            self.Dis = np.append(self.Dis,np.linalg.matrix_power(self.Phi, i-1)).reshape(np.size(self.A,0)*i,np.size(self.A,1))

        #return self.A_i, self.At_i, self.v, self.w, ur, self.phi, self.Phi, self.Phit, self.Dis

    def Cal_rotation(self):
        rotation = np.array([[np.cos(self.phi), -np.sin(self.phi), 0],
                            [np.sin(self.phi),  np.cos(self.phi), 0],
                            [0,0,1]])
        return rotation
    
    
    
    def set_opt(self):
        c_opt = self.opti.variable(10)

        self.x = self.opti.parameter(3)
        self.Phitx = self.opti.parameter(15,3)
        self.Disx = self.opti.parameter(15,3)
        self.x_lbx = self.opti.parameter(15,1)
        self.x_ubx = self.opti.parameter(15,1)
        self.u_lbx = self.opti.parameter(10,1)
        self.u_ubx = self.opti.parameter(10,1)
        self.alp = self.opti.parameter(1)
        self.opti.set_value(self.x,self.error_state)
        self.opti.set_value(self.Phitx,self.Phit)
        self.opti.set_value(self.Disx,self.Dis)
        self.opti.set_value(self.x_lbx,self.x_lb)
        self.opti.set_value(self.x_ubx,self.x_ub)
        self.opti.set_value(self.u_lbx,self.u_lb)
        self.opti.set_value(self.u_ubx,self.u_ub)
        self.opti.set_value(self.alp,self.alpha)
        self.opti.minimize( c_opt.T@self.Psit@c_opt)
        # self.opti.minimize( c_opt.T@self.Psit@c_opt)
        self.opti.subject_to( self.Phitx@self.x + self.Bt@c_opt >= self.x_lbx )
        self.opti.subject_to( self.Phitx@self.x + self.Bt@c_opt <= self.x_ubx )
        self.opti.subject_to( self.K@self.x + c_opt[0:2] <= self.u_ub[0:2] )
        self.opti.subject_to( self.K@self.x + c_opt[0:2] >= self.u_lb[0:2] )
        # opti.subject_to( Kt@(Disx@x + Bt@c_opt) + c_opt <= u_ub )
        # opti.subject_to( Kt@(Disx@x + Bt@c_opt) + c_opt >= u_lb )
        # self.opti.subject_to( c_opt.T@self.Psit@c_opt <= self.alp )
        opts_setting = {'ipopt.max_iter':1000, 'ipopt.print_level':0, 'print_time':0, 'ipopt.acceptable_tol':1e-6, 'ipopt.acceptable_obj_change_tol':1e-6}
        self.opti.solver('ipopt',opts_setting)
        return c_opt
    

    def update_opt(self):
        self.opti.set_value(self.x,self.error_state)
        self.opti.set_value(self.Phitx,self.Phit)
        self.opti.set_value(self.Disx,self.Dis)
        self.opti.set_value(self.x_lbx,self.x_lb)
        self.opti.set_value(self.x_ubx,self.x_ub)
        self.opti.set_value(self.u_lbx,self.u_lb)
        self.opti.set_value(self.u_ubx,self.u_ub)
        self.opti.set_value(self.alp,self.alpha)


    def Cal_error_input(self,c_opt):
        self.error_input = self.K@self.error_state + c_opt[0:2]
        return self.error_input
    
    def Cal_error_state(self,c_opt):
        self.error_state = self.state_space(self.error_state, c_opt)
        return self.error_state
    
    def Cal_true_state(self,desire_state):
        true_state = self.error_state + desire_state
        return true_state
    
    def Cal_true_input(self,desire_input):
        true_input = self.error_input + desire_input
        return true_input
    
    def Cal_tao(self,desire_input):
        self.tao = self.tao + self.k_i*self.Z@(self.error_input + desire_input)
        return self.tao
