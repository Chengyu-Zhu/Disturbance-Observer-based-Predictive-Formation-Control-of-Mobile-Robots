
import numpy as np
from sqp_qcqp import solve_qcqp


class DualMPC:

    def __init__(self, T=0.1, Np=5):
        self.T = T
        self.Np = Np
        self.error_state = np.zeros(3)
        # system params (will be set in sys_para)
        self.sys_para()

    def sys_para(self):
        self.A = np.array([[0, self.T, 0], [self.T, 0, self.T], [0, 0, 0]])
        self.B = np.array([[self.T, 0], [0, 0], [0, self.T]])
        self.P = np.array([[63.1806, -9.4404, -3.29], [-5.8, 2072.7, 64.644], [0.8785, 56.5868, 58.6687]])
        self.K = np.array([[-3.6621, 0.3378, -0.0517], [-0.3904, -3.3302, -3.4047]])
        self.Q = 10 * np.eye(2)
        self.R = 1 * np.eye(2)
        self.Phi = self.A + self.B @ self.K
        self.Psi = self.R + self.B.T @ self.P @ self.B
        # W_{i,l} = 0.5 * I (scalar stored at [0,0])
        self.W = 0.5 * np.eye(1)
        # alpha_i = 0.01
        self.alpha = 0.01
        self.beta = 0.01

        # --- Disturbance observer
        nx = self.A.shape[0]
        self.nd = nx
        self.D = np.eye(nx)
        self.L = np.zeros((self.nd, nx))
        if self.nd >= 3:
            self.L[0, 0] = 0.89
            self.L[1, 1] = 0.901
            self.L[2, 2] = 0.89
        else:
            for i in range(self.nd):
                self.L[i, i] = 0.89

        nu = self.B.shape[1]
        self.Kw = np.zeros((nu, self.nd))
        try:
            self.Kw[0, 0] = -10.0
            self.Kw[1, 2] = -10.0
        except Exception:
            pass
        self.wbar = np.zeros((self.nd,))
        self.theta = np.zeros((self.nd,))

    def sys_init(self):
        nx = self.A.shape[0]
        nu = self.B.shape[1]
        self.At = np.zeros((0, nx))
        self.Phit = np.zeros((0, nx))
        self.Dis = np.zeros((0, nx))
        self.Bt = np.zeros((0, nu * self.Np))
        self.Kt = np.zeros((0, nu))
        self.Psit = np.zeros((0, nu * self.Np))
        self.Qt = np.zeros((0, 2))
        self.Rt = np.zeros((0, 2))
        self.Pt = np.zeros((0, nx))
        self.Wt = np.zeros((0, 1))

        # Build Bt as block lower-triangular Toeplitz: size (Np*nx) x (Np*nu)
        nx = self.A.shape[0]
        nu = self.B.shape[1]
        self.Bt = np.zeros((self.Np * nx, self.Np * nu))
        for i in range(self.Np):
            for j in range(i + 1):
                # block at row i, col j is A^{i-j} * B
                block = np.linalg.matrix_power(self.A, i - j) @ self.B
                r0 = i * nx
                c0 = j * nu
                self.Bt[r0:r0 + nx, c0:c0 + nu] = block
            # append other stacked matrices
            self.At = np.vstack([self.At, np.linalg.matrix_power(self.A, i + 1)])
            self.Phit = np.vstack([self.Phit, np.linalg.matrix_power(self.Phi, i + 1)])
            self.Dis = np.vstack([self.Dis, np.linalg.matrix_power(self.Phi, i)])
            self.Wt = np.vstack([self.Wt, self.W])

        # Build Psit (block diagonal of Psi repeated)
        self.Psit = np.kron(np.eye(self.Np), self.Psi)

    def update_ref(self, tao):
        # simple reference path (same as Multi_sin style)
        difx1 = 0.01
        dify1 = 2 * 0.01 * np.cos(0.01 * tao)
        self.v = np.sqrt(difx1**2 + dify1**2)
        # avoid divide by zero
        self.w = 0.0 if abs(self.v) < 1e-8 else 0.0
        # update Phi with a simple approx
        self.Phi = self.A + self.B @ self.K

    def constraint_init(self, desire_x, desire_y, desire_phi):
        # build simple box constraints around reference
        nx = self.A.shape[0]
        nu = self.B.shape[1]
        x_des = np.tile([desire_x, desire_y, desire_phi], self.Np)
        self.x_ub = 6 * np.ones_like(x_des) - x_des
        self.x_lb = -6 * np.ones_like(x_des) - x_des
        self.u_ub = 0.4 * np.ones(2 * self.Np)
        self.u_lb = -0.4 * np.ones(2 * self.Np)
        

    def solve_control(self, tao1, tao2):
        n = 2 * self.Np

        # ensure k_i and Z exist
        if not hasattr(self, 'k_i'):
            self.k_i = 0.1
        if not hasattr(self, 'Z'):
            # default per-step Z (length nu=2)
            self.Z = np.array([0.0, 1.0])

        # build Wt as block-diagonal kron(I_Np, W_step) where W_step is nu x nu
        nu = self.B.shape[1]
        try:
            w_scalar = float(self.W[0, 0])
        except Exception:
            w_scalar = 0.0
        W_step = w_scalar * np.eye(nu)
        Wt = np.kron(np.eye(self.Np), W_step)

        # build Z_vec by tiling per-step Z
        z_step = np.asarray(self.Z).flatten()
        if z_step.size == nu:
            Z_vec = np.tile(z_step, self.Np)
        elif z_step.size == n:
            Z_vec = z_step.copy()
        else:
            # fallback: zeros
            Z_vec = np.zeros(n)

        # quadratic and linear terms for solver 0.5*c' H c + f' c
        H = 2.0 * self.Psit
        # linear term from cross-terms: 2*k_i*(tao1+tao2) * Z^T Wt c  => f = 2*k_i*(tao1+tao2) * (Wt @ Z_vec)
        f = 2.0 * self.k_i * (tao1 + tao2) * (Wt @ Z_vec)

        # constant term (not used in optimization) for logging
        try:
            const_term = (self.k_i ** 2) * float(Z_vec.T @ (Wt @ Z_vec))
        except Exception:
            const_term = None

        # ensure shapes and symmetry
        H = 0.5 * (H + H.T)
        H = H.astype(np.float64)
        f = np.asarray(f, dtype=np.float64).flatten()
        # keep Wt and Z_vec for debugging
        self.Wt_full = Wt
        self.Z_vec = Z_vec
        self.last_qp_const = const_term

        # Only simple linear constraints on first two entries (u bounds)
        # K@x + c[0:2] <= u_ub_first
        # K@x + c[0:2] >= u_lb_first  -> translate to linear inequalities for c
        # Here we approximate using simple bounds on c[0:2]
        a_list = []
        b_list = []
        Q_list = []

        # convert simple bounds to linear inequality rows of form A c <= b
        A = np.zeros((4, n))
        A[0, 0] = 1
        A[1, 1] = 1
        A[2, 0] = -1
        A[3, 1] = -1
        ub = np.array([self.u_ub[0], self.u_ub[1], -self.u_lb[0], -self.u_lb[1]])

        # Convert each linear row A_i x <= ub_i into QCQP constraint of form
        # 0.5 x^T Q x + a^T x + b <= 0 where Q=0, a=A_i, b=-ub_i
        for i_row in range(A.shape[0]):
            a_row = A[i_row, :].copy()
            Q_list.append(np.zeros((n, n)))
            a_list.append(a_row)
            b_list.append(-ub[i_row])

        # --- Add state constraints: x = Phit * error_state + Bt * c
        # Ensure Phit and Bt exist (sys_init should have been called)
        try:
            Phit = self.Phit
            Bt = self.Bt
        except Exception:
            Phit = None
            Bt = None

        if Phit is not None and Bt is not None and hasattr(self, 'x_ub') and hasattr(self, 'x_lb'):
            # current predicted state offset
            phit_x = Phit @ self.error_state
            # rhs for inequalities: Bt * c <= x_ub - Phit*x
            rhs_ub = self.x_ub - phit_x
            rhs_lb = self.x_lb - phit_x
            # for each row i: Bt[i,:] * c <= rhs_ub[i]
            for i_row in range(Bt.shape[0]):
                bt_row = Bt[i_row, :].copy()
                # upper bound: bt_row @ c <= rhs_ub[i]
                Q_list.append(np.zeros((n, n)))
                a_list.append(bt_row)
                b_list.append(-rhs_ub[i_row])
                # lower bound: -bt_row @ c <= -rhs_lb[i] -> -bt_row @ c + rhs_lb <= 0
                Q_list.append(np.zeros((n, n)))
                a_list.append(-bt_row)
                b_list.append(rhs_lb[i_row])

        # --- Add quadratic constraint c'Psit*c <= alpha (alpha updated each loop)
        try:
            if hasattr(self, 'last_qp_solution') and self.last_qp_solution is not None:
                alpha_val = self.Cal_alpha(self.last_qp_solution)
            else:
                # fallback to stored alpha (initialized in sys_para)
                alpha_val = getattr(self, 'alpha', 1e5)
        except Exception:
            alpha_val = getattr(self, 'alpha', 1e5)

        # convert c'Psit*c <= alpha  -> 0.5*c'*(2*Psit)*c + 0'*c + (-alpha) <= 0
        try:
            Q_list.append(2.0 * self.Psit)
            a_list.append(np.zeros(n))
            b_list.append(-float(alpha_val))
        except Exception:
            # if Psit not set or shapes wrong, skip adding
            pass

        # solve using solve_qcqp which expects quadratic constraints optionally
        try:
            x_opt, fval, history = solve_qcqp(H, f, Q_list, a_list, b_list, x0=np.zeros(n), max_iter=50, tol=1e-6, plot_history=False)
        except Exception:
            # fallback to zeros and record
            x_opt = np.zeros(n)
            fval = float('inf')
            history = {}

        # normalize solver output to 1D numpy array
        try:
            x_opt = np.asarray(x_opt).flatten()
        except Exception:
            x_opt = np.zeros(n)

        # pad or truncate
        if x_opt.size < n:
            x_opt = np.pad(x_opt, (0, n - x_opt.size))
        elif x_opt.size > n:
            x_opt = x_opt[:n]

        # replace non-finite values
        if not np.isfinite(x_opt).all():
            x_opt = np.nan_to_num(x_opt, nan=0.0, posinf=0.0, neginf=0.0)

        # store debug info
        self.last_qp_solution = x_opt.copy()
        try:
            self.last_qp_fval = float(fval) if isinstance(fval, (int, float, np.floating)) else None
        except Exception:
            self.last_qp_fval = None
        self.last_qp_history = history

        # --- Disturbance observer update (discrete) ---
        try:
            xk = self.error_state.copy()
            uk = (self.K @ self.error_state + x_opt[0:2]).flatten()
            # predicted next state (error) given c_opt
            xbar_next = self.Cal_error_state(x_opt)

            # compute theta(k+1) = - (L D - I) wbar(k) - L (A x_k + B u_k)
            # Note: shapes: L (nd x nx), D (nx x nd), wbar (nd,), A (nx x nx), B (nx x nu), uk (nu,)
            LD_minus_I = self.L @ self.D - np.eye(self.nd)
            theta_next = - (LD_minus_I @ self.wbar.flatten()) - self.L @ (self.A @ xk + self.B @ uk)

            # wbar(k+1) = theta(k+1) + L xbar(k+1)
            wbar_next = theta_next + (self.L @ xbar_next).flatten()

            # assign back (ensure 1D arrays)
            try:
                self.theta = np.asarray(theta_next).flatten()
                self.wbar = np.asarray(wbar_next).flatten()
            except Exception:
                pass
        except Exception:
            # if observer update fails, keep previous estimates
            pass

        return x_opt

    # helper methods
    def Cal_error_input(self, c_opt):
        """
        Compute feedback input with disturbance compensation:
        u_err = K * error_state + c[0:2] + K_w * wbar
        """
        base = self.K @ self.error_state + c_opt[0:2]
        # add disturbance compensation if available
        try:
            dw = (self.Kw @ self.wbar).flatten()
            # ensure shape length 2
            if dw.size == base.size:
                return base + dw
            else:
                # fallback if shapes mismatch
                return base
        except Exception:
            return base

    def Cal_error_state(self, c_opt):
        # discrete dynamics: x_{k+1} = Phi x_k + B c[0:2]
        return self.Phi @ self.error_state + self.B @ c_opt[0:2]

    def Cal_true_state(self, desire_state):
        return self.error_state + desire_state

    def Cal_tao(self, desire_input):
        # small integrator example
        if not hasattr(self, 'tao'):
            self.tao = 0.0
        if not hasattr(self, 'k_i'):
            self.k_i = 0.1
            self.Z = np.array([0, 1])
        self.tao = self.tao + self.k_i * self.Z @ (self.error_state[0:2] + desire_input)
        return self.tao

    def Cal_alpha(self, c_opt):
        try:
            c = np.asarray(c_opt).reshape(-1, 1)
            if hasattr(self, 'Psit'):
                sc = float((c.T @ self.Psit @ c).squeeze())
            else:
                sc = float((c.T @ c).squeeze())
            psi_term = 0.0
            if hasattr(self, 'Psi'):
                psi_term = float((c[0:2].T @ self.Psi @ c[0:2]).squeeze())
            beta = float(getattr(self, 'beta', 0.01))
            alpha_val = float(sc - beta * psi_term)
            # sanity: ensure finite and non-negative (avoid infeasible negative alpha)
            if not np.isfinite(alpha_val):
                return float(getattr(self, 'alpha', 1e5))
            # enforce a small positive lower bound to keep constraint feasible
            alpha_val = max(alpha_val, 1e-6)
            self.alpha = float(alpha_val)
            try:
                # record history for diagnostics
                if not hasattr(self, 'alpha_history'):
                    self.alpha_history = []
                self.alpha_history.append(self.alpha)
            except Exception:
                pass
            return self.alpha
        except Exception:
            return float(getattr(self, 'alpha', 1e5))
