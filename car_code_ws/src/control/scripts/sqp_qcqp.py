import numpy as np
import matplotlib.pyplot as plt
from scipy.optimize import minimize, LinearConstraint
from scipy.integrate import solve_ivp

# import the ODE-based RNN solver implementation from RNN_New.py in same folder
import RNN_New

def solve_qcqp(H, f, Q_list, a_list, b_list, x0=None, max_iter=100, tol=1e-6, plot_history=False):
    n = H.shape[0]
    m = len(Q_list)
    if x0 is None:
        x = np.zeros(n)
    else:
        x = np.array(x0).copy()
    lambd = np.zeros(m)
    history = {'x': [], 'fval': [], 'constr': [], 'lambda': []}

    if m == 0:
        def qp_obj(x_):
            return 0.5 * x_ @ H @ x_ + f @ x_
        def qp_jac(x_):
            return H @ x_ + f
        res = minimize(qp_obj, x, jac=qp_jac, method='trust-constr')
        x_opt = res.x
        fval_opt = qp_obj(x_opt)
        history['x'].append(x_opt.copy())
        history['fval'].append(fval_opt)
        history['constr'].append([])
        history['lambda'].append([])
        if plot_history:
            plot_optimization_history(history)
        return x_opt, fval_opt, history

    B = np.eye(n)
    grad_f_prev = H @ x + f
    x_prev = x.copy()
    for k in range(max_iter):
        history['x'].append(x.copy())
        fval = 0.5 * x @ H @ x + f @ x
        history['fval'].append(fval)
        constr_vals = [0.5 * x @ Q_list[i] @ x + a_list[i] @ x + b_list[i] for i in range(m)]
        history['constr'].append(constr_vals)
        history['lambda'].append(lambd.copy())
        grad_f = H @ x + f
        grad_c = np.column_stack([Q_list[i] @ x + a_list[i] for i in range(m)])
        # BFGS更新
        if k > 0:
            s = x - x_prev
            y = grad_f - grad_f_prev
            if s @ y >= 0.2 * s @ B @ s:
                theta = 1
            else:
                theta = (0.8 * s @ B @ s) / (s @ B @ s - s @ y)
            y = theta * y + (1 - theta) * B @ s
            if s @ y > tol:
                Bs = B @ s
                B = B - np.outer(Bs, Bs) / (s @ Bs) + np.outer(y, y) / (s @ y)
        grad_f_prev = grad_f.copy()
        x_prev = x.copy()
        # 构造QP子问题
        H_qp = B
        f_qp = grad_f
        A_qp = grad_c.T
        b_qp = np.array([- (0.5 * x @ Q_list[i] @ x + a_list[i] @ x + b_list[i]) for i in range(m)])
        def qp_obj(d):
            return 0.5 * d @ H_qp @ d + f_qp @ d
        def qp_jac(d):
            return H_qp @ d + f_qp
        # solve QP subproblem using RNN ODE solver (from RNN_New)
        def rnn_solve_qp(H_qp, f_qp, A_ineq, b_ineq, gamma=0.1, Upsilon=1.0, eta=0.5,
                         t_span=(0.0, 5.0), y0=None):
            """
            Solve QP: min 0.5*x'Hx + f'x s.t. A_ineq x <= b_ineq
            using RNN ODE solver implemented in RNN_New. Returns x_opt.
            """
            n = H_qp.shape[0]
            m_ineq = b_ineq.size if hasattr(b_ineq, 'size') else len(b_ineq)

            W = H_qp
            q = f_qp
            # F: equality constraints (none for our QP subproblem) -> zero-rows
            F = np.zeros((0, n))
            M = A_ineq.reshape((m_ineq, n)) if m_ineq > 0 else np.zeros((0, n))
            h = np.array(b_ineq).reshape((m_ineq,)) if m_ineq > 0 else np.array([])
            g_val = np.array([])

            # build P and v like in RNN_New
            n_x = W.shape[0]
            n_lambda = h.size
            n_mu = F.shape[0]

            P = np.block([
                [W, F.T, M.T],
                [F, np.zeros((n_mu, n_mu)), np.zeros((n_mu, n_lambda))],
                [-eta * M, np.zeros((n_lambda, n_mu)), eta * np.eye(n_lambda)]
            ])

            if n_mu == 0:
                v = np.concatenate([q, np.array([]), eta * h + 1.0])
            else:
                v = np.concatenate([q, -g_val, eta * h + 1.0])

            # initial state
            if y0 is None:
                x0 = np.zeros(n_x)
                lambda0 = np.zeros(n_lambda)
                mu0 = np.zeros(n_mu)
                y0 = np.concatenate([x0, lambda0, mu0])

            # call RNN_New.ode_system via solve_ivp
            sol = solve_ivp(lambda t, y: RNN_New.ode_system(t, y, P, v, gamma, Upsilon, W, F, M, h, eta),
                            t_span, y0, method='RK45')
            if not sol.success:
                raise RuntimeError('RNN ODE solver failed to integrate')
            y_end = sol.y[:, -1]
            x_opt = y_end[:n_x]
            return x_opt

        try:
            d = rnn_solve_qp(H_qp, f_qp, A_qp, b_qp, gamma=0.1, Upsilon=1.0, eta=0.5, t_span=(0, 5))
        except Exception as e:
            if not res.success:
                print('QP子问题求解失败')
                break
            d = res.x
        # 线搜索
        alpha = 1.0
        max_ls = 10
        merit_decreased = False
        constr_viol_old = sum(max(0, v) for v in constr_vals)
        merit_old = fval + 10 * constr_viol_old
        for ls in range(max_ls):
            x_new = x + alpha * d
            constr_viol_new = 0
            fval_new = 0.5 * x_new @ H @ x_new + f @ x_new
            for i in range(m):
                c_val = 0.5 * x_new @ Q_list[i] @ x_new + a_list[i] @ x_new + b_list[i]
                if c_val > 0:
                    constr_viol_new += c_val
            merit_new = fval_new + 10 * constr_viol_new
            if merit_new <= merit_old:
                x = x_new
                merit_decreased = True
                break
            else:
                alpha *= 0.5
        if not merit_decreased:
            x = x + d
        if np.linalg.norm(d) < tol:
            break
    x_opt = x
    fval_opt = 0.5 * x_opt @ H @ x_opt + f @ x_opt
    if plot_history:
        plot_optimization_history(history)
    return x_opt, fval_opt, history

# 兼容原有接口
def sqp_qcqp():
    # 问题参数
    n = 2  # 变量维度
    m = 2  # 约束数量

    H = np.array([[4, -1],
                  [-1, 4]])
    f = np.array([1, 2])
    Q = [np.eye(2), np.eye(2)]
    a = [np.array([-1, -1]), np.array([1, 1])]
    b = [-1, -2]
    x_opt, fval_opt, history = solve_qcqp(H, f, Q, a, b, x0=None, max_iter=100, tol=1e-6, plot_history=True)
    print('\n===== SQP求解QCQP结果 =====')
    print(f'最优解: {x_opt}')
    print(f'目标函数值: {fval_opt}')
    print('约束违反检查:')
    for i in range(len(Q)):
        constr_val = 0.5 * x_opt @ Q[i] @ x_opt + a[i] @ x_opt + b[i]
        print(f'  约束{i+1}: {constr_val} (应≤0)')
    return x_opt, fval_opt, history

def plot_optimization_history(history):
    k = len(history['x'])
    x_hist = np.array(history['x'])
    fval_hist = np.array(history['fval'])
    constr_hist = np.array(history['constr'])
    lambda_hist = np.array(history['lambda'])
    plt.rcParams['font.sans-serif'] = ['SimHei']  # 设置中文字体为黑体
    plt.rcParams['axes.unicode_minus'] = False 
    plt.figure(figsize=(12, 8))
    plt.subplot(2,2,1)
    plt.plot(range(1, k+1), fval_hist, 'b-o', linewidth=1.5)
    plt.title('目标函数值变化')
    plt.xlabel('迭代次数')
    plt.ylabel('目标函数值')
    plt.grid(True)
    plt.subplot(2,2,2)
    plt.plot(range(1, k+1), x_hist[:,0], 'r-o', label='x1')
    plt.plot(range(1, k+1), x_hist[:,1], 'b-o', label='x2')
    plt.title('变量变化')
    plt.xlabel('迭代次数')
    plt.ylabel('变量值')
    plt.legend()
    plt.grid(True)
    plt.subplot(2,2,3)
    for i in range(constr_hist.shape[1]):
        plt.plot(range(1, k+1), constr_hist[:,i], 'o-', linewidth=1.5, label=f'约束{i+1}')
    plt.title('约束值变化')
    plt.xlabel('迭代次数')
    plt.ylabel('约束值')
    plt.legend()
    plt.grid(True)
    plt.subplot(2,2,4)
    for i in range(lambda_hist[0].shape[0]):
        plt.plot(range(1, k+1), lambda_hist[:,i], 'o-', linewidth=1.5, label=f'λ{i+1}')
    plt.title('拉格朗日乘子变化')
    plt.xlabel('迭代次数')
    plt.ylabel('乘子值')
    plt.legend()
    plt.grid(True)
    plt.suptitle('SQP求解QCQP优化过程', fontsize=14)
    plt.tight_layout(rect=[0, 0, 1, 0.96])
    plt.show()

if __name__ == '__main__':
    sqp_qcqp()
