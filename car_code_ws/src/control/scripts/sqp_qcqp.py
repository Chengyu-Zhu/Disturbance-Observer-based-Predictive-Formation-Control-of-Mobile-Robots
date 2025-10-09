import numpy as np
import matplotlib.pyplot as plt
from scipy.optimize import minimize, LinearConstraint

def solve_qcqp(H, f, Q_list, a_list, b_list, x0=None, max_iter=100, tol=1e-6, plot_history=False):
    """
    通用QCQP求解器（支持多个二次约束），基于SQP+BFGS+scipy.optimize.minimize
    min 0.5*x'Hx + f'x
    s.t. 0.5*x'Q_i x + a_i'x + b_i <= 0, i=1,...,m
    参数：
        H, f: 目标函数参数
        Q_list, a_list, b_list: 约束参数（list，每个元素为ndarray）
        x0: 初始点
        max_iter, tol: SQP参数
        plot_history: 是否绘制优化过程
    返回：x_opt, fval_opt, history
    """
    n = H.shape[0]
    m = len(Q_list)
    if x0 is None:
        x = np.zeros(n)
    else:
        x = np.array(x0).copy()
    lambd = np.zeros(m)
    history = {'x': [], 'fval': [], 'constr': [], 'lambda': []}
    # 如果没有二次约束，直接用QP求解器
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
    # 否则走SQP主循环
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
        lin_con = LinearConstraint(A_qp, -np.inf * np.ones(m), b_qp)
        res = minimize(qp_obj, np.zeros(n), jac=qp_jac, constraints=[lin_con], method='trust-constr')
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
