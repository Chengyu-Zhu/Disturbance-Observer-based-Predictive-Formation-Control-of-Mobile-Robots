# import numpy as np
# from scipy.optimize import minimize
# from scipy.integrate import solve_ivp

# # 全局变量用于收敛检查
# prev_u = None
# convergence_count = 0

# def main():
#     """
#     主函数：演示使用RNN方法求解凸优化问题
#     """
#     # 二次规划问题参数
#     n = 2  # 解向量的维数
#     r = 1   # 不等式约束数量
#     l = 1   # 等式约束数量

#     # 定义二次规划问题
#     Q = np.array([[8, 0], [0, 5]])
#     c = np.array([2, 3])

#     # 上下限定义
#     ub = np.array([3, 1])
#     lb = np.array([-1, -1])

#     # 不等式约束
#     D = np.array([[2, 1]])  # r*n
#     d = np.array([1])       # r*1

#     # 等式约束
#     B = np.array([[3, 2]])  # l*n
#     b = np.array([1])       # l*1

#     # 使用标准QP求解器验证解的正确性
#     print("=== 标准QP求解器验证 ===")
#     opt = solve_qp(Q, c, D, d, B, b, lb, ub)
#     print(f"QP求解器解: {opt}")
#     print(f"目标函数值: {0.5 * opt.T @ Q @ opt + c.T @ opt}")
    
#     print("\n=== RNN求解器求解 ===")
#     # 使用RNN方法求解
#     x_star = RNN_solve_convex(Q, c, D, d, B, b, lb, ub, n, r, l)
#     print(f"RNN求解解: {x_star}")
#     print(f"目标函数值: {0.5 * x_star.T @ Q @ x_star + c.T @ x_star}")
    
#     # 验证约束满足情况
#     print("\n=== 约束验证 ===")
#     if B is not None and b is not None:
#         eq_constraint = B @ x_star - b
#         print(f"等式约束违反: {np.linalg.norm(eq_constraint):.6e}")
    
#     if D is not None and d is not None:
#         ineq_constraint = D @ x_star - d
#         print(f"不等式约束违反: {np.maximum(ineq_constraint, 0).sum():.6e}")
    
#     print(f"下界约束违反: {np.maximum(lb - x_star, 0).sum():.6e}")
#     print(f"上界约束违反: {np.maximum(x_star - ub, 0).sum():.6e}")

# def solve_qp(Q, c, D=None, d=None, B=None, b=None, lb=None, ub=None):
#     """
#     使用SciPy的minimize函数求解二次规划问题
#     """
#     n = len(c)
    
#     # 构建约束条件
#     constraints = []
    
#     # 等式约束
#     if B is not None and b is not None:
#         constraints.append({'type': 'eq', 'fun': lambda x: B @ x - b})
    
#     # 不等式约束
#     if D is not None and d is not None:
#         constraints.append({'type': 'ineq', 'fun': lambda x: d - D @ x})
    
#     # 构建边界条件
#     if lb is None:
#         lb = -np.inf * np.ones(n)
#     if ub is None:
#         ub = np.inf * np.ones(n)
    
#     bounds = [(lb_i, ub_i) for lb_i, ub_i in zip(lb, ub)]
    
#     # 求解二次规划问题
#     res = minimize(
#         lambda x: 0.5 * x.T @ Q @ x + c.T @ x,
#         x0=np.zeros(n),
#         constraints=constraints,
#         bounds=bounds,
#         method='SLSQP'
#     )
    
#     return res.x

# def RNN_solve_convex(Q, c, D, d, B, b, lb, ub, n, r, l):
#     """
#     使用RNN方法求解凸优化问题
#     """
#     global prev_u, convergence_count
#     prev_u = None
#     convergence_count = 0
    
#     # 负无穷替换，维度为r
#     p = -np.inf * np.ones(r)

#     # 转换矩阵
#     A = np.vstack((B, D)) if (B is not None and D is not None) else (
#         B if B is not None else D
#     )
    
#     b1 = np.concatenate((b, p)) if (b is not None) else p
#     b2 = np.concatenate((b, d)) if (b is not None and d is not None) else (
#         b if b is not None else d
#     )
    
#     E = np.vstack((A, np.eye(n)))
#     d_vec = np.concatenate((b1, lb))
#     h_vec = np.concatenate((b2, ub))

#     # 缩放因子
#     lambda_val = 1

#     # 问题转换成微分方程
#     Q_inv = np.linalg.inv(Q)
#     W = E @ Q_inv @ E.T
#     q = -E @ Q_inv @ c
#     R = Q_inv @ E.T
#     a = -Q_inv @ c

#     # 定义初始状态,维度=l+r+n
#     u0 = np.zeros(l + r + n)

#     # 定义时间范围
#     tspan = (0, 100)  # 设置足够大的时间范围

#     # 定义微分方程函数
#     def state_equations(t, u):
#         return lambda_val * (P_X(W @ u + q - u, d_vec, h_vec) - W @ u - q)
    
#     # 定义收敛检查函数 - 修改为返回浮点数而非布尔值
#     def convergence_check(t, u):
#         global prev_u, convergence_count
        
#         if prev_u is None:
#             prev_u = u
#             return 1.0  # 初始返回正值
        
#         diff = np.linalg.norm(u - prev_u)
#         prev_u = u
        
#         # 连续多次收敛才认为真正收敛
#         if diff < 1e-6:
#             convergence_count += 1
#         else:
#             convergence_count = 0
            
#         # 返回与阈值的差值，这样当diff<1e-6时返回负值
#         return diff - 1e-6
    
#     # 设置事件函数，当convergence_check返回值符号变化时触发
#     convergence_check.terminal = True  # 事件触发时停止积分
    
#     # 尝试使用更稳定的ODE求解方法
#     try:
#         sol = solve_ivp(
#             state_equations, 
#             tspan, 
#             u0, 
#             events=convergence_check,
#             rtol=1e-6, 
#             atol=1e-6,
#             method='BDF'  # 使用BDF方法可能更稳定
#         )
        
#         # 如果因事件停止，使用最终状态
#         if sol.t_events[0].size > 0:
#             u_final = sol.y_events[0][-1]
#         else:
#             u_final = sol.y[:, -1]
            
#     except ValueError as e:
#         # 如果事件检测失败，使用固定时间积分
#         print(f"事件检测失败: {e}, 使用固定时间积分")
#         sol = solve_ivp(
#             state_equations, 
#             tspan, 
#             u0,
#             rtol=1e-6, 
#             atol=1e-6,
#             method='BDF'
#         )
#         u_final = sol.y[:, -1]
    
#     # 计算输出
#     x_star = R @ u_final + a
#     return x_star

# def P_X(u, d, h):
#     """
#     投影激活函数，将输入限制在[d, h]范围内
#     """
#     P = np.zeros_like(u)
#     for i in range(len(u)):
#         if u[i] < d[i]:
#             P[i] = d[i]
#         elif u[i] > h[i]:
#             P[i] = h[i]
#         else:
#             P[i] = u[i]
#     return P

# if __name__ == "__main__":
#     main()


import numpy as np
from scipy.integrate import solve_ivp
import matplotlib.pyplot as plt

def main():
    """
    主函数：演示使用RNN方法求解线性系统 Py + v = 0
    """
    print("=== RNN方法求解线性系统 Py + v = 0 ===")
    
    # 设置线性系统参数
    P, v = define_linear_system()
    print(f"矩阵 P = \n{P}")
    print(f"向量 v = {v}")
    
    # 使用直接方法求解作为对比
    y_exact = solve_direct(P, v)
    print(f"\n直接求解解: y = {y_exact}")
    print(f"直接求解误差: ||P y + v|| = {np.linalg.norm(P @ y_exact + v):.6e}")
    
    # 使用RNN方法求解
    y_rnn, sol = solve_with_rnn(P, v)
    print(f"\nRNN求解解: y = {y_rnn}")
    print(f"RNN求解误差: ||P y + v|| = {np.linalg.norm(P @ y_rnn + v):.6e}")
    
    # 绘制收敛过程
    plot_convergence(sol, P, v)
    
    return y_rnn

def define_linear_system():
    """
    定义线性系统 Py + v = 0 的参数
    使用一个简单的2x2系统作为示例
    """
    # 定义一个正定矩阵P以确保系统有唯一解
    P = np.array([[3, 1], 
                  [1, 2]], dtype=float)
    
    # 定义向量v
    v = np.array([2, 1], dtype=float)
    
    return P, v

def solve_direct(P, v):
    """
    使用直接矩阵求逆方法求解 Py + v = 0
    即 y = -P^(-1) v
    """
    try:
        P_inv = np.linalg.inv(P)
        y = -P_inv @ v
        return y
    except np.linalg.LinAlgError:
        print("矩阵P奇异，无法直接求逆")
        return None

def upsilon(e, kappa=1.0):
    """
    激活函数 Υ(e) = exp(κe) - exp(-κe)
    这是一个奇函数且单调递增，保证收敛性
    """
    return np.exp(kappa * e) - np.exp(-kappa * e)

def solve_with_rnn(P, v, gamma=10.0, kappa=1.0, t_span=(0, 5)):
    """
    使用RNN动力学方法求解线性系统 Py + v = 0
    
    参数:
        P: 系统矩阵
        v: 常数向量
        gamma: 收敛率参数
        kappa: 激活函数参数
        t_span: 积分时间范围
    
    返回:
        y: 求解得到的向量
        sol: ODE求解结果
    """
    n = P.shape[0]  # 系统维度
    
    # 定义RNN动力学方程
    def rnn_dynamics(t, y):
        """
        RNN动力学方程: dy/dt = -γ P^T Υ(P y + v)
        """
        e = P @ y + v  # 误差项 e = P y + v
        return -gamma * P.T @ upsilon(e, kappa)
    
    # 初始条件（零初始化）
    y0 = np.zeros(n)
    
    # 定义收敛检查函数
    def convergence_check(t, y):
        e = P @ y + v
        error_norm = np.linalg.norm(e)
        # 当误差小于阈值时触发停止事件
        return error_norm - 1e-6
    
    convergence_check.terminal = True  # 事件触发时停止积分
    
    # 求解微分方程
    sol = solve_ivp(
        rnn_dynamics, 
        t_span, 
        y0,
        events=convergence_check,
        method='RK45',
        rtol=1e-8,
        atol=1e-8
    )
    
    # 获取最终状态
    y_final = sol.y[:, -1]
    
    print(f"RNN求解信息:")
    print(f"  迭代点数: {len(sol.t)}")
    print(f"  最终时间: {sol.t[-1]:.4f}")
    print(f"  收敛状态: {'成功' if sol.status == 0 else '未完全收敛'}")
    
    return y_final, sol

def plot_convergence(sol, P, v):
    """
    绘制RNN求解过程的收敛情况
    """
    # 计算每个时间点的误差
    errors = []
    for i in range(sol.y.shape[1]):
        y_i = sol.y[:, i]
        error = np.linalg.norm(P @ y_i + v)
        errors.append(error)
    
    # 创建图形
    plt.figure(figsize=(12, 5))
    
    # 误差收敛图
    plt.subplot(1, 2, 1)
    plt.semilogy(sol.t, errors, 'b-', linewidth=2)
    plt.xlabel('时间 (t)')
    plt.ylabel('误差 ||P y + v||')
    plt.title('RNN求解误差收敛过程')
    plt.grid(True, alpha=0.3)
    
    # 解分量变化图
    plt.subplot(1, 2, 2)
    for i in range(sol.y.shape[0]):
        plt.plot(sol.t, sol.y[i, :], linewidth=2, label=f'y_{i+1}')
    plt.xlabel('时间 (t)')
    plt.ylabel('解分量值')
    plt.title('解分量随时间变化')
    plt.legend()
    plt.grid(True, alpha=0.3)
    
    plt.tight_layout()
    plt.show()

if __name__ == "__main__":
    # 运行主程序
    y_solution = main()


    