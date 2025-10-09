import numpy as np
from scipy.optimize import minimize
import matplotlib.pyplot as plt
import time

plt.rcParams['font.sans-serif'] = ['SimHei']  # 设置中文字体为黑体
plt.rcParams['axes.unicode_minus'] = False    # 正常显示负号

def sqp_qcqp_quadprog():
    # SQP方法求解QCQP问题
    # 问题形式:
    #   min  0.5*x'*H*x + f'*x
    #   s.t. 0.5*x'*Q_i*x + a_i'*x + b_i <= 0, i=1,2,...,m
    
    # 问题参数定义
    n = 2  # 变量维度
    m = 2  # 约束数量
    
    # 目标函数
    H = np.array([[4, -1], 
                  [-1, 4]])   # Hessian矩阵 (对称正定)
    f = np.array([1, 2])      # 线性项
    
    # 二次约束定义 (存储为列表)
    Q = []
    a = []
    b = []
    
    # 约束1
    Q.append(np.array([[1, 0], [0, 1]]))  # Hessian
    a.append(np.array([-1, -1]))          # 线性项
    b.append(-1)                         # 常数项
    
    # 约束2
    Q.append(np.array([[1, 0], [0, 1]]))  # Hessian
    a.append(np.array([1, 1]))            # 线性项
    b.append(-2)                          # 常数项
    
    # SQP算法参数
    max_iter = 100       # 最大迭代次数
    tol = 1e-6           # 收敛容差
    x = np.array([0.0, 0.0])  # 初始点
    lambda_vals = np.zeros(m) # 初始拉格朗日乘子
    
    # 存储优化过程信息
    history = {
        'x': np.zeros((n, max_iter)),
        'fval': np.zeros(max_iter),
        'constr': np.zeros((m, max_iter)),
        'lambda': np.zeros((m, max_iter))
    }
    
    # 初始化BFGS所需的变量
    B = np.eye(n)  # Hessian近似
    grad_f_prev = H @ x + f
    x_prev = x.copy()
    
    # SQP主循环
    k = 0
    for k in range(max_iter):
        # 保存当前迭代信息
        history['x'][:, k] = x
        history['fval'][k] = 0.5 * x.T @ H @ x + f.T @ x
        for i in range(m):
            history['constr'][i, k] = 0.5 * x.T @ Q[i] @ x + a[i].T @ x + b[i]
        history['lambda'][:, k] = lambda_vals
        
        # 计算当前点的目标函数梯度和约束梯度
        grad_f = H @ x + f
        grad_c = np.zeros((n, m))  # 存储所有约束的梯度
        
        for i in range(m):
            grad_c[:, i] = Q[i] @ x + a[i]
        
        # 构建Lagrange Hessian的近似 (BFGS更新)
        if k > 0:
            # BFGS更新
            s = x - x_prev
            y = grad_f - grad_f_prev
            
            # 确保更新保持正定性
            if s.T @ y >= 0.2 * s.T @ B @ s:
                theta = 1.0
            else:
                theta = (0.8 * s.T @ B @ s) / (s.T @ B @ s - s.T @ y)
            y = theta * y + (1 - theta) * B @ s
            
            # BFGS公式更新
            if s.T @ y > tol:
                B = B - (B @ np.outer(s, s) @ B) / (s.T @ B @ s) + np.outer(y, y) / (s.T @ y)
        
        # 保存当前梯度用于下一次BFGS更新
        grad_f_prev = grad_f.copy()
        x_prev = x.copy()
        
        # 构造QP子问题 (在x处线性化约束)
        # 目标函数: min 0.5*d'*B*d + grad_f'*d
        # 约束: grad_c'*d <= -c(x)  其中 c(x) = 0.5*x'*Q_i*x + a_i'*x + b_i
        
        # 线性化约束的右侧项
        b_qp = np.zeros(m)
        for i in range(m):
            c_val = 0.5 * x.T @ Q[i] @ x + a[i].T @ x + b[i]
            b_qp[i] = -c_val
        
        # 定义QP子问题的目标函数
        def qp_objective(d):
            return 0.5 * d.T @ B @ d + grad_f.T @ d
        
        # 定义QP子问题的约束 (不等式约束)
        constraints = []
        for i in range(m):
            def constr(d, i=i):
                return grad_c[:, i].T @ d - b_qp[i]
            constraints.append({'type': 'ineq', 'fun': constr})
        
        # 求解QP子问题
        res = minimize(qp_objective, 
                       np.zeros(n), 
                       constraints=constraints,
                       method='SLSQP',
                       options={'ftol': 1e-9, 'disp': False})
        
        if not res.success:
            print('QP子问题求解失败:', res.message)
            break
        
        d = res.x
        # 从QP解中提取乘子 (SciPy返回的乘子结构)
        lambda_vals = np.zeros(m)
        for i in range(m):
            # 乘子对应不等式约束
            lambda_vals[i] = max(0, -res.jac[i])  # 近似获取乘子
        
        # 步长选择 (线搜索)
        alpha = 1.0
        max_ls = 10
        merit_decreased = False
        
        # 计算当前点约束违反程度(标量)
        constr_viol_old = 0
        for i in range(m):
            c_val = history['constr'][i, k]
            if c_val > 0:
                constr_viol_old += c_val
        
        fval_old = history['fval'][k]
        merit_old = fval_old + 10 * constr_viol_old
        
        for ls in range(max_ls):
            x_new = x + alpha * d
            
            # 计算新点约束违反程度(标量)
            constr_viol_new = 0
            fval_new = 0.5 * x_new.T @ H @ x_new + f.T @ x_new
            
            for i in range(m):
                c_val = 0.5 * x_new.T @ Q[i] @ x_new + a[i].T @ x_new + b[i]
                if c_val > 0:
                    constr_viol_new += c_val
            
            merit_new = fval_new + 10 * constr_viol_new
            
            # 检查merit函数改进
            if merit_new <= merit_old:
                x = x_new
                merit_decreased = True
                break
            else:
                alpha *= 0.5
        
        if not merit_decreased:
            x = x + d  # 即使没有改进也接受步长
        
        # 收敛检查
        norm_d = np.linalg.norm(d)
        if norm_d < tol:
            break
    
    # 保存最终结果
    x_opt = x
    fval_opt = 0.5 * x_opt.T @ H @ x_opt + f.T @ x_opt
    
    # 显示结果
    print('\n===== SQP求解QCQP结果 =====')
    print(f'迭代次数: {k+1}')
    print(f'最优解: [{x_opt[0]:.6f}, {x_opt[1]:.6f}]')
    print(f'目标函数值: {fval_opt:.6f}')
    print('约束违反检查:')
    for i in range(m):
        constr_val = 0.5 * x_opt.T @ Q[i] @ x_opt + a[i].T @ x_opt + b[i]
        print(f'  约束{i+1}: {constr_val:.6f} (应≤0)')
    
    # 裁剪历史记录
    history['x'] = history['x'][:, :k+1]
    history['fval'] = history['fval'][:k+1]
    history['constr'] = history['constr'][:, :k+1]
    history['lambda'] = history['lambda'][:, :k+1]
    
    # 绘制优化过程
    plot_optimization_history(history, k+1)
    
    return x_opt, fval_opt

def plot_optimization_history(history, k):
    # 创建图形
    plt.figure(figsize=(12, 10))
    
    # 目标函数变化
    plt.subplot(2, 2, 1)
    plt.plot(range(1, k+1), history['fval'][:k], 'b-o', linewidth=1.5)
    plt.title('目标函数值变化')
    plt.xlabel('迭代次数')
    plt.ylabel('目标函数值')
    plt.grid(True)
    
    # 解变化
    plt.subplot(2, 2, 2)
    plt.plot(range(1, k+1), history['x'][0, :k], 'r-o', label='x1')
    plt.plot(range(1, k+1), history['x'][1, :k], 'b-o', label='x2')
    plt.title('变量变化')
    plt.xlabel('迭代次数')
    plt.ylabel('变量值')
    plt.legend()
    plt.grid(True)
    
    # 约束值变化
    plt.subplot(2, 2, 3)
    for i in range(history['constr'].shape[0]):
        plt.plot(range(1, k+1), history['constr'][i, :k], 'o-', linewidth=1.5, label=f'约束{i+1}')
    plt.title('约束值变化')
    plt.xlabel('迭代次数')
    plt.ylabel('约束值')
    plt.legend()
    plt.grid(True)
    
    # 拉格朗日乘子变化
    plt.subplot(2, 2, 4)
    for i in range(history['lambda'].shape[0]):
        plt.plot(range(1, k+1), history['lambda'][i, :k], 'o-', linewidth=1.5, label=f'λ{i+1}')
    plt.title('拉格朗日乘子变化')
    plt.xlabel('迭代次数')
    plt.ylabel('乘子值')
    plt.legend()
    plt.grid(True)
    
    plt.suptitle('SQP求解QCQP优化过程', fontsize=14)
    plt.tight_layout(rect=[0, 0, 1, 0.96])
    plt.show()

# 运行SQP求解器
if __name__ == "__main__":
    start_time = time.time()
    x_opt, fval_opt = sqp_qcqp_quadprog()
    end_time = time.time()
    print(f"\n计算时间: {end_time - start_time:.4f}秒")