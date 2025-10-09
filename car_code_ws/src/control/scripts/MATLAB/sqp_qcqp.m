function [x_opt, fval_opt] = sqp_qcqp_quadprog()
    % SQP方法求解QCQP问题（使用quadprog迭代求解）
    % 问题形式:
    %   min  0.5*x'*H*x + f'*x
    %   s.t. 0.5*x'*Q_i*x + a_i'*x + b_i <= 0, i=1,2,...,m
    
    % 问题参数定义
    n = 2;  % 变量维度
    m = 2;  % 约束数量
    
    % 目标函数
    H = [4, -1; 
         -1, 4];   % Hessian矩阵 (对称正定)
    f = [1; 2];     % 线性项
    
    % 二次约束定义 (存储为单元数组)
    Q = cell(m, 1);
    a = cell(m, 1);
    b = cell(m, 1);
    
    Q{1} = [1, 0; 0, 1];  % 约束1的Hessian
    a{1} = [-1; -1];      % 约束1的线性项
    b{1} = -1;            % 约束1的常数项
    
    Q{2} = [1, 0; 0, 1];  % 约束2的Hessian
    a{2} = [1; 1];        % 约束2的线性项
    b{2} = -2;            % 约束2的常数项
    
    % SQP算法参数
    max_iter = 100;       % 最大迭代次数
    tol = 1e-6;           % 收敛容差
    x = [0; 0];           % 初始点
    lambda = zeros(m, 1); % 初始拉格朗日乘子
    
    % 存储优化过程信息
    history.x = zeros(n, max_iter);
    history.fval = zeros(1, max_iter);
    history.constr = zeros(m, max_iter);
    history.lambda = zeros(m, max_iter);
    
    % 初始化BFGS所需的变量
    B = eye(n);
    grad_f_prev = H*x + f;
    x_prev = x;
    
    % SQP主循环
    for k = 1:max_iter
        % 保存当前迭代信息
        history.x(:, k) = x;
        history.fval(k) = 0.5*x'*H*x + f'*x;
        for i = 1:m
            history.constr(i, k) = 0.5*x'*Q{i}*x + a{i}'*x + b{i};
        end
        history.lambda(:, k) = lambda;
        
        % 计算当前点的目标函数梯度和约束梯度
        grad_f = H*x + f;
        grad_c = zeros(n, m); % 存储所有约束的梯度
        
        for i = 1:m
            grad_c(:, i) = Q{i}*x + a{i};
        end
        
        % 构建Lagrange Hessian的近似 (BFGS更新)
        if k > 1
            % BFGS更新
            s = x - x_prev;
            y = grad_f - grad_f_prev;
            
            % 确保更新保持正定性
            if s'*y >= 0.2*s'*B*s
                theta = 1;
            else
                theta = (0.8*s'*B*s)/(s'*B*s - s'*y);
            end
            y = theta*y + (1-theta)*B*s;
            
            % BFGS公式更新
            if s'*y > tol
                B = B - (B*(s*s')*B)/(s'*B*s) + (y*y')/(s'*y);
            end
        end
        
        % 保存当前梯度用于下一次BFGS更新
        grad_f_prev = grad_f;
        x_prev = x;
        
        % 构造QP子问题 (在x处线性化约束)
        H_qp = B;
        f_qp = grad_f;
        
        % 线性化约束
        A_qp = grad_c';
        b_qp = zeros(m, 1);
        for i = 1:m
            b_qp(i) = - (0.5*x'*Q{i}*x + a{i}'*x + b{i});
        end
        
        % 求解QP子问题
        options = optimoptions('quadprog', 'Display', 'off');
        [d, ~, exitflag_qp, ~, lambda_qp] = quadprog(H_qp, f_qp, A_qp, b_qp, [], [], [], [], [], options);
        
        if exitflag_qp <= 0
            warning('QP子问题求解失败');
            break;
        end
        
        % 从QP解中提取乘子
        lambda = lambda_qp.ineqlin;
        
        % 步长选择 (线搜索)
        alpha = 1.0;
        max_ls = 10;
        merit_decreased = false;
        
        % 计算当前点约束违反程度(标量)
        constr_viol_old = sum(max(0, history.constr(:,k)));
        fval_old = history.fval(k);
        merit_old = fval_old + 10 * constr_viol_old;
        
        for ls = 1:max_ls
            x_new = x + alpha*d;
            
            % 计算新点约束违反程度(标量)
            constr_viol_new = 0;
            fval_new = 0.5*x_new'*H*x_new + f'*x_new;
            
            for i = 1:m
                c_val = 0.5*x_new'*Q{i}*x_new + a{i}'*x_new + b{i};
                if c_val > 0
                    constr_viol_new = constr_viol_new + c_val;
                end
            end
            
            merit_new = fval_new + 10 * constr_viol_new;
            
            % 检查merit函数改进
            if merit_new <= merit_old
                x = x_new;
                merit_decreased = true;
                break;
            else
                alpha = alpha * 0.5;
            end
        end
        
        if ~merit_decreased
            x = x + d; % 即使没有改进也接受步长
        end
        
        % 收敛检查
        norm_d = norm(d);
        if norm_d < tol
            break;
        end
    end
    
    % 保存最终结果
    x_opt = x;
    fval_opt = 0.5*x_opt'*H*x_opt + f'*x_opt;
    
    % 显示结果
    fprintf('\n===== SQP求解QCQP结果 =====\n');
    fprintf('迭代次数: %d\n', k);
    fprintf('最优解: [%f, %f]\n', x_opt(1), x_opt(2));
    fprintf('目标函数值: %f\n', fval_opt);
    fprintf('约束违反检查:\n');
    for i = 1:m
        constr_val = 0.5*x_opt'*Q{i}*x_opt + a{i}'*x_opt + b{i};
        fprintf('  约束%d: %f (应≤0)\n', i, constr_val);
    end
    
    % 裁剪历史记录
    history.x = history.x(:, 1:k);
    history.fval = history.fval(1:k);
    history.constr = history.constr(:, 1:k);
    history.lambda = history.lambda(:, 1:k);
    
    % 绘制优化过程
    plot_optimization_history(history);
end

function plot_optimization_history(history)
    k = size(history.x, 2);
    
    % 创建图形
    figure('Position', [100, 100, 1000, 700]);
    
    % 目标函数变化
    subplot(2,2,1);
    plot(1:k, history.fval, 'b-o', 'LineWidth', 1.5);
    title('目标函数值变化');
    xlabel('迭代次数');
    ylabel('目标函数值');
    grid on;
    
    % 解变化
    subplot(2,2,2);
    plot(1:k, history.x(1,:), 'r-o', 'DisplayName', 'x1');
    hold on;
    plot(1:k, history.x(2,:), 'b-o', 'DisplayName', 'x2');
    hold off;
    title('变量变化');
    xlabel('迭代次数');
    ylabel('变量值');
    legend('Location', 'best');
    grid on;
    
    % 约束值变化
    subplot(2,2,3);
    for i = 1:size(history.constr, 1)
        plot(1:k, history.constr(i,:), 'o-', 'LineWidth', 1.5, 'DisplayName', ['约束', num2str(i)]);
        hold on;
    end
    hold off;
    title('约束值变化');
    xlabel('迭代次数');
    ylabel('约束值');
    legend('Location', 'best');
    grid on;
    
    % 拉格朗日乘子变化
    subplot(2,2,4);
    for i = 1:size(history.lambda, 1)
        plot(1:k, history.lambda(i,:), 'o-', 'LineWidth', 1.5, 'DisplayName', ['λ', num2str(i)]);
        hold on;
    end
    hold off;
    title('拉格朗日乘子变化');
    xlabel('迭代次数');
    ylabel('乘子值');
    legend('Location', 'best');
    grid on;
    
    sgtitle('SQP求解QCQP优化过程', 'FontSize', 14);
end