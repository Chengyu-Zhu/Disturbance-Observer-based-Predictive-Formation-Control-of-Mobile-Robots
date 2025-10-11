clear all; close all;clc;

%% 二次规划问题描述
% n是解向量的维数
n = 2;
r=1;
l=1;

% 定义二次规划问题
Q = [5 0; 0 5];
c = [2;5];

% 上下限定义,和状态量维度相同
ub = [3;1];
lb = [-1;-1];

% 不等式约束,r由约束数量确定,本例r=1
D = [2 1];%r*n
d = [1];%r*1

% 等式约束，l由约束数量确定,本例l=1
B = [3 2];%l*n
b = [1];%l*1

% 利用QP求解器检查解的正确性
[opt] = quadprog(Q,c,D,d,B,b);
disp(opt)

% 主程序
[x_star] = RNN_solve_convex(Q,c,D,d,B,b,lb,ub,n,r,l);
disp(x_star)

% %% 转换入神经网络过程
% % 负无穷替换，维度为r
% p = -inf*ones(r,1);
% 
% % 转换矩阵
% A = [B;D];
% b1 = [b;p];
% b2 = [b;d];
% 
% E = [A;eye(n)];
% d = [b1;lb];
% h = [b2;ub];
% 
% % 缩放因子
% lambda = 1;
% 
% % 问题转换成微分方程
% W = E*inv(Q)*E';
% q = -E*inv(Q)*c;
% 
% R = inv(Q)*E';
% a = -inv(Q)*c;
% 
% 
% %% 进行计算
% % 定义初始状态,维度=n+l+r,本例为4
% u0 = zeros(4, 1);
% 
% % 定义时间范围
% tspan = [0 inf]; % 从时间0到时间inf进行求解
% 
% % % 定义输出函数
% % outputFunc = @(t, u) R*u + a;
% 
% % 定义微分方程函数
% stateEquations = @(t, u) lambda*(P_X((W*u + q - u),d,h) - W*u - q);
% 
% % 定义变化幅度停止条件
% options = odeset('RelTol', 1e-6, 'AbsTol',1e-6,'OutputFcn', @convergenceCheck);
% 
% % 求解微分方程
% [t, u] = ode45(stateEquations, tspan, u0, options);
% 
% % % 计算输出
% % x = outputFunc(t, u');
% 
% x_star = R*u(end,:)' + a;
% 
% disp(x_star)

% figure
% plot(t,x(1,:),t,x(2,:));

%% 求解函数
function [x_star] = RNN_solve_convex(Q,c,D,d,B,b,lb,ub,n,r,l)

% 负无穷替换，维度为r
p = -inf*ones(r,1);

% 转换矩阵
A = [B;D];

b1 = [b;p];

b2 = [b;d];

E = [A;eye(n)];

d = [b1;lb];

h = [b2;ub];

% 缩放因子
lambda = 1;

% 问题转换成微分方程
W = E*inv(Q)*E';

q = -E*inv(Q)*c;

R = inv(Q)*E';

a = -inv(Q)*c;
%% 进行计算
% 定义初始状态,维度=n+l+r,本例为4
u0 = zeros(4, 1);

% 定义时间范围
tspan = [0 inf]; % 从时间0到时间inf进行求解

% % 定义输出函数
% outputFunc = @(t, u) R*u + a;

% 定义微分方程函数
stateEquations = @(t, u) lambda*(P_X((W*u + q - u),d,h) - W*u - q);

% 定义变化幅度停止条件
options = odeset('RelTol', 1e-6, 'AbsTol',1e-6,'OutputFcn', @convergenceCheck);

% 求解微分方程
[t, u] = ode45(stateEquations, tspan, u0, options);

% % 计算输出
% x = outputFunc(t, u');

x_star = R*u(end,:)' + a;

disp(x_star)

end
%% 激活函数
function P = P_X(u, d, h)
P = [];
    for i = 1:1:4
        if u(i) < d(i)
            P = [P;d(i)];
        elseif u(i) >= d(i) && u(i) <= h(i)
            P = [P;u(i)];
        else
            P = [P;h(i)];
        end
    end
end


%%  定义收敛判断函数
function status = convergenceCheck(t, y, flag)
    persistent prevY;
    if isempty(prevY)
        prevY = y;
        status = 0;
    else
        diff = norm(norm(y) - norm(prevY));
        if diff < 1e-6
            status = 1; % 停止求解
        else
            prevY = y;
            status = 0;
        end
    end
end

