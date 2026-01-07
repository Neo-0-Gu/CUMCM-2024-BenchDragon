function optimize_model()
    % 优化变量边界
    lb = [-inf, -inf, -inf, -inf, 0, -pi, -pi, -pi, -pi, 0, 0];    % 下界
    ub = [inf, inf, inf, inf, inf, pi, pi, pi, pi, inf, inf];      % 上界
    
    % 遗传算法选项
    options = optimoptions('ga', 'Display', 'iter', 'PopulationSize', 100, 'MaxGenerations', 200);
    [x, fval] = ga(@objective_function, 11, [], [], [], [], lb, ub, @nonlinear_constraints, options);
    
    % 显示结果
    disp('优化结果:');
    disp(x);
    disp(['目标函数值:', num2str(fval)]);
end

% 目标函数
function f = objective_function(x)
    R = x(5);
    alpha0 = x(6);
    alpha1 = x(7);
    beta0 = x(8);
    beta1 = x(9); % PDF中识别为 betal
    
    % 计算目标函数 (似乎是让角度差尽可能小，同时考虑半径 R)
    f = 2 * R * abs(alpha1 - alpha0) + R * abs(beta1 - beta0);
end

% 非线性约束
function [c, ceq] = nonlinear_constraints(x)
    x1 = x(1); y1 = x(2); 
    x2 = x(3); y2 = x(4);
    R = x(5);
    alpha0 = x(6); alpha1 = x(7); 
    beta0 = x(8); beta1 = x(9);
    theta_start = x(10); 
    theta_end = x(11);
    
    b = 170 / (2*pi); % 螺距系数
    
    % --- 等式约束 (Geometry Closure) ---
    ceq = zeros(8, 1);
    
    % 坐标闭环约束 
    ceq(1) = x1 + 2*R*cos(alpha0) - b*theta_end*cos(theta_end);
    ceq(2) = y1 + 2*R*sin(alpha0) - b*theta_end*sin(theta_end);
    
    ceq(3) = x2 + R*cos(beta1) - b*theta_start*cos(theta_start);
    ceq(4) = y2 + R*sin(beta1) - b*theta_start*sin(theta_start);
    
    % 两个圆弧连接处的连续性
    ceq(5) = x1 + 2*R*cos(alpha1) - (x2 + R*cos(beta0)); 
    ceq(6) = y1 + 2*R*sin(alpha1) - (y2 + R*sin(beta0)); 
    
    ceq(7) = alpha1 - beta0; % 切线方向相同
    ceq(8) = f(theta_end) - g(alpha0); % 使用自定义函数匹配性质
    
    % --- 不等式约束 (Boundary conditions) ---
    c = zeros(10, 1);
    c(1) = b * theta_start - 450;    % b*theta_start <= 450
    c(2) = b * theta_end - 450;      % 似乎也是边界限制
    c(3) = -R;                       % R >= 0
    c(4) = -theta_start;             % theta_start >= 0
    c(5) = -theta_end;
    
    % 距离约束：某点必须在圆内
    c(6) = ((x1 + 2*R*cos(alpha1))^2 + (y1 + 2*R*sin(alpha1))^2) - 450^2;
    
    c(7) = R - 300;                 
    c(8) = 170 - R;                  
    
    c(9) = (alpha1 - alpha0) - pi;   % 角度差限制
    c(10) = (beta1 - beta0) - pi;
end

% 自定义辅助函数 f
function res = my_f(theta)
    res = (sin(theta) + theta * cos(theta)) / (cos(theta) - theta * sin(theta));
end

% 自定义辅助函数 g
function res = my_g(gamma)
    % 对应 PDF 中的 result = -1/tan(gamma)
    res = -1 / tan(gamma);
end