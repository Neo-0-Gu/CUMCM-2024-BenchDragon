clear,clc
% 给定值l1 = 286;
l2 = 165;
b = 55 / (2*pi);
dt = 0.1;
n = 300/dt+1;
v = 100; % 1m/s=100cm/s
% 初始化theta = zeros(224,n);
theta(1,1) = 32*pi;
x = zeros(224,n);
y = zeros(224,n);
x_ans = zeros(224,301);
y_ans = zeros(224,301);
% 计算坐标
for i=1:n % 时间循环，i表示时间节点
    for j=2:224 % 后续节点循环，j表示节点
        x(j-1,i) = (b * theta(j-1,i)) * cos(theta(j-1,i)); % i个时间节点，第j-1个节点x轴坐标
        y(j-1,i) = (b * theta(j-1,i)) * sin(theta(j-1,i)); % i个时间节点，第j-1个节点y轴坐标
        
        if j == 2
            distance_func = @(t) ( ( (b * t) * cos(t) - x(j-1,i) )^2 + ( ( b * t) * sin(t) - y(j-1,i) )^2 ) - l1^2;
        else
            distance_func = @(t) ( ( (b * t) * cos(t) - x(j-1,i) )^2 + ( ( b * t) * sin(t) - y(j-1,i) )^2 ) - l2^2;
        end
        % 设置初始值
        initial_guess = theta(j-1,i) + 0.01; % 初始值一定要大于上一个节点的值
        lower_bound = theta(j-1,i) + 0.01; % 解的下界
        upper_bound = theta(j-1,i) + 2*pi; % 解的上界

        max_attempts = 500; % 最多尝试次数
        attempt = 0;
        while distance_func(lower_bound) * distance_func(upper_bound) > 0 && attempt < max_attempts
            upper_bound = upper_bound - 0.01; % 逐渐降低上界
            if upper_bound <= lower_bound
            % 若有需要，扩展下界
                lower_bound = lower_bound - 0.01;
            end
            attempt = attempt + 1;
        end

        if attempt == max_attempts
            error('Unable to find an interval with differing signs for fzero at index %d', i);
        end
        theta(j,i) = fzero(distance_func, [lower_bound, upper_bound]);
    end

    % 计算龙头节点的delta_theta
    delta_theta = 0;
    distance_func = @(t) ( 2 * v * dt ) / ( b * ( sqrt ( 1 + theta(1,i) ^ 2 ) + sqrt ( 1 + (theta(1,i) + t ) ^ 2 ) ) ) - t;

    % 设置初始值
    initial_guess = 0.01; % 初始值一定要大于上一个节点的值
    lower_bound = 0.01; % 解的下界
    upper_bound = 2*pi; % 解的上界

    max_attempts = 500; % 最多尝试次数
    attempt = 0;
    while distance_func(lower_bound) * distance_func(upper_bound) > 0 && attempt < max_attempts
        upper_bound = upper_bound - 0.01; % 逐渐降低上界
        if upper_bound <= lower_bound
            % 若有需要，扩展下界
            lower_bound = lower_bound - 0.01;
        end
        attempt = attempt + 1;
    end

    if attempt == max_attempts
        error('Unable to find an interval with differing signs for fzero at index %d', i);
    end

    delta_theta = fzero(distance_func, [lower_bound, upper_bound]);
    theta(1,i+1) = theta(1,i) - delta_theta;
end

for i = 1:n
    x(224,i) = (b * theta(224,i)) * cos(theta(224,i));
    y(224,i) = (b * theta(224,i)) * sin(theta(224,i));
end
% x，y坐标答案
for i = 2:301
    for j =1:224
        x_ans(j,i) = x(j,(i-1)*(1/dt));
        y_ans(j,i) = y(j,(i-1)*(1/dt));
    end
end

for i = 1:224
    x_ans(i,1) = x(i,1);
    y_ans(i,1) = y(i,1);
end

% 计算速度矩阵
speed = zeros(224,n); % 224个节点, n个时间点
speed_ans = zeros(224,301);
for i = 1:n-1
    for j = 1:224
        if i == 1
            speed(j,i) = 0;
        else
        % 其余时间点, 用前后2dt秒的距离除以2dt
        distance = sqrt((x(j,i+1) - x(j,i-1))^2 + (y(j,i+1) - y(j,i-1))^2);
        speed(j,i) = distance / (2 * dt);
        end
    end
end

% 处理最后一个时间点的速度
for j = 1:224
    distance = sqrt((x(j,n) - x(j,n-1))^2 + (y(j,n) - y(j,n-1))^2);
    speed(j,n) = distance / dt;
end

% 速度答案
for i = 2:301
    for j =1:224
        speed_ans(j,i) = speed(j,(i-1)*(1/dt));
    end
end

for i = 1:224
    speed_ans(i,1) = 0;
end

% 将x_ans和y_ans交替写入Excel文件
filename = '.\result1.xlsx';
sheetname = '位置';

% 将数据乘以0.01
x_ans = x_ans * 0.01

y_ans = y_ans * 0.01;
% 合并 x_ans 和 y_ans 并交替写入
[row_count, col_count] = size(x_ans);
combined_data = zeros(2 * row_count, col_count);

for i = 1:row_count
    combined_data(2*i-1, :) = x_ans(i, :); % 写入 x_ans 的一行
    combined_data(2*i, :) = y_ans(i, :); % 写入 y_ans 的一行
end

% 写入到 Excel，从第二行第二列开始
writematrix(combined_data, filename, 'Sheet', sheetname, 'Range', 'B2');