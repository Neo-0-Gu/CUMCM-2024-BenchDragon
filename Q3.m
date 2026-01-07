clear, clc
a = 30; % 下界
c = 70; % 上界
m = 1; % 收敛阈值
step_size = 0.5; % 步长
best_d = 0; % 答案
min_reward = 1000; % 最小的reward值

% 第一题给定值
l1 = 286;
l2 = 165;
dt = 0.5;
v = 100; % 1m/s=100cm/s
% 木板的尺寸（长度和宽度）
length1 = 341; % 龙头的长度
width1 = 30; % 龙头的宽度
length2 = 220; % 其余板的长度
width2 = 30; % 其余板的宽度
% 定义孔的距离
edge_dist1 = 27.5; % 孔到木板边缘的距离
flag = 0;

% 遍历以步长0.5遍历d的值
for d = a:step_size:c
    b = d / (2*pi);
    flag_1 = 0; % 用于标记是否有碰撞
    flag_2 = 0; % 用于标记是否到达掉头范围
    temp_theta = zeros(224, 1000);
    temp_x = zeros(224, 1000);
    temp_y = zeros(224, 1000);
    temp_theta(1, 1) = 32 * pi;
    temp_time = 0;

    if flag == 1
        break
    end
    % 开始运动模拟
    for i = 1:1000
        if flag_1 == 1 || flag_2 == 1
            break
        end

        for j = 2:224 % 后续节点循环，j表示节点
            temp_x(j-1, i) = (b * temp_theta(j-1, i)) * cos(temp_theta(j-1, i)); %
                % i时间点，第j-1个节点x轴坐标
            temp_y(j-1, i) = (b * temp_theta(j-1, i)) * sin(temp_theta(j-1, i)); %
                % i时间点，第j-1个节点y轴坐标

            if j == 2
                distance_func = @(t) ((b * t) * cos(t) - temp_x(j-1, i))^2 + ((b * t) * sin(t) - temp_y(j-1, i))^2 - l1^2;
            else        
                distance_func = @(t) ((b * t) * cos(t) - temp_x(j-1, i))^2 + ((b * t) * sin(t) - temp_y(j-1, i))^2 - l2^2; 
            end
            % 设置初始值
            initial_guess = 0.01; % 初始值一定要大于上一个节点的值
            lower_bound = 0.01;   % 解的下界
            upper_bound = 2*pi;   % 解的上界

            max_attempts = 500;   % 最多尝试次数
            attempt = 0;

            % 寻找异号区间 [lower_bound, upper_bound]
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

            temp_theta(j, i) = fzero(distance_func, [lower_bound, upper_bound]);
            temp_x(j, i) = (b * temp_theta(j, i)) * cos(temp_theta(j, i)); %i时间点，第j个节点x轴坐标
            temp_y(j, i) = (b * temp_theta(j, i)) * sin(temp_theta(j, i)); %i时间点，第j个节点y轴坐标end
        end

        head_front = [temp_x(1,i), temp_y(1,i)]; % 第i时间点龙头前的x,y坐标

        % 判断是否进入调头区域 (半径 < 450cm)
        if norm(head_front) < 450
            flag_2 = 1;
            temp_time = i - 1;
        elseif norm(head_front) == 450
            flag_2 = 1;
            temp_time = i;
        end

        % 定义龙头后端的坐标 (假设连接点是第二个节点的坐标)
        head_last = zeros(1,2);
        head_last(1,1) = temp_x(2,i); 
        head_last(1,2) = temp_y(2,i);

        % 计算木板方向角 (相对于X轴)
        theta1 = atan2(head_last(1,2) - head_front(1,2), head_last(1,1) - head_front(1,1));

        % 旋转矩阵
        R1 = [cos(theta1), -sin(theta1); sin(theta1), cos(theta1)];

        % 定义未旋转的矩形顶点 (相对于孔的位置)
        unrotated_corners1 = [
            -edge_dist1, -width1/2;
            length1 - edge_dist1, -width1/2;
            length1 - edge_dist1, width1/2;
            -edge_dist1, width1/2
        ];

        % 计算旋转平移后的龙头矩形四个顶点
        head_board = (R1 * unrotated_corners1')' + head_front; 

        % 筛选可能碰撞的板凳 (距离相差2pi以内的)
        matrix_array = {};
        for k = 2:224
            if temp_theta(1,i) - temp_theta(k,i) < 2*pi % 只考虑附近的节点
                matrix_array{end + 1} = [temp_x(k,i), temp_y(k,i)];
            end
        end
        matrix_array(1) = []; % 去除第一块木板防止自交 (根据逻辑可能需要)

        % 存储其他板凳的顶点
        board_x_y_array = zeros(2, 4, length(matrix_array)-1);

        for k = 1:length(matrix_array)-1
            hole3 = matrix_array{k};   % 前孔
            hole4 = matrix_array{k+1}; % 后孔
            
            % 计算第k块板的角度
            theta2 = atan2(hole4(2) - hole3(2), hole4(1) - hole3(1));
            
            % 定义第k块板的未旋转顶点
            unrotated_corners2 = [
                -edge_dist1, -width2/2;
                length2 - edge_dist1, -width2/2;
                length2 - edge_dist1, width2/2;
                -edge_dist1, width2/2
            ];
            
            R2 = [cos(theta2), -sin(theta2); sin(theta2), cos(theta2)];
            corners_board2 = (R2 * unrotated_corners2')' + hole3;
            
            % 存入数组
            for q = 1:4
                board_x_y_array(1,q,k) = corners_board2(q,1);
                board_x_y_array(2,q,k) = corners_board2(q,2);
            end
        end

        % 核心碰撞检测：判断龙头矩形是否与其他矩形相交
        for k = 1:4 % 龙头的4个顶点
            % ... (此处逻辑稍微简化，通常应该是双向判断)
            % 检查点是否在多边形内
            temp_x_y_check = [head_board(k, 1), head_board(k, 2)];
            
            for q = 1:length(matrix_array)-1
                temp_xv = [board_x_y_array(1,1,q), board_x_y_array(1,2,q), board_x_y_array(1,3,q), board_x_y_array(1,4,q)];
                temp_yv = [board_x_y_array(2,1,q), board_x_y_array(2,2,q), board_x_y_array(2,3,q), board_x_y_array(2,4,q)];
                
                [in, on] = inpolygon(temp_x_y_check(1,1), temp_x_y_check(1,2), temp_xv, temp_yv);
                
                if any(in == 1) || any(on == 1)
                    disp(['Collision at step: ', num2str(i)]);
                    flag_1 = 1;
                    break; 
                end
            end
        end
        % 计算龙头节点的delta_theta
        distance_func = @(t) (2 * v * dt) / (b * (sqrt(1 + temp_theta(1, i)^2) + sqrt(1 + (temp_theta(1, i) + t)^2))) - t;
        initial_guess = 0.01;
        lower_bound = 0.01;
        upper_bound = 2 * pi;

        attempt = 0;
        while distance_func(lower_bound) * distance_func(upper_bound) > 0 && attempt <
            max_attempts
            upper_bound = upper_bound - 0.01;
            if upper_bound <= lower_bound
                lower_bound = lower_bound - 0.01;
            end
            attempt = attempt + 1;
        end

        if attempt == max_attempts
            error('Unable to find an interval with differing signs for fzero at index %d', i);
        end

        delta_theta = fzero(distance_func, [lower_bound, upper_bound]);
        temp_theta(1, i+1) = temp_theta(1, i) - delta_theta;
    end

    %d = d - 30/2;
    % 计算reward函数值

    % 计算 Reward / 目标函数
    if flag_1 == 1
        f_x = 1000; % 惩罚
    elseif flag_2 == 1
        f_x = abs(norm([temp_x(1, temp_time), temp_y(1, temp_time)])- 450); 
    else
        f_x = 1000;
    end

    % 更新最优值
    if f_x < min_reward
        min_reward = f_x;
        best_d = d; % d 是螺距参数
        flag = 1;
    end
end

fprintf('最优的d值为: %.2f，最小reward为: %.2f\n，时间为: %.1f\n', best_d, min_reward,temp_time);