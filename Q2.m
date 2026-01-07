clear,clc
% 第一题给定值
l1 = 286;
l2 = 165;
b = 55 / (2*pi);
dt = 1;
v = 100; % 1m/s=100cm/s
flag = 0;
% 木板的尺寸（长度和宽度）
length1 = 341; % 龙头的长度
width1 = 30; % 龙头的宽度
length2 = 220; % 其余板的长度
width2 = 30; % 其余板的宽度

% 定义孔的距离
edge_dist1 = 27.5; % 孔到木板边缘的距离
theta = zeros(30,500);
theta(1,1) = 57.144840319137955; %假设从第300秒开始
x = zeros(30,500);
y = zeros(30,500);
for i=1:200 %时间循环，i表示秒
    if flag == 1
        break
    end
    for j=2:30 %后续节点循环，j表示节点
        x(j-1,i) = (b * theta(j-1,i)) * cos(theta(j-1,i)); %i秒，第j-1个节点x轴坐标
        y(j-1,i) = (b * theta(j-1,i)) * sin(theta(j-1,i)); %i秒，第j-1个节点y轴坐标
        if j == 2
            distance_func = @(t) ( ( (b * t) * cos(t) - x(j-1,i) )^2 + ( ( b * t) * sin(t) - y(j-1,i) )^2 ) - l1^2;
        else
            distance_func = @(t) ( ( (b * t) * cos(t) - x(j-1,i) )^2 + ( ( b * t) * sin(t) - y(j-1,i) )^2 ) - l2^2;
        end
        % 设置初始值
        initial_guess = theta(j-1,i) + 0.01; % 初始值一定要大于上一个节点的值
        lower_bound = theta(j-1,i) + 0.01; % 解的下界
        upper_bound = theta(j-1,i) + 2*pi; % 解的上界

        max_attempts = 500;%最多尝试次数
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
        x(j,i) = (b * theta(j,i)) * cos(theta(j,i)); %i秒，第j个节点x轴坐标
        y(j,i) = (b * theta(j,i)) * sin(theta(j,i)); %i秒，第j个节点y轴坐标
    end


    head_front = zeros(1,2); %第i秒龙头前的x，y坐标
    head_front(1,1) = x(1,i); %x坐标
    head_front(1,2) = y(1,i); %y坐标
    head_last = zeros(1,2); %第i秒龙头后的x，y坐标
    head_last(1,1) = x(2,i); %x坐标
    head_last(1,2) = y(2,i); %y坐标% 计算木板的方向角（相对于X轴），通过两孔确定方向
    theta1 = atan2(head_last(1,2) - head_front(1,2), head_last(1,1) - head_front(1,1)); %龙头的角度
    % 定义旋转矩阵
    R1 = [cos(theta1), -sin(theta1); sin(theta1), cos(theta1)]; % 龙头的旋转矩阵
    % 相对于孔的位置定义未旋转的顶点
    unrotated_corners1 = [
        -edge_dist1, -width1/2;
        length1 - edge_dist1, -width1/2;
        length1 - edge_dist1, width1/2;
        -edge_dist1, width1/2
    ];
    head_board = (R1 * unrotated_corners1')' + head_front; % 第一块木板顶点坐标
    %disp('第一块木板的顶点坐标：');
    %disp(head_board);
    %disp(size(head_board));

    %定义一个cell数组，用于存储符合条件的 (1, 2) 矩阵
    matrix_array = {};
    for k = 2:30
        if theta(1,i)-theta(k,i)<2*pi %只考虑与第一个节点差为2pi的节点
            matrix_array{end + 1} = [x(k,i),y(k,i)];
        end
    end
    matrix_array(1) = []; %去除第一块木板，防止干扰
    board_x_y_array = zeros(2,4,length(matrix_array)-1); %存储有可能相交的矩阵的顶点x，y坐标
    
    for k = 1:length(matrix_array)-1
        hole3 = matrix_array{k};
        hole4 = matrix_array{k+1};
        %disp(size(hole3));
        theta2 = atan2(hole4(2) - hole3(2), hole4(1) - hole3(1)); % 木板的角度
        % 相对于孔的位置定义未旋转的顶点
        unrotated_corners2 = [
            -edge_dist1, -width2/2;
            length2 - edge_dist1, -width2/2;
            length2 - edge_dist1, width2/2;
            -edge_dist1, width2/2
        ];
        R2 = [cos(theta2), -sin(theta2); sin(theta2), cos(theta2)]; % 木板的旋转矩阵
        corners_board2 = (R2 * unrotated_corners2')' + hole3; % 木板顶点坐标
        for q=1:4
            board_x_y_array(1,q,k) = corners_board2(q,1);
            board_x_y_array(2,q,k) = corners_board2(q,2);
        end
    end
    %分别判断龙头的四个点是否与其他板有交点
    for k = 1:4
        temp_x_y = [head_board(k,1),head_board(k,2)]; %用于判断的龙头顶点
        for q = 1:length(matrix_array)-1
            temp_xv = [board_x_y_array(1,1,q),board_x_y_array(1,2,q),board_x_y_array(1,3,q),board_x_y_array(1,4,q)];
            temp_yv = [board_x_y_array(2,1,q),board_x_y_array(2,2,q),board_x_y_array(2,3,q),board_x_y_array(2,4,q)];
            [in, on] = inpolygon(temp_x_y(1,1),temp_x_y(1,2),temp_xv,temp_yv);
            if any(in == 1) || any(on == 1)
                disp(i);
                flag = 1;
                break; % 假设此代码在一个循环中，遇到 1 就会终止
            end

        end
    end

    %计算龙节头点的delta_theta
    delta_theta = 0;
    distance_func = @(t) ( 2 * v * dt ) / ( b * ( sqrt ( 1 + theta(1,i) ^ 2 ) + sqrt ( 1 + ( theta(1,i) + t ) ^ 2 ) ) ) - t;
    % 设置初始值
    initial_guess = 0.01; % 初始值一定要大于上一个节点的值
    lower_bound = 0.01; % 解的下界
    upper_bound = 2*pi; % 解的上界

    max_attempts = 500;%最多尝试次数
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