function spine_visualization()
    % 实时脊柱姿态可视化 - MATLAB版本
    % 串口设置
    com_port = 14;
    baud_rate = 115200;
    
    try
        % 创建串口对象
        s = serial(sprintf('COM%d', com_port), 'BaudRate', baud_rate, 'Timeout', 0.1);
        fopen(s);
        fprintf('成功连接到COM%d\n', com_port);
    catch ME
        fprintf('串口连接失败: %s\n', ME.message);
        return;
    end
    
    % 创建图形窗口
    figure('Name', '实时脊柱姿态可视化', 'NumberTitle', 'off', 'Position', [100, 100, 1200, 800]);
    ax = axes('Parent', gcf);
    hold(ax, 'on');
    grid(ax, 'on');
    
    % 设置坐标轴
    xlabel(ax, 'X (前)');
    ylabel(ax, 'Y (左)');
    zlabel(ax, 'Z (上)');
    title(ax, '实时脊柱姿态可视化');
    axis(ax, [-0.3, 0.3, -0.3, 0.3, 0, 0.5]);
    view(ax, 3);
    
    % 脊柱参数
    spine_length = 0.4;  % 脊柱总长度 (米)
    segment_length = spine_length / 4;  % 每段长度
    
    % 初始化绘图对象
    spine_line = plot3(ax, NaN, NaN, NaN, 'b-o', 'LineWidth', 3, 'MarkerSize', 8);
    imu_points = zeros(5, 1);
    imu_labels = zeros(5, 1);
    
    for i = 1:5
        imu_points(i) = plot3(ax, NaN, NaN, NaN, 'ro', 'MarkerSize', 10);
        imu_labels(i) = text(ax, NaN, NaN, NaN, sprintf('IMU%d', i), 'FontSize', 8, 'Color', 'red');
    end
    
    legend(ax, '脊柱', 'Location', 'best');
    
    % 主循环
    running = true;
    fprintf('启动实时可视化，按 Ctrl+C 停止...\n');
    
    while running
        try
            % 检查串口是否有数据
            if s.BytesAvailable > 0
                % 读取一行数据
                line = fgetl(s);
                
                if ~isempty(line)
                    % 解析数据
                    data = str2double(strsplit(line, ','));
                    
                    if length(data) == 16  % 时间戳 + 5个IMU * 3个角度
                        imu_angles = data(2:end);  % 跳过时间戳
                        
                        % 计算脊柱点
                        spine_points = calculate_spine_points(imu_angles, segment_length);
                        
                        % 更新脊柱线条
                        set(spine_line, 'XData', spine_points(:, 1), ...
                                      'YData', spine_points(:, 2), ...
                                      'ZData', spine_points(:, 3));
                        
                        % 更新IMU点位置
                        for i = 1:5
                            set(imu_points(i), 'XData', spine_points(i, 1), ...
                                             'YData', spine_points(i, 2), ...
                                             'ZData', spine_points(i, 3));
                            
                            set(imu_labels(i), 'Position', spine_points(i, :));
                        end
                        
                        % 刷新图形
                        drawnow;
                    end
                end
            end
            
            % 短暂暂停
            pause(0.01);
            
        catch ME
            fprintf('错误: %s\n', ME.message);
            break;
        end
    end
    
    % 清理
    fclose(s);
    delete(s);
    clear s;
    fprintf('可视化已停止\n');
end

function spine_points = calculate_spine_points(imu_angles, segment_length)
    % 根据IMU角度计算脊柱各点位置
    spine_points = zeros(6, 3);
    current_pos = [0, 0, 0];
    current_orientation = eye(3);
    
    % 计算每个脊柱段的位置
    for i = 1:5
        % 获取当前IMU的欧拉角
        roll = imu_angles((i-1)*3 + 1);
        pitch = imu_angles((i-1)*3 + 2);
        yaw = imu_angles((i-1)*3 + 3);
        
        % 计算旋转矩阵
        R = euler_to_rotation_matrix(roll, pitch, yaw);
        
        % 更新当前方向
        current_orientation = R * current_orientation;
        
        % 计算下一段的方向向量 (Z轴方向)
        direction = current_orientation * [0; 0; 1];
        
        % 计算下一段的位置
        next_pos = current_pos + (direction * segment_length)';
        
        spine_points(i, :) = current_pos;
        current_pos = next_pos;
    end
    
    % 添加最后一个点
    spine_points(6, :) = current_pos;
end

function R = euler_to_rotation_matrix(roll, pitch, yaw)
    % 欧拉角转旋转矩阵
    % 转换为弧度
    roll = deg2rad(roll);
    pitch = deg2rad(pitch);
    yaw = deg2rad(yaw);
    
    % 旋转矩阵
    Rx = [1, 0, 0;
          0, cos(roll), -sin(roll);
          0, sin(roll), cos(roll)];
    
    Ry = [cos(pitch), 0, sin(pitch);
          0, 1, 0;
          -sin(pitch), 0, cos(pitch)];
    
    Rz = [cos(yaw), -sin(yaw), 0;
          sin(yaw), cos(yaw), 0;
          0, 0, 1];
    
    R = Rz * Ry * Rx;
end