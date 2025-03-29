function show_traject(T_start,T_end,T_TIME,num)
    dh = struct('alpha0', 0, ...
                'alpha1', -pi/2, ...
                'alpha2', 0, ...
                'alpha3', -pi/2, ...
                'alpha4', pi/2, ...
                'alpha5', -pi/2, ...
                'a0', 0, ...
                'a1', 0.04, ...
                'a2', 0.18, ...
                'a3', 0.055, ...
                'a4', 0, ...
                'a5', 0, ...
                'd1', 0.212, ...
                'd2', 0.062, ...
                'd3', -0.05, ...
                'd4', 0.205027, ...
                'd5', 0, ...
                'd6', 0);
    theta123456_init = IK_algorithm(T_start, dh);
    theta123456_end = IK_algorithm(T_end, dh);
    
    % 保存 x, y, z, theta4, theta5, theta6
    position_init = [T_start(1,4), T_start(2,4), T_start(3,4), theta123456_init(4), theta123456_init(5), theta123456_init(6)];
    position_end = [T_end(1,4), T_end(2,4), T_end(3,4), theta123456_end(4), theta123456_end(5), theta123456_end(6)];
    
    for i = 1:6
        [stage(i,:),~,~,T_BASE] = linetraject(position_init(i) , position_end(i) , T_TIME , num);
    end

    % labels 对应 x, y, z, theta4, theta5, theta6
    labels = {'x', 'y', 'z', 'theta4', 'theta5', 'theta6'};
    figure;  % 新建一个 figure
    for i = 1:6
        subplot(6, 1, i); % 每次循环对应一个子图
        plot(T_BASE, stage(i,:), 'LineWidth', 2);
        xlabel('时间 (秒)');
        ylabel(labels{i});
        title([labels{i} ' 随时间的变化曲线']);
        grid on;
    end
    
    for i = 1:num
        T_mid = [1.0000,    0,         0,    stage(1,i);
                     0,   -1.0000,         0,    stage(2,i);
                     0,         0,   -1.0000,    stage(3,i);
                     0,         0,         0,    1.0000];
        theta123456_output(i,:) = IK_algorithm(T_mid, dh);
    end
    

    % 初始化机器人，并显示及演示计算得到的关节角序列
    base_xyz = [0 0 0];
    robot = puma560_robot(base_xyz);
    robot.display();    

    % 创建 figure，并在进入循环之前设置坐标轴反转
    figure;
    opengl hardware;
    axesHandle = gca;
    set(axesHandle, 'XDir', 'reverse', 'YDir', 'reverse');    
    % 初始化机器人图形，这一步会创建用于动画的句柄
    % 绘制机器人时降低绘图细节
    robot.plot(theta123456_output(1,:), 'delay', 0, 'fps', Inf, 'noshading', 'noshadow', ...
           'nowrist', 'noarrow', 'notiles', 'noname','trail', {'r', 'LineWidth', 2});

    % 使用循环逐次更新机器人各关节角度
    for i = 1:num
        robot.animate(theta123456_output(i,:));
        pause(0.002);  % 根据需要添加延时以减轻刷新压力
    end


end