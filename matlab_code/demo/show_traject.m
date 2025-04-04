function show_traject(x_init,y_init,z_init,alpha_init,beta_init,gamma_init,...
                      x_end,y_end,z_end,alpha_end,beta_end,gamma_end,...
                      T_TIME,num)
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
    
    % 保存 x, y, z, theta4, theta5, theta6
    position_init = [x_init, y_init, z_init, alpha_init,beta_init,gamma_init];
    position_end = [x_end,y_end,z_end,alpha_end,beta_end,gamma_end];
    disp("init");
    disp(position_init);
    disp("end");
    disp(position_end);
    for i = 1:6
        [stage(i,:),~,~,T_BASE] = linetraject(position_init(i) , position_end(i) , T_TIME , num);
    end
    disp(stage);

    % labels 对应 x, y, z, alpha,beta,gamma
    labels = {'x', 'y', 'z', 'alpha','beta','gamma'};
    figure;  % 新建一个 figure
    for i = 1:6
        subplot(6, 1, i); % 每次循环对应一个子图
        plot(T_BASE, stage(i,:), 'LineWidth', 2);
        xlabel('时间 (秒)');
        ylabel(labels{i});
        title([labels{i} ' 随时间的变化曲线']);
        grid on;
    end

    alpha_matrix(1,:) = stage(4,:);
    beta_matrix(1,:)  = stage(5,:);
    gamma_matrix(1,:) = stage(6,:);
    
    % 遍历进行变换与逆运动学计算
    for i = 1:num
        % 构造变换矩阵 T_mid
        T_mid = [cos(beta_matrix(i))*cos(gamma_matrix(i)),    cos(gamma_matrix(i))*sin(alpha_matrix(i))*sin(beta_matrix(i)) - cos(alpha_matrix(i))*sin(gamma_matrix(i)),    cos(alpha_matrix(i))*cos(gamma_matrix(i))*sin(beta_matrix(i)) + sin(alpha_matrix(i))*sin(gamma_matrix(i)),    stage(1,i);
                 cos(beta_matrix(i))*sin(gamma_matrix(i)),    cos(alpha_matrix(i))*cos(gamma_matrix(i)) + sin(alpha_matrix(i))*sin(beta_matrix(i))*sin(gamma_matrix(i)),    -cos(gamma_matrix(i))*sin(alpha_matrix(i)) + cos(alpha_matrix(i))*sin(beta_matrix(i))*sin(gamma_matrix(i)),   stage(2,i);
                                     -sin(beta_matrix(i)),                                                                     cos(beta_matrix(i))*sin(alpha_matrix(i)),                                                                      cos(alpha_matrix(i))*cos(beta_matrix(i)),   stage(3,i);
                                                        0,                                                                                                            0,                                                                                                             0,      1.0000];
        
        % 调用逆运动学算法函数
        theta123456_output(i,:) = IK_algorithm(T_mid, dh);
        
        % 输出结果到命令行
        disp(theta123456_output(i,:));
        disp(i);
    end

    % labels 对应 theta1,theta2,theta3,theta4,theta5,theta6
    labels_2 = {'theta1','theta2','theta3','theta4','theta5','theta6'};
    figure;  % 新建一个 figure
    for i = 1:6
        subplot(6, 1, i); % 每次循环对应一个子图
        plot(T_BASE, theta123456_output(:,i), 'LineWidth', 2);
        xlabel('时间 (秒)');
        ylabel(labels_2{i});
        title([labels_2{i} ' 随时间的变化曲线']);
        grid on;
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
    robot.plot(theta123456_output(1,:), 'delay', 0, 'fps', Inf, 'noshadow', ...
           'nowrist', 'notiles', 'noname','trail', {'r', 'LineWidth', 2});

    % 使用循环逐次更新机器人各关节角度
    for i = 1:num
        robot.animate(theta123456_output(i,:));
      % % 2. 计算末端位姿（齐次变换矩阵）
      %   T_end = robot.fkine(theta123456_output(1,:));
      % 
      %   % 3. 叠加显示末端坐标系
      %   trplot(T_end, 'frame', 'E', 'color', 'blue', 'length', 0.05);
        pause(0.002);  % 根据需要添加延时以减轻刷新压力
    end


end