clear; clc;

%% 已知条件
% 已知DH参数
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
        
% 已知工具坐标系的初始位置
T0_C_init = [1.0000,         0,         0,    0.2450;
             0,   -1.0000,         0,    0.0120;
             0,         0,   -1.0000,    0.4470;
             0,         0,         0,    1.0000];
         
% 已知工具坐标系的中间位置
T0_C_middle = [0.583359, -0.492574,  0.645804,  0.046752;
               0.346615, -0.568099, -0.746406, -0.009192;
               0.734540,  0.659268, -0.160673,  0.568912;
               0,         0,         0,         1.000000];
           
% 已知工具坐标系的最终位置
T0_C_end = [  -0.0154,    0.0764,   -0.9970,    0.2391;
              -0.2596,   -0.9632,   -0.0698,    0.1487;
              -0.9656,    0.2578,    0.0347,    0.2212;
                    0,         0,         0,    1.0000;];
        
% 已知工具坐标系相对于6坐标系的变换矩阵
T6_C = [1, 0, 0, 0;
        0, 1, 0, 0;
        0, 0, 1, 0.0865;
        0, 0, 0, 1];
    
% 定义阶段时间变量
Completion_time1 = 10;    % 示例值（请根据实际情况修改）
Completion_time2 = 10;    % 示例值（请根据实际情况修改）
t1 = linspace(0, Completion_time1, 100);       % 阶段1：0~Completion_time1（100 个点）
t2 = linspace(Completion_time1, Completion_time1 + Completion_time2, 100); % 阶段2：Completion_time1~(Completion_time1+Completion_time2)（100 个点）
% 合并时间向量，去掉阶段衔接处重复的点（总长度 = 100 + 99 = 199）
t = [t1, t2(2:end)];
N = length(t);


% 得到 T0_6_init, T0_6_middle, T0_6_end（注意这里使用 inv(T6_C) 求逆）
T0_6_init = T0_C_init %* inv(T6_C);
T0_6_middle = T0_C_middle %* inv(T6_C);
T0_6_end =  T0_C_end %* inv(T6_C);

%% 建立速度表和加速度表
% 通过逆解得到 4,5,6 号角（假定 IK_algorithm 函数已定义）
theta123456_init = IK_algorithm(T0_C_init, dh);
theta123456_middle = IK_algorithm(T0_C_middle, dh);
theta123456_end = IK_algorithm(T0_C_end, dh);

% 保存 x, y, z, theta4, theta5, theta6
position_init = [T0_C_init(1,4), T0_C_init(2,4), T0_C_init(3,4), theta123456_init(4), theta123456_init(5), theta123456_init(6)];
position_middle = [T0_6_middle(1,4), T0_6_middle(2,4), T0_6_middle(3,4), theta123456_middle(4), theta123456_middle(5), theta123456_middle(6)];
position_end = [T0_6_end(1,4), T0_6_end(2,4), T0_6_end(3,4), theta123456_end(4), theta123456_end(5), theta123456_end(6)];

% 计算各段的速度（假设为匀速运动）
Speed = zeros(2, 6);
Speed(1,:) = (position_middle - position_init) / Completion_time1;
Speed(2,:) = (position_end - position_middle) / Completion_time2;

% 加速度示例（具体值根据实际情况调整）
Acceleration = [0,    0,    0,    0,    0,    0;  
                0,    0,    0,    0,    0,    0];

%% 构建工具位姿轨迹 (position)
% position 的每一行为 [x, y, z, theta4, theta5, theta6]
% 其中position_init, position_middle, position_end 均为 1×6 向量（平移部分来自 T0_C，腕部角由 IK 算法计算得到）
position_stage1 = repmat(position_init, length(t1), 1) + ...
                  repmat((position_middle - position_init), length(t1), 1) .* (t1' / Completion_time1);
tau = t2 - Completion_time1;  % tau 取值范围：0 ~ Completion_time2
position_stage2 = repmat(position_middle, length(t2), 1) + ...
                  repmat((position_end - position_middle), length(t2), 1) .* (tau' / Completion_time2);
% 拼接两个阶段的轨迹（去掉阶段衔接处重复的点）
position = [position_stage1; position_stage2(2:end, :)];

%% 图1：绘制工具位姿轨迹各分量随时间变化
figure(1);
labels = {'X', 'Y', 'Z', '\theta_4', '\theta_5', '\theta_6'};
for i = 1:6
    subplot(2,3,i);
    plot(t, position(:, i), 'LineWidth', 2);
    xlabel('时间 (秒)');
    ylabel(labels{i});
    title([labels{i} ' 随时间的变化']);
    grid on;
end

%% 图2：绘制工具位姿的 3D 笛卡尔轨迹（仅取平移部分 X, Y, Z）
figure(2);
plot3(position(:,1), position(:,2), position(:,3), 'b-', 'LineWidth', 2);
hold on;
% 定义关键点索引：第一行、数据中间行、最后一行
idx1 = 1;
idx2 = round(length(t)/2);
idx3 = length(t);
plot3(position(idx1,1), position(idx1,2), position(idx1,3), 'ro', 'MarkerSize', 8, 'LineWidth',2);
text(position(idx1,1), position(idx1,2), position(idx1,3), '  初始点', 'Color','r','FontSize',10);
plot3(position(idx2,1), position(idx2,2), position(idx2,3), 'go', 'MarkerSize', 8, 'LineWidth',2);
text(position(idx2,1), position(idx2,2), position(idx2,3), '  中间点', 'Color','g','FontSize',10);
plot3(position(idx3,1), position(idx3,2), position(idx3,3), 'mo', 'MarkerSize', 8, 'LineWidth',2);
text(position(idx3,1), position(idx3,2), position(idx3,3), '  终止点', 'Color','m','FontSize',10);
% 绘制参考坐标系：以初始点为原点
ref = position(1,1:3);
scale = max(max(abs(position(:,1:3)))) * 0.1;
quiver3(ref(1), ref(2), ref(3), scale, 0, 0, 'r', 'LineWidth',2, 'MaxHeadSize',0.5);
quiver3(ref(1), ref(2), ref(3), 0, scale, 0, 'g', 'LineWidth',2, 'MaxHeadSize',0.5);
quiver3(ref(1), ref(2), ref(3), 0, 0, scale, 'b', 'LineWidth',2, 'MaxHeadSize',0.5);
hold off;
xlabel('X'); ylabel('Y'); zlabel('Z');
title('工具位姿 3D 笛卡尔轨迹');
grid on;
axis equal;

%% 图3：构建机器人关节角轨迹（θ1～θ6）
% 说明：利用工具位姿轨迹 pos (position)，构造工具位姿矩阵 (旋转部分取单位矩阵)、
% 调用 IK_algorithm 得到完整 1×6 关节角向量，将得到的前 3 个关节角与将腕部角（pos(:,4:6)）拼接，
% 得到完整的关节角轨迹 joint_traj。
joint_traj = zeros(N, 6);
for idx = 1:N
    p_interp = position(idx, 1:3)';  % 平移部分, 列向量
    theta456_interp = position(idx, 4:6);  % 腕部角
    T0_C_interp = [eye(3), p_interp; 0 0 0 1];
    theta_sol = IK_algorithm(T0_C_interp, dh);
    joint_traj(idx, :) = [theta_sol(1:3), theta456_interp];
end

figure(3);
joint_labels = {'\theta_1', '\theta_2', '\theta_3', '\theta_4', '\theta_5', '\theta_6'};
for i = 1:6
    subplot(2,3,i);
    plot(t, joint_traj(:, i), 'LineWidth', 2);
    xlabel('时间 (秒)');
    ylabel(joint_labels{i});
    title([joint_labels{i} ' 随时间变化']);
    grid on;
end

%% 图4：正运动学检查 —— 利用 joint_traj 的全部 6 个关节角求末端 3D 笛卡尔轨迹
cartesian_traj_fk = zeros(N, 3);
for idx = 1:N
    % 提取6个关节角
    theta1 = joint_traj(idx, 1);
    theta2 = joint_traj(idx, 2);
    theta3 = joint_traj(idx, 3);
    theta4 = joint_traj(idx, 4);
    theta5 = joint_traj(idx, 5);
    theta6 = joint_traj(idx, 6);

    % 调用 forward_kinematics 函数
    T06 = forward_kinematics(theta1, theta2, theta3, theta4, theta5, theta6, dh);
    
    % 提取 T06 的平移部分作为末端位置
    cartesian_traj_fk(idx, :) = T06(1:3, 4)';
end

% 绘制末端 3D 笛卡尔轨迹
figure(4);
plot3(cartesian_traj_fk(:,1), cartesian_traj_fk(:,2), cartesian_traj_fk(:,3), 'b-', 'LineWidth', 2);
hold on;
% 标注关键点：初始、中间、终止
idx1 = 1;
idx2 = round(N/2);
idx3 = N;
plot3(cartesian_traj_fk(idx1,1), cartesian_traj_fk(idx1,2), cartesian_traj_fk(idx1,3), 'ro', 'MarkerSize', 8, 'LineWidth', 2);
text(cartesian_traj_fk(idx1,1), cartesian_traj_fk(idx1,2), cartesian_traj_fk(idx1,3), '  初始点', ...
    'Color', 'r', 'FontSize', 10);
plot3(cartesian_traj_fk(idx2,1), cartesian_traj_fk(idx2,2), cartesian_traj_fk(idx2,3), 'go', 'MarkerSize', 8, 'LineWidth', 2);
text(cartesian_traj_fk(idx2,1), cartesian_traj_fk(idx2,2), cartesian_traj_fk(idx2,3), '  中间点', ...
    'Color', 'g', 'FontSize', 10);
plot3(cartesian_traj_fk(idx3,1), cartesian_traj_fk(idx3,2), cartesian_traj_fk(idx3,3), 'mo', 'MarkerSize', 8, 'LineWidth', 2);
text(cartesian_traj_fk(idx3,1), cartesian_traj_fk(idx3,2), cartesian_traj_fk(idx3,3), '  终止点', ...
    'Color', 'm', 'FontSize', 10);

% 绘制参考坐标系：以轨迹起始点为原点
ref = cartesian_traj_fk(1, :);
scale = max(max(abs(cartesian_traj_fk))) * 0.1;
quiver3(ref(1), ref(2), ref(3), scale, 0, 0, 'r', 'LineWidth', 2, 'MaxHeadSize', 0.5); % X轴
quiver3(ref(1), ref(2), ref(3), 0, scale, 0, 'g', 'LineWidth', 2, 'MaxHeadSize', 0.5); % Y轴
quiver3(ref(1), ref(2), ref(3), 0, 0, scale, 'b', 'LineWidth', 2, 'MaxHeadSize', 0.5); % Z轴
hold off;
xlabel('X'); ylabel('Y'); zlabel('Z');
title('全6轴正运动学 3D 笛卡尔轨迹及参考坐标系');
grid on;
axis equal;


%% 平滑两段工具位姿轨迹（linearBlendTrajectory function）
% 初始化
smoothed_stage1 = zeros(size(position_stage1));  % 第一阶段平滑结果
smoothed_stage2 = zeros(size(position_stage2));  % 第二阶段平滑结果
Tb1 = Completion_time1 * 0.5;                   % 第一阶段加速/减速段时间
Tb2 = Completion_time2 * 0.2;                   % 第二阶段加速/减速段时间

% 平滑处理第一阶段 (position_stage1)
for i = 1:6
    [smoothed_stage1(:, i), ~, ~, ~] = linearBlendTrajectory( ...
        position_stage1(1, i), position_stage1(end, i), Completion_time1, Tb1, size(position_stage1, 1));
end

% 平滑处理第二阶段 (position_stage2)
for i = 1:6
    [smoothed_stage2(:, i), ~, ~, ~] = linearBlendTrajectory( ...
        position_stage2(1, i), position_stage2(end, i), Completion_time2, Tb2, size(position_stage2, 1));
end

% 合并两段平滑结果
smoothed_pos = [smoothed_stage1; smoothed_stage2(2:end, :)];

%% 图5：绘制平滑后的轨迹各分量随时间变化
figure(5);
labels = {'X', 'Y', 'Z', '\theta_4', '\theta_5', '\theta_6'};
for i = 1:6
    subplot(2, 3, i);
    plot(t, smoothed_pos(:, i), 'LineWidth', 2);
    xlabel('时间 (秒)');
    ylabel(labels{i});
    title([labels{i} ' 平滑后的曲线']);
    grid on;
end

%% 图6：绘制平滑后的 3D 笛卡尔轨迹
figure(6);
plot3(smoothed_pos(:,1), smoothed_pos(:,2), smoothed_pos(:,3), 'b-', 'LineWidth', 2);
hold on;
% 标注起点和终点
plot3(smoothed_pos(1,1), smoothed_pos(1,2), smoothed_pos(1,3), 'ro', 'MarkerSize', 8, 'LineWidth', 2);
text(smoothed_pos(1,1), smoothed_pos(1,2), smoothed_pos(1,3), '  起点', 'Color', 'r', 'FontSize', 10);
plot3(smoothed_pos(end,1), smoothed_pos(end,2), smoothed_pos(end,3), 'go', 'MarkerSize', 8, 'LineWidth', 2);
text(smoothed_pos(end,1), smoothed_pos(end,2), smoothed_pos(end,3), '  终点', 'Color', 'g', 'FontSize', 10);
% 添加参考坐标系
ref = smoothed_pos(1, 1:3);
scale = max(max(abs(smoothed_pos(:,1:3)))) * 0.1;
quiver3(ref(1), ref(2), ref(3), scale, 0, 0, 'r', 'LineWidth', 2, 'MaxHeadSize', 0.5); % X轴
quiver3(ref(1), ref(2), ref(3), 0, scale, 0, 'g', 'LineWidth', 2, 'MaxHeadSize', 0.5); % Y轴
quiver3(ref(1), ref(2), ref(3), 0, 0, scale, 'b', 'LineWidth', 2, 'MaxHeadSize', 0.5); % Z轴
hold off;
xlabel('X'); ylabel('Y'); zlabel('Z');
title('平滑后的 3D 笛卡尔轨迹');
grid on;
axis equal;

% 假设工具轨迹（figure5 数据），前3列为 X, Y, Z，后3列为 \theta_4, \theta_5, \theta_6
tool_trajectory = smoothed_pos;  % 假设平滑后的数据直接用作工具轨迹

% 时间向量（假设时间已定义）
timeVec = t;

%% Figure 7：利用逆运动学求解每个时刻的关节角
numPoints = size(tool_trajectory, 1);  % 数据点数
joint_angles = zeros(numPoints, 6);    % 初始化关节角轨迹

for idx = 1:numPoints
    % 提取当前时刻的工具位姿平移部分 [X, Y, Z]
    p_current = tool_trajectory(idx, 1:3)';
    
    % 构造工具末端齐次变换矩阵 T0_6
    % 工具姿态部分暂假设为单位矩阵（无旋转），可根据具体姿态信息修改
    T0_6 = [eye(3), p_current; 0 0 0 1];
    
    % 调用逆运动学函数 IK_algorithm（用户需要自定义此函数）
    execute_solution = IK_algorithm(T0_6, dh);
    
    % 将逆解的前三个关节角与后三个工具角（\theta_4, \theta_5, \theta_6）拼接
    joint_angles(idx, :) = [execute_solution(1:3), tool_trajectory(idx, 4:6)];
end

% 绘制关节角轨迹（Figure 7）
figure(7);
joint_labels = {'\theta_1', '\theta_2', '\theta_3', '\theta_4', '\theta_5', '\theta_6'};
for i = 1:6
    subplot(2, 3, i);
    plot(timeVec, joint_angles(:, i), 'LineWidth', 2);
    xlabel('时间 (秒)');
    ylabel(joint_labels{i});
    title([joint_labels{i} ' 随时间变化']);
    grid on;
end

%% Figure 8：加入动态点沿轨迹运动，反映速度和加速度

% 绘制静态轨迹作为参考
figure(8);
plot3(cartesian_traj_fk(:,1), cartesian_traj_fk(:,2), cartesian_traj_fk(:,3), 'k--', 'LineWidth', 1); % 静态轨迹
hold on;
xlabel('X'); ylabel('Y'); zlabel('Z');
title('动态点沿轨迹运动（反映速度和加速度）');
grid on;
axis equal;

% 动态绘制点
h_point = plot3(cartesian_traj_fk(1,1), cartesian_traj_fk(1,2), cartesian_traj_fk(1,3), 'ro', 'MarkerSize', 10, 'LineWidth', 2); % 点对象

% 绘制起点和终点
plot3(cartesian_traj_fk(1,1), cartesian_traj_fk(1,2), cartesian_traj_fk(1,3), 'ro', 'MarkerSize', 8, 'LineWidth', 2);
plot3(cartesian_traj_fk(end,1), cartesian_traj_fk(end,2), cartesian_traj_fk(end,3), 'go', 'MarkerSize', 8, 'LineWidth', 2);

% 动态点沿轨迹移动
for idx = 1:length(cartesian_traj_fk)
    % 更新点的位置
    set(h_point, 'XData', cartesian_traj_fk(idx,1), ...
                 'YData', cartesian_traj_fk(idx,2), ...
                 'ZData', cartesian_traj_fk(idx,3));

    % 暂停以实现动态效果
    pause(0.05);
end

hold off;





%% 平滑曲线函数
function [pos, vel, acc, timeVec] = linearBlendTrajectory(p0, pf, T, Tb, numPoints)
    % 生成从 p0 到 pf 平滑轨迹的离散数据
    timeVec = linspace(0, T, numPoints)';  % 时间向量
    pos = zeros(numPoints, 1);
    vel = zeros(numPoints, 1);
    acc = zeros(numPoints, 1);

    % 总位移
    D = pf - p0;

    % 分段规划：
    %  加速段（0 ~ Tb）：位移 = 0.5*a*Tb^2
    %  匀速段（Tb ~ T-Tb）：位移 = V*(T-2*Tb)，其中 V = a*Tb
    %  减速段（T-Tb ~ T）：位移 = 0.5*a*Tb^2
    % 总位移 D = a * Tb * (T - Tb)  ==>  a = D / (Tb*(T-Tb))
    a = D / (Tb * (T - Tb));
    V = a * Tb;

    for i = 1:length(timeVec)
        t = timeVec(i);
        if t < Tb
            % 加速段
            pos(i) = p0 + 0.5 * a * t^2;
            vel(i) = a * t;
            acc(i) = a;
        elseif t <= (T - Tb)
            % 匀速段
            pos(i) = p0 + 0.5 * a * Tb^2 + V * (t - Tb);
            vel(i) = V;
            acc(i) = 0;
        else
            % 减速段
            t_dec = T - t;
            pos(i) = pf - 0.5 * a * t_dec^2;
            vel(i) = a * t_dec;
            acc(i) = -a;
        end
    end
end




%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%以下是正逆解


function execute_solution = IK_algorithm(T0_6, dh)
    % 计算四组θ₁, θ₂, θ₃的解
    [sol123_1, sol123_2, sol123_3, sol123_4] = solve_theta123(T0_6, dh);
    
    % 计算每组θ₁, θ₂, θ₃对应的两组手腕角度（θ₄, θ₅, θ₆）
    [sol1_1, sol1_2] = calc_wrist_angles(sol123_1(1), sol123_1(2), sol123_1(3), T0_6, dh);
    [sol2_1, sol2_2] = calc_wrist_angles(sol123_2(1), sol123_2(2), sol123_2(3), T0_6, dh);
    [sol3_1, sol3_2] = calc_wrist_angles(sol123_3(1), sol123_3(2), sol123_3(3), T0_6, dh);
    [sol4_1, sol4_2] = calc_wrist_angles(sol123_4(1), sol123_4(2), sol123_4(3), T0_6, dh);
    
    % 整合八组解到八个6x1矩阵
    solutions = zeros(6, 8);
    solutions(:, 1) = [sol123_1; sol1_1];
    solutions(:, 2) = [sol123_1; sol1_2];
    solutions(:, 3) = [sol123_2; sol2_1];
    solutions(:, 4) = [sol123_2; sol2_2];
    solutions(:, 5) = [sol123_3; sol3_1];
    solutions(:, 6) = [sol123_3; sol3_2];
    solutions(:, 7) = [sol123_4; sol4_1];
    solutions(:, 8) = [sol123_4; sol4_2];
    
    % 默认返回第一组解（可根据需求修改选择逻辑）
    execute_solution = solutions(:, 1)';

end



function [sol1, sol2, sol3, sol4] = solve_theta123(T0_6, dh)
% solve_theta123 求解机器人前三个关节角的四组解
%
% 输入:
%   T0_6 - 4x4末端位姿矩阵
%   dh   - 结构体，包含以下字段:
%          a1, a2, a3, d1, d2, d3, d4
%
% 输出:
%   sol1, sol2, sol3, sol4 - 分别为 [θ1; θ2; θ3] 的四组解
%
% 例如，调用方法:
%   dh.a1 = 0.04;  dh.a2 = 0.18;   dh.a3 = 0.055;
%   dh.d1 = 0.212; dh.d2 = 0.062;   dh.d3 = -0.05;  dh.d4 = 0.205027;
%   T0_6 = [ 0.583359, -0.492574,  0.645804, 0.046752;
%            0.346615, -0.568099, -0.746406, -0.009192;
%            0.734540,  0.659268, -0.160673, 0.568912;
%            0,         0,         0,         1];
%   [sol1,sol2,sol3,sol4] = solve_theta123(T0_6, dh);
%
% 算法基于已知末端位置以及 DH 参数，通过几何法求解前三轴的逆解，
% 可得肘上/肘下两大构型，每一构型又有左右解。

    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %% 提取末端位姿的平移部分（即4坐标系原点在基坐标系下的位置）
    P0_4 = [T0_6(1,4); T0_6(2,4); T0_6(3,4); 1];
    x = P0_4(1);
    y = P0_4(2);
    z = P0_4(3);
    
    % 计算 r = x^2 + y^2 + z^2
    r = x^2 + y^2 + z^2;
    
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %% 利用模型参数计算中间变量
    % 虚拟连杆长度 L（结合 a3 与 d4 的影响）
    L = sqrt(dh.a3^2 + dh.d4^2);
    
    % 计算 R_x = sqrt(r - (d2+d3)^2 - z^2)
    R_x = sqrt(r - (dh.d2 + dh.d3)^2 - z^2);
    
    % 定义水平平面上的变量 X 和竖直方向上的变量 Z，
    % 用来消除 a1 与 d1 对位置的影响
    X = R_x - dh.a1;
    Z = z - dh.d1;
    
    % 辅助变量 D 利用余弦定律计算（D = cos(角D)）
    D = (dh.a2^2 + L^2 - (Z^2 + X^2)) / (2 * dh.a2 * L);
    
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %% 第一组解（肘上构型1）
    theta2_sol_1 = pi/2 - atan2(Z, X) - atan2( L * sqrt(1 - D^2), dh.a2 - L * D) ;
    theta3_sol_1 = -( acos(D) - atan2(dh.a3, dh.d4) - pi/2 ) ;
    theta1_sol_1 = atan2(y, x) - asin((dh.d2 + dh.d3) / sqrt(x^2 + y^2)) ;
    
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %% 第二组解（肘下构型1）
    theta2_sol_2 = atan2( L * sqrt(1 - D^2), dh.a2 - L * D) + atan2(X, Z);
    theta3_sol_2 = -( pi - ( acos(D) - atan2(dh.d4, dh.a3) ) );
    theta1_sol_2 = theta1_sol_1;
    
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %% 为计算第三、第四组解构造新的中间量：
    % 对 X 加上 2*a1，得到 X_（消除额外的 a1 偏移）
    X_ = X + 2 * dh.a1;
    D_ = (dh.a2^2 + L^2 - (Z^2 + X_^2)) / (2 * dh.a2 * L);
    
    % 第三组解（肘上构型2）
    theta1_sol_3 = theta1_sol_1 - 2 * acos((dh.d2 + dh.d3) / sqrt(x^2 + y^2));
    theta2_sol_3 = -( atan2(X_, Z) - atan2( L * sqrt(1 - D_^2), dh.a2 - L * D_) ) ;
    theta3_sol_3 = -( pi - ( acos(D_) - atan2(dh.d4, dh.a3) ) ) ;
    
    % 第四组解（肘下构型2）
    theta1_sol_4 = theta1_sol_3;
    theta2_sol_4 = -( atan2(X_, Z) + atan2( L * sqrt(1 - D_^2), dh.a2 - L * D_) ) ;
    theta3_sol_4 = pi - acos(D_) - atan2(dh.d4, dh.a3) ;
    
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % fprintf('\n【第一组姿态】\n  theta1 = %.6f rad\n  theta2 = %.6f rad\n  theta3 = %.6f rad\n',...
    % theta1_sol_1, theta2_sol_1, theta3_sol_1);
    % fprintf('\n【第二组姿态】\n  theta1 = %.6f rad\n  theta2 = %.6f rad\n  theta3 = %.6f rad\n',...
    %     theta1_sol_2, theta2_sol_2, theta3_sol_2);
    % fprintf('\n【第三组姿态】\n  theta1 = %.6f rad\n  theta2 = %.6f rad\n  theta3 = %.6f rad\n',...
    %     theta1_sol_3, theta2_sol_3, theta3_sol_3 );
    % fprintf('\n【第四组姿态】\n  theta1 = %.6f rad\n  theta2 = %.6f rad\n  theta3 = %.6f rad\n',...
    %     theta1_sol_4, theta2_sol_4, theta3_sol_4);

    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %% 输出四组解，每组解为列向量 [θ1; θ2; θ3]
    sol1 = [theta1_sol_1; theta2_sol_1; theta3_sol_1];
    sol2 = [theta1_sol_2; theta2_sol_2; theta3_sol_2];
    sol3 = [theta1_sol_3; theta2_sol_3; theta3_sol_3];
    sol4 = [theta1_sol_4; theta2_sol_4; theta3_sol_4];
end


function [sol1, sol2] = calc_wrist_angles(theta1, theta2, theta3, T0_6, dh)
% calc_wrist_angles 计算机器人后三轴（腕部）的两个解
%
% 输入:
%   theta1, theta2, theta3 - 前三轴关节角（单位：弧度）
%   dh - 结构体，包含以下字段:
%          dh.alpha0, dh.alpha1, dh.alpha2, dh.alpha3,
%          dh.a0, dh.a1, dh.a2, dh.a3,
%          dh.d1, dh.d2, dh.d3, dh.d4
%   T0_6 - 4x4末端位姿齐次变换矩阵
%
% 输出:
%   sol1 - 第一组腕部关节角 [theta4; theta5; theta6]
%   sol2 - 第二组腕部关节角（腕翻解） [theta4; theta5; theta6]

    %% 计算前三个关节的正运动学变换，直接使用 dh 内部的值
    T0_1 = transformation_matrix(theta1,        dh.alpha0, dh.a0, dh.d1);
    T1_2 = transformation_matrix(theta2 - pi/2,   dh.alpha1, dh.a1, dh.d2);
    T2_3 = transformation_matrix(theta3,        dh.alpha2, dh.a2, dh.d3);
    T0_3 = T0_1 * T1_2 * T2_3;
    R0_3 = T0_3(1:3, 1:3);
    
    %% 计算腕部坐标系与欧拉角坐标系之间的变换
    T3_4 = transformation_matrix(0, dh.alpha3, dh.a3, dh.d4);
    R0_E = R0_3 * T3_4(1:3, 1:3);
    
    %% 从 T0_6 中提取末端旋转矩阵，并计算腕部部分的相对旋转矩阵
    R0_6 = T0_6(1:3, 1:3);
    R_end = transpose(R0_E) * R0_6;
    
    %% 利用标准 Z–Y–Z 分解提取腕部角度（非奇异情况）
    sol1_alpha = atan2( R_end(2,3), R_end(1,3) );
    sol1_gamma = atan2( R_end(3,2), -R_end(3,1) );
    sol1_beta = atan2( R_end(3,2)/sin(sol1_gamma), R_end(3,3)) - pi/2;
    
    
    %% 第一组解
    sol1_theta4 = sol1_alpha;
    sol1_theta5 = -sol1_beta + pi;
    sol1_theta6 = sol1_gamma;
    sol1 = [sol1_theta4; sol1_theta5; sol1_theta6];
    
    %% 第二组解
    sol2_theta4 = sol1_theta4 + pi;
    sol2_theta5 = sol1_beta;
    sol2_theta6 = sol1_theta6 + pi;
    sol2 = [sol2_theta4; sol2_theta5; sol2_theta6];

    % %显示 
    % fprintf('--- Modified DH 模型下腕部反解的两组解 ---\n');
    % fprintf('【解 1】\n');
    % fprintf('theta4 = %.6f rad\n', sol1(1));
    % fprintf('theta5 = %.6f rad\n', sol1(2));
    % fprintf('theta6 = %.6f rad\n\n', sol1(3));
    % 
    % fprintf('【解 2】\n');
    % fprintf('theta4 = %.6f rad\n', sol2(1));
    % fprintf('theta5 = %.6f rad\n', sol2(2));
    % fprintf('theta6 = %.6f rad\n\n', sol2(3));
end

function T = transformation_matrix(theta_i, alphai_1_prev, ai_1_prev, d_i)
    % 修正后的变换矩阵
    T = [
        cos(theta_i), -sin(theta_i), 0, ai_1_prev;
        sin(theta_i)*cos(alphai_1_prev), cos(theta_i)*cos(alphai_1_prev), -sin(alphai_1_prev), -sin(alphai_1_prev)*d_i;
        sin(theta_i)*sin(alphai_1_prev), cos(theta_i)*sin(alphai_1_prev), cos(alphai_1_prev), cos(alphai_1_prev)*d_i;
        0, 0, 0, 1
    ];
end

function R = forward_kinematics(theta1, theta2, theta3, theta4, theta5, theta6, dh)
    % forward_kinematics - 计算机器人末端的4×4变换矩阵
    % 
    % 输入:
    %   theta1, theta2, theta3, theta4, theta5, theta6 - 六个关节角（单位：弧度）
    %   dh - 结构体，包含机器人的 DH 参数（例如 dh.a1, dh.d1 等）
    %
    % 输出:
    %   R - 4×4 齐次变换矩阵，描述从基座到末端的位姿

    % 提取 DH 参数
    a1 = dh.a1; a2 = dh.a2; a3 = dh.a3;
    d1 = dh.d1; d2 = dh.d2; d3 = dh.d3; d4 = dh.d4;

    % 计算齐次变换矩阵的各个元素
    r11 = sin(theta6)*(cos(theta4)*sin(theta1) - sin(theta4)*(cos(theta1)*cos(theta2)*sin(theta3) + cos(theta1)*cos(theta3)*sin(theta2))) ...
          - cos(theta6)*(sin(theta5)*(sin(theta1)*sin(theta4) + cos(theta4)*(cos(theta1)*cos(theta2)*sin(theta3) + cos(theta1)*cos(theta3)*sin(theta2))) ...
          + cos(theta5)*(cos(theta1)*sin(theta2)*sin(theta3) - cos(theta1)*cos(theta2)*cos(theta3)));

    r12 = cos(theta6)*(cos(theta4)*sin(theta1) - sin(theta4)*(cos(theta1)*cos(theta2)*sin(theta3) + cos(theta1)*cos(theta3)*sin(theta2))) ...
          + sin(theta6)*(sin(theta5)*(sin(theta1)*sin(theta4) + cos(theta4)*(cos(theta1)*cos(theta2)*sin(theta3) + cos(theta1)*cos(theta3)*sin(theta2))) ...
          + cos(theta5)*(cos(theta1)*sin(theta2)*sin(theta3) - cos(theta1)*cos(theta2)*cos(theta3)));

    r13 = sin(theta5)*(cos(theta1)*sin(theta2)*sin(theta3) - cos(theta1)*cos(theta2)*cos(theta3)) ...
          - cos(theta5)*(sin(theta1)*sin(theta4) + cos(theta4)*(cos(theta1)*cos(theta2)*sin(theta3) + cos(theta1)*cos(theta3)*sin(theta2)));

    r14 = a3*(cos(theta1)*cos(theta2)*sin(theta3) + cos(theta1)*cos(theta3)*sin(theta2)) ...
          - d4*(cos(theta1)*sin(theta2)*sin(theta3) - cos(theta1)*cos(theta2)*cos(theta3)) ...
          + a1*cos(theta1) - d2*sin(theta1) - d3*sin(theta1) + a2*cos(theta1)*sin(theta2);

    r21 = cos(theta6)*(sin(theta5)*(cos(theta1)*sin(theta4) - cos(theta4)*(cos(theta2)*sin(theta1)*sin(theta3) + cos(theta3)*sin(theta1)*sin(theta2))) ...
          - cos(theta5)*(sin(theta1)*sin(theta2)*sin(theta3) - cos(theta2)*cos(theta3)*sin(theta1))) ...
          - sin(theta6)*(cos(theta1)*cos(theta4) + sin(theta4)*(cos(theta2)*sin(theta1)*sin(theta3) + cos(theta3)*sin(theta1)*sin(theta2)));

    r22 = -cos(theta6)*(cos(theta1)*cos(theta4) + sin(theta4)*(cos(theta2)*sin(theta1)*sin(theta3) + cos(theta3)*sin(theta1)*sin(theta2))) ...
          - sin(theta6)*(sin(theta5)*(cos(theta1)*sin(theta4) - cos(theta4)*(cos(theta2)*sin(theta1)*sin(theta3) + cos(theta3)*sin(theta1)*sin(theta2))) ...
          - cos(theta5)*(sin(theta1)*sin(theta2)*sin(theta3) - cos(theta2)*cos(theta3)*sin(theta1)));

    r23 = cos(theta5)*(cos(theta1)*sin(theta4) - cos(theta4)*(cos(theta2)*sin(theta1)*sin(theta3) + cos(theta3)*sin(theta1)*sin(theta2))) ...
          + sin(theta5)*(sin(theta1)*sin(theta2)*sin(theta3) - cos(theta2)*cos(theta3)*sin(theta1));

    r24 = a3*(cos(theta2)*sin(theta1)*sin(theta3) + cos(theta3)*sin(theta1)*sin(theta2)) ...
          - d4*(sin(theta1)*sin(theta2)*sin(theta3) - cos(theta2)*cos(theta3)*sin(theta1)) ...
          + d2*cos(theta1) + d3*cos(theta1) + a1*sin(theta1) + a2*sin(theta1)*sin(theta2);

    r31 = -cos(theta6)*(sin(theta2+theta3)*cos(theta5) + cos(theta2+theta3)*cos(theta4)*sin(theta5)) - cos(theta2+theta3)*sin(theta4)*sin(theta6);

    r32 = sin(theta6)*(sin(theta2+theta3)*cos(theta5) + cos(theta2+theta3)*cos(theta4)*sin(theta5)) - cos(theta2+theta3)*cos(theta6)*sin(theta4);

    r33 = sin(theta2+theta3)*sin(theta5) - cos(theta2+theta3)*cos(theta4)*cos(theta5);

    r34 = d1 + a3*cos(theta2+theta3) - d4*sin(theta2+theta3) + a2*cos(theta2);

    r41 = 0;
    r42 = 0;
    r43 = 0;
    r44 = 1;

    % 合并成 4×4 齐次变换矩阵
    R = [r11 r12 r13 r14;
         r21 r22 r23 r24;
         r31 r32 r33 r34;
         r41 r42 r43 r44];
end




 







