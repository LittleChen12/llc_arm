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
        

T0_C_init = [1.0000,         0,         0,    0.2450;
             0,   -1.0000,         0,    0.0120;
             0,         0,   -1.0000,    0.4470;
             0,         0,         0,    1.0000];
       
% 已知工具坐标系的最终位置
T0_C_end = [  -0.0154,    0.0764,   -0.9970,    0.2391;
              -0.2596,   -0.9632,   -0.0698,    0.1487;
              -0.9656,    0.2578,    0.0347,    0.2212;
                    0,         0,         0,    1.0000;];

theta1 = IK_algorithm(T0_C_init, dh);
disp(theta1);
theta2 = IK_algorithm(T0_C_end, dh);
disp(theta2);



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
    execute_solution = solutions;

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




 







