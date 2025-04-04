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
    % % 打印所有8组解
    % disp('All solutions:');
    % disp(solutions');

    % % 设置每个角度的有效范围（请根据实际情况修改数值）
    % angle_ranges = [
    %     min1, max1;
    %     min2, max2;
    %     min3, max3;
    %     min4, max4;
    %     min5, max5;
    %     min6, max6;
    % ];
    % 
    % % 初始化逻辑数组，标记每组解是否有效（1×8）
    % is_valid = true(1, size(solutions, 2));
    % 
    % % 检查每个角度是否满足范围要求
    % for i = 1:size(solutions, 1)
    %     is_valid = is_valid & (solutions(i, :) >= angle_ranges(i, 1)) & (solutions(i, :) <= angle_ranges(i, 2));
    % end
    % 
    % execute_solution = solutions(:, is_valid);
    execute_solution = solutions(:,1);
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

    % % 第三组解（肘上构型2）
    % theta1_sol_3 = theta1_sol_1 - 2 * acos((dh.d2 + dh.d3) / sqrt(x^2 + y^2));
    % theta2_sol_3 = -( atan2(X_, Z) - atan2( L * sqrt(1 - D_^2), dh.a2 - L * D_) ) ;
    % theta3_sol_3 = -( pi - ( acos(D_) - atan2(dh.d4, dh.a3) ) ) ;
    % 
    % % 第四组解（肘下构型2）
    % theta1_sol_4 = theta1_sol_3;
    % theta2_sol_4 = -( atan2(X_, Z) + atan2( L * sqrt(1 - D_^2), dh.a2 - L * D_) ) ;
    % theta3_sol_4 = pi - acos(D_) - atan2(dh.d4, dh.a3) ;
    
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
    sol3 = [0; 0; 0];
    sol4 = [0; 0; 0];
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
    T3_4 = transformation_matrix(0, dh.alpha3, dh.a3, dh.d4);%这儿可以简化
    R0_E = R0_3 * T3_4(1:3, 1:3);
    
    %% 从 T0_6 中提取末端旋转矩阵，并计算腕部部分的相对旋转矩阵
    R0_6 = T0_6(1:3, 1:3);
    R_end = transpose(R0_E) * R0_6;%此处得到的是6号坐标系在4号坐标系下的表示
    
    %% 利用标准 Z–Y–Z 分解提取腕部角度
% R_total = 
% [cos(alpha)*cos(beta)*cos(gamma) - sin(alpha)*sin(gamma), - cos(gamma)*sin(alpha) - cos(alpha)*cos(beta)*sin(gamma), cos(alpha)*sin(beta)]
% [cos(alpha)*sin(gamma) + cos(beta)*cos(gamma)*sin(alpha),   cos(alpha)*cos(gamma) - cos(beta)*sin(alpha)*sin(gamma), sin(alpha)*sin(beta)]
% [                                -1*cos(gamma)*sin(beta),                                      sin(beta)*sin(gamma),            cos(beta)]

    sol1_beta = atan2( sqrt(R_end(3,1)^2 + R_end(3,2)^2) , R_end(3,3));%由于第一部分非负，所以角度范围为0到pi，0和pi均可取，
    tol = 1e-6;%由于0和pi都是奇异情况，真实情况下，很难说绝对的0和pi，计算都有误差，所以在这里取了一个区间。
    if abs(sol1_beta) > tol && abs(sol1_beta - pi) > tol
        sol1_alpha = atan2( R_end(2,3)/sin(sol1_beta), R_end(1,3)/sin(sol1_beta));%除以sin(sol1_beta)是为了消除sin(sol1_beta)的影响！！
        sol1_gamma = atan2( R_end(3,2)/sin(sol1_beta), -R_end(3,1)/sin(sol1_beta) );%除以sin(sol1_beta)是为了消除sin(sol1_beta)的影响
    elseif abs(sol1_beta) <= tol   % 当 beta 充分接近 0 时 奇异情况（但是是针对特殊的初始位置），出现奇异解的时候，直接让sol1_alpha为0，求解sol1_gamma。（本质上也可以让sol1_gamma为0，求解sol1_alpha）
        sol1_alpha = 0;
        sol1_gamma = atan2(-R_end(1,2), R_end(1,1));
    elseif abs(sol1_beta - pi) <= tol  % 当 beta 充分接近 pi 时 奇异情况（但是是针对特殊的初始位置）
        sol1_alpha = 0;
        sol1_gamma = atan2(R_end(1,2), -R_end(1,1));        
    end

    % sol1_theta4 = sol1_alpha - pi;%我们知道，由于ZYZ的解法 sol1_theta4 = sol1_alpha - pi；所以实际上4角的范围是-2pi到0
    % sol1_theta5 = sol1_beta;%我们知道，由于ZYZ的解法 sol1_theta5 = sol1_beta；所以实际上5角就是sol1_beta
    % sol1_theta6 = sol1_gamma - pi;%我们知道，由于ZYZ的解法 sol1_theta6 = sol1_gamma - pi；所以实际上6角的范围是-2pi到0
        % 修改后：将角度包装到 [-pi, pi]
        % 使用单片机实现的时候用fmodf即可
        % float wrapToPi_f32(float angle) {
        %     float two_pi = 2.0f * PI;  // 注意确保 PI 已正确定义
        %     // 先将角度加上 PI，然后使用 fmodf 取模
        %     angle = fmodf(angle + PI, two_pi);
        %     if (angle < 0)
        %         angle += two_pi;//这儿是因为需要取正模，matlab中的mod已经有取正模，但是单片机中没有
        %     return angle - PI;
        % }
    sol1_theta4 = mod(sol1_alpha - pi + pi, 2*pi) - pi;%注意：sol1_alpha - pi是原先的sol1_theta4，平移pi后
    sol1_theta5 = sol1_beta;  % 已经是[0,pi]
    sol1_theta6 = mod(sol1_gamma - pi + pi, 2*pi) - pi;

    sol1 = [sol1_theta4; sol1_theta5; sol1_theta6];
    
    %% 第二组解
    sol2_theta4 = sol1_alpha;%由于第二组解，本来就是sol1_theta4加pi，所以取值空间为-pi到pi
    sol2_theta5 = -sol1_beta;%第二组解的取值空间在-pi到0
    sol2_theta6 = sol1_gamma;%由于第二组解，本来就是sol1_theta6加pi，所以取值空间为-pi到pi
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