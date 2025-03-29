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
