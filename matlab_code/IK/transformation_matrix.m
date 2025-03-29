function T = transformation_matrix(theta_i, alphai_1_prev, ai_1_prev, d_i)
    % 修正后的变换矩阵
    T = [
        cos(theta_i), -sin(theta_i), 0, ai_1_prev;
        sin(theta_i)*cos(alphai_1_prev), cos(theta_i)*cos(alphai_1_prev), -sin(alphai_1_prev), -sin(alphai_1_prev)*d_i;
        sin(theta_i)*sin(alphai_1_prev), cos(theta_i)*sin(alphai_1_prev), cos(alphai_1_prev), cos(alphai_1_prev)*d_i;
        0, 0, 0, 1
    ];
end