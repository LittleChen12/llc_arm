clear; clc;
%直线规划1
% 定义符号变量
% syms a0 a1 a2 a3 a4 v_mid M T tacc p0 pf;
% M = (T-tacc);
% 
% eq1 = a1 + 2*a2*M + 3*a3*M^2 + 4*a4*M^3 == v_mid;
% eq2 = a1 + 2*a2*T + 3*a3*T^2 +4*a4*T^3 == 0;
% eq3 = 2*a2 + 6*a3*M + 12*a4*M^2 == 0;
% eq4 = 2*a2 + 6*a3*T + 12*a4*T^2 == 0;
% eq5 = a0 + a1*M + a2*M^2 + a3*M^3 + a4*M^4 == p0 + v_mid*M;
% 
% % 用 solve 函数求解
% sol = solve([eq1, eq2, eq3, eq4, eq5], [a0, a1, a2, a3, a4]);
% 
% disp('解得未知变量：');
% disp(sol);

%直线规划2
syms  a tb p0 pf tf;

eq1 = a*tb^2 - a*tf*tb +(pf - p0) == 0;

% 用 solve 函数求解
sol = solve(eq1, tb);
disp('解得未知变量：');
disp(sol);

