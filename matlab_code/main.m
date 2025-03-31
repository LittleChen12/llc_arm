clear;clc;close all

addpath(genpath('FK'),genpath('IK'),genpath('TRAJECT'),genpath('demo'),genpath('robot'))%添加文件的路径

% 已知工具坐标系的初始位置
T0_C_init = [1.0000,         0,         0,    0.2450;
                  0,    -1.0000,         0,    0.0120;
                  0,         0,    -1.0000,    0.4470;
                  0,         0,         0,    1.0000];
       
% 已知工具坐标系的最终位置
T0_C_end = [ -0.0154,    0.0764,   -0.9970,    0.2391;
			-0.2596,   -0.9632,   -0.0698,    0.1487;
			-0.9656,    0.2578,    0.0347,    0.2212;
						0,         0,         0,    1.0000];
        
% 已知工具坐标系相对于6坐标系的变换矩阵
T6_C = [1, 0, 0, 0;
        0, 1, 0, 0;
        0, 0, 1, 0.0865;
        0, 0, 0, 1];

demo_ik_fk=0; %正逆解测试
demo_show_forward=0; %正解显示
demo_show_inverse=0; %位置逆解显示
demo_trajectory_plan=1;   %自己写的轨迹规划代码

if demo_ik_fk+demo_show_forward+demo_show_inverse+demo_trajectory_plan>1
    error('run one function one time 一次运行一个选项')
end

%% 测试inverse kinematics 和 forward kinematics
if demo_ik_fk
    %测试正逆解的代码是否正确
    %输入一个目标角度先用正解找到末端执行器的矩阵后,逆解出8组解，在将八组解正解看是否相同，
    %调用的方法位于demo文件夹
    
end
 
if demo_show_forward
    %直观的查看正解
    %输入关节显示正解的矩阵并显示robot的状态
    %调用的方法位于demo文件夹
    joints = [ -3.0437   -0.5073   -1.9977    3.1416   -2.5049    3.0437];
    show_forward(joints);
end

if demo_show_inverse
    %直观的查看逆解
    %输入末端执行器的矩阵并显示robot的状态
    %调用的方法位于demo文件夹
    T_IK_TEST = [
    1.0000  ,       0   ,      0   ,    0.2450;
         0  ,  -1.0000   ,     0    ,    0.0120;
         0  ,       0   ,   -1.0000 ,    0.4470;
         0  ,       0   ,      0   ,    1.0000;        
    
    ];
    show_ik(T_IK_TEST);

end

if demo_trajectory_plan
    %直观的查看轨迹规划
    show_traject(T0_C_init,T0_C_end,10,500);
    
end