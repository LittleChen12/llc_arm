#ifndef __IK_H
#define __IK_H


//头文件包含
#include "arm_math.h"
#include "math.h"
#include "string.h"
#include "stdio.h"
#include "controler.h"

//自定义函数
void IK_matrix_init(void);

//给theta123456[8][6]数组赋值，整理八组解
void Assign_theta123456(void);

void IK(float T0_6_matrix[16],float dh_matrix[6][4],float IK_theta[6]);

/* 
 * 函数：solve_theta123
 * ---------------------
 * 根据末端位姿 T0_6 以及 DH 参数表（dh_matrix, 6×4）求解机器人前三轴角的四组解。
 * 输入：
 *    T0_6         - 长度为 16 的一维数组，保存 4×4 末端位姿矩阵（行主序）
 *    dh_matrix    - 6×4 的二维 float 数组，保存 DH 参数 [α, a, d, θ_offset]。
 *                   其中：
 *                     a1  = dh_matrix[1][1],
 *                     a2  = dh_matrix[2][1],
 *                     a3  = dh_matrix[3][1],
 *                     d1  = dh_matrix[0][2],
 *                     d2  = dh_matrix[1][2],
 *                     d3  = dh_matrix[2][2],
 *                     d4  = dh_matrix[3][2].
 * 输出：
 *    sol1, sol2, sol3, sol4 - 四组解
 */
void solve_theta123(float T0_6[16], float dh_matrix[6][4],
                    float sol1[3], float sol2[3],
                    float sol3[3], float sol4[3]);

/*
 * 函数: calc_wrist_angles
 * -------------------------
 * 计算机器人后三轴（腕部）的两个解
 *
 * 输入:
 *    theta1, theta2, theta3 - 前三轴关节角（单位：弧度）
 *    T0_6 - 4×4 末端位姿矩阵（长度 16 的数组，行主序存储）
 *    dh_matrix - 6×4 的 DH 参数表数组，每行顺序为 {alpha, a, d, theta_offset}
 *                本函数只使用前 4 行对应的：Joint 1 ~ Joint 4 参数。
 *
 * 输出:
 *    sol1 - 第一组腕部关节角 [theta4; theta5; theta6]
 *    sol2 - 第二组腕部（腕翻）关节角 [theta4; theta5; theta6]
 */
void calc_wrist_angles(float theta[6],
                       float T0_6[16], float dh_matrix[6][4],
                       float sol1[3], float sol2[3]);


//自定义变量
extern float T0_6[16];
extern float dh_matrix_IK[6][4];




#endif
