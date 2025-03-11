#ifndef __CONTROLER_H
#define __CONTROLER_H

//头文件包含
#include "arm_math.h"
#include "math.h"
#include "string.h"
#include "stdio.h"


//自定义函数
// 写入六个电机角度到改进型 DH 参数表
void Write_DH(float (*modified_dh_matrix)[6][4], float* theta);

// 计算改进型 DH 参数对应的变换矩阵
// 参数 modified_dh_matrix 的每一行存放： θ, d, a, α
void compute_modified_dh_matrix(float modified_dh_matrix[6][4], int i);

// 正向运动学
void forward_kinematics(float dh_matrix[6][4], int num_joints);

void dh_test(void);

#endif
