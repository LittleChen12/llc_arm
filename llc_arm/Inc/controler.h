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
void compute_modified_dh_matrix(float modified_dh_matrix[6][4], int i,float real_motor_theta[6],float t_trans_matrix[16]);
//正向运动学
void forward_kinematics(float modified_dh_matrix[6][4], int num_joints,float motor_theta[6],float t_result_matrix[16],arm_matrix_instance_f32 *T_result_matrix); 
// 正向运动学方案二
void forward_kinematics2(float dh_matrix[6][4]);
//测试正解
void dh_test(void);
//矩阵初始化
void matrix_init(void);

//自定义变量
extern float dh_matrix[6][4];
//电机的角度值
extern float real_motor_theta[6];
extern float end_trans[16];

#endif
