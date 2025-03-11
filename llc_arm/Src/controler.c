#include "controler.h"

//自定义变量
//DH表
float dh_matrix[6][4] = {
		{0.0f,     0.0f,   0.212f,    0.0f},       // Joint 1
		{-PI/2.0f, 0.04f,  0.062f,    -PI/2.0f},   // Joint 2
		{0.0f,     0.18f,  -0.05f,    0.0f},       // Joint 3
		{-PI/2.0f, 0.055f, 0.205027f, 0.0f},       // Joint 4
		{PI/2.0f,  0.0f,   0.0f,      PI/2.0f},    // Joint 5
		{-PI/2.0f, 0.0f,   0.0f,   		0.0f}        // Joint 6
};

//变化矩阵
arm_matrix_instance_f32 T_trans;
float t_trans[16];

//矩阵累乘的临时矩阵									 
arm_matrix_instance_f32	 T_temp;
float t_temp[16] = {0};

//最后的输出矩阵
arm_matrix_instance_f32 T_result;
float t_result[16] = {1.0f, 0.0f, 0.0f, 0.0f,
												0.0f, 1.0f, 0.0f, 0.0f,
												0.0f, 0.0f, 1.0f, 0.0f,
												0.0f, 0.0f, 0.0f, 1.0f};

//末端相对于6轴的位姿矩阵
arm_matrix_instance_f32 T_xyz_6;	
float t_xyz_6[16] = {1.0f, 0.0f, 0.0f, 0.0f,
									 0.0f, 1.0f, 0.0f, 0.0f,
									 0.0f, 0.0f, 1.0f, 0.0865f,
									 0.0f, 0.0f, 0.0f, 1.0f};											

//末端相对于0轴的位置矩阵
arm_matrix_instance_f32 T_xyz_0;	
float t_xyz_0[16] = {1.0f, 0.0f, 0.0f, 0.0f,
									 0.0f, 1.0f, 0.0f, 0.0f,
									 0.0f, 0.0f, 1.0f, 0.0f,
									 0.0f, 0.0f, 0.0f, 1.0f};			

//电机角度
float motor_theta[6]={0.513709f,0.514567f,0.515424f,0.516280f,0.517137f,0.517992f};//电机减速比50,50,50,4,4,30


void matrix_init(void)
{
	arm_mat_init_f32(&T_trans, 4, 4, t_trans);   // 初始化变化矩阵	
	arm_mat_init_f32(&T_result, 4, 4, t_result); // 初始化输出矩阵
	arm_mat_init_f32(&T_xyz_6, 4, 4, t_xyz_6);   // 初始化末端姿态矩阵
	arm_mat_init_f32(&T_xyz_0, 4, 4, t_xyz_0);   // 初始化末端姿态矩阵
	arm_mat_init_f32(&T_temp, 4, 4, t_temp);     //初始化临时计算矩阵
}

//测试正解
void dh_test(void)
{
	matrix_init();
	forward_kinematics(dh_matrix,6);
}

// 计算改进型 DH 参数对应的变换矩阵
// 参数 modified_dh_matrix 的每一行存放：α, a, d,θ(正常)
void compute_modified_dh_matrix(float modified_dh_matrix[6][4], int i) {
    float alpha = modified_dh_matrix[i][0];
    float a     = modified_dh_matrix[i][1];
    float d     = modified_dh_matrix[i][2];
    float theta = modified_dh_matrix[i][3] + motor_theta[i];

    float cos_theta = arm_cos_f32(theta);
    float sin_theta = arm_sin_f32(theta);
    float cos_alpha = arm_cos_f32(alpha);
    float sin_alpha = arm_sin_f32(alpha);

    // 基于改进型 DH 参数的齐次变换矩阵
    // 注意：其中 sin_alpha*d 前面带负号，cos_alpha*d 前面无符号
    float t_data[16] = {
        // 第一行
        cos_theta,            -sin_theta,                0.0f,              a,
        // 第二行
        sin_theta * cos_alpha, cos_theta * cos_alpha,    -sin_alpha,        -sin_alpha * d,
        // 第三行
        sin_theta * sin_alpha, cos_theta * sin_alpha,     cos_alpha,         cos_alpha * d,
        // 第四行
        0.0f,                 0.0f,                     0.0f,              1.0f
    };
		memcpy(t_trans,t_data,sizeof(t_data));
}

// 正向运动学
void forward_kinematics(float matrix[6][4], int num_joints) {
    for (int i = 0; i < num_joints; i++) {
        compute_modified_dh_matrix(matrix, i);//计算变化矩阵
        arm_mat_mult_f32(&T_result, &T_trans, &T_temp);//不断左乘变化矩阵
			  // 将乘法结果复制回 t_result 并更新 T_result
        memcpy(t_result, t_temp, sizeof(t_temp));
    }
		arm_mat_mult_f32(&T_result,&T_xyz_6,&T_xyz_0);
}




