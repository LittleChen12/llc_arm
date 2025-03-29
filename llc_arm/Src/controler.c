#include "controler.h"

//自定义变量
//DH表
float dh_matrix[6][4] = {
	  //alpha    a        d         theta
		{0.0f,     0.0f,   0.212f,    0.0f},       // Joint 1
		{-PI/2.0f, 0.04f,  0.062f,    -PI/2.0f},   // Joint 2
		{0.0f,     0.18f,  -0.05f,    0.0f},       // Joint 3
		{-PI/2.0f, 0.055f, 0.205027f, 0.0f},       // Joint 4
		{PI/2.0f,  0.0f,   0.0f,      PI/2.0f},    // Joint 5
		{-PI/2.0f, 0.0f,   0.0f,   		0.0f}        // Joint 6
};

//每个关节的变化矩阵
arm_matrix_instance_f32 T_trans;
float t_trans[16] = {0};

//矩阵累乘的临时矩阵									 
arm_matrix_instance_f32	 T_temp;
float t_temp[16] = {0};

//正解方案2的输出矩阵
arm_matrix_instance_f32 END_trans;
float end_trans[16]={1.0f, 0.0f, 0.0f, 0.0f,
												0.0f, 1.0f, 0.0f, 0.0f,
												0.0f, 0.0f, 1.0f, 0.0f,
												0.0f, 0.0f, 0.0f, 1.0f};

//正解方案1的输出矩阵
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

arm_matrix_instance_f32 Cumulative_matrix;
float cumulative_matrix[16] = {
												1.0f, 0.0f, 0.0f, 0.0f,
												0.0f, 1.0f, 0.0f, 0.0f,
												0.0f, 0.0f, 1.0f, 0.0f,
												0.0f, 0.0f, 0.0f, 1.0f};									 
//电机角度  
float real_motor_theta[6]={0.513709f,0.514567f,0.515424f,0.516280f,0.517137f,0.517992f};//电机减速比50,50,50,4,4,30 


void matrix_init(void)
{
	arm_mat_init_f32(&T_trans, 4, 4, t_trans);   // 初始化变化矩阵	
	arm_mat_init_f32(&T_result, 4, 4, t_result); // 初始化输出矩阵
	arm_mat_init_f32(&T_xyz_6, 4, 4, t_xyz_6);   // 初始化末端姿态矩阵
	arm_mat_init_f32(&T_xyz_0, 4, 4, t_xyz_0);   // 初始化末端 姿态矩阵
	arm_mat_init_f32(&T_temp, 4, 4, t_temp);     //初始化临时计算矩阵
	arm_mat_init_f32(&Cumulative_matrix, 4, 4, cumulative_matrix); // 累乘矩阵	
}

//测试正解
void dh_test(void)
{
	forward_kinematics(dh_matrix,6,real_motor_theta,t_result,t_trans,&T_result,&T_trans);
	forward_kinematics2(dh_matrix);
}

// 计算改进型 DH 参数对应的变换矩阵
// 参数 modified_dh_matrix 的每一行存放：α, a, d,θ(正常)
void compute_modified_dh_matrix(float modified_dh_matrix[6][4], int i,float motor_theta[6],float t_trans_matrix[16]) 
{
	
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
		memcpy(t_trans_matrix,t_data,sizeof(t_data));
}


//正向运动学
void forward_kinematics(float modified_dh_matrix[6][4], int num_joints,float motor_theta[6],
float t_result_matrix[16],float t_trans_matrix[16],
arm_matrix_instance_f32 *T_result_matrix,arm_matrix_instance_f32 *T_trans_matrix) 
{
		float cumulative_matrix_init[16] = {
													1.0f, 0.0f, 0.0f, 0.0f,
													0.0f, 1.0f, 0.0f, 0.0f,
													0.0f, 0.0f, 1.0f, 0.0f,
													0.0f, 0.0f, 0.0f, 1.0f};			
		memcpy(cumulative_matrix, cumulative_matrix_init, sizeof(&cumulative_matrix_init)*16);				
													
    for (int i = 0; i < num_joints; i++) {
        compute_modified_dh_matrix(modified_dh_matrix, i,motor_theta,t_trans_matrix);//计算变化矩阵
        arm_mat_mult_f32(&Cumulative_matrix, T_trans_matrix, &T_temp);//不断左乘变化矩阵
				memcpy(cumulative_matrix, t_temp, sizeof(&t_temp)*16);				
    }
		// 将乘法结果复制回 t_result 并更新 T_result
		memcpy(t_result_matrix, t_temp, sizeof(&t_temp)*16);		
		arm_mat_mult_f32(&T_result,&T_xyz_6,&T_xyz_0);
}



// 正向运动学方案二
void forward_kinematics2(float dh_matrix[6][4])
{
    // 局部复制 DH 参数矩阵
    float dh[6][4];
    for (int i = 0; i < 6; i++) {
        for (int j = 0; j < 4; j++) {
            dh[i][j] = dh_matrix[i][j];
        }
    }
    // 根据电机实际转角对每个关节的theta进行调整
    for (int i = 0; i < 6; i++) {
        dh[i][3] = real_motor_theta[i];
    }

    // 预计算每个关节角度（从 DH 表中取出并计算三角函数值）
    float c1 = arm_cos_f32(dh[0][3]);
    float s1 = arm_sin_f32(dh[0][3]);
    float c2 = arm_cos_f32(dh[1][3]);
    float s2 = arm_sin_f32(dh[1][3]);
    float c3 = arm_cos_f32(dh[2][3]);
    float s3 = arm_sin_f32(dh[2][3]);
    float c4 = arm_cos_f32(dh[3][3]);
    float s4 = arm_sin_f32(dh[3][3]);
    float c5 = arm_cos_f32(dh[4][3]);
    float s5 = arm_sin_f32(dh[4][3]);
    float c6 = arm_cos_f32(dh[5][3]);
    float s6 = arm_sin_f32(dh[5][3]);

    // 预计算 theta2 + theta3 及其三角函数值
    float t23 = dh[1][3] + dh[2][3];
    float c23 = arm_cos_f32(t23);
    float s23 = arm_sin_f32(t23);
    
    // 提取各杆长(dh[][1]为a参数，dh[][2]为d参数)
    float a1_val = dh[1][1];          // Joint 2
    float a2_val = dh[2][1];          // Joint 3
    float a3_val = dh[3][1];          // Joint 4
    // 以下 a4、a5 未参与本变换矩阵计算，可省略
    float d1_val = dh[0][2];          // Joint 1
    float d2_val = dh[1][2];          // Joint 2
    float d3_val = dh[2][2];          // Joint 3
    float d4_val = dh[3][2];          // Joint 4

    float term1 = c4 * s1 - s4 * (c1 * c2 * s3 + c1 * c3 * s2);
		
    float term2 = s5 * (s1 * s4 + c4 * (c1 * c2 * s3 + c1 * c3 * s2))
                  + c5 * (c1 * s2 * s3 - c1 * c2 * c3);
		
    float r11 = s6 * term1 - c6 * term2;
    
    float r12 = c6 * term1 + s6 * term2;
    
    float r13 = s5 * (c1 * s2 * s3 - c1 * c2 * c3)
                  - c5 * (s1 * s4 + c4 * (c1 * c2 * s3 + c1 * c3 * s2));
    
    float r14 = a3_val * (c1 * c2 * s3 + c1 * c3 * s2)
                  - d4_val * (c1 * s2 * s3 - c1 * c2 * c3)
                  + a1_val * c1 - d2_val * s1 - d3_val * s1 + a2_val * c1 * s2;
    
    float r21 = c6 * ( s5 * (c1 * s4 - c4 * (c2 * s1 * s3 + c3 * s1 * s2))
                        - c5 * (s1 * s2 * s3 - c2 * c3 * s1) )
                   - s6 * (c1 * c4 + s4 * (c2 * s1 * s3 + c3 * s1 * s2));
    
    float r22 = -c6 * (c1 * c4 + s4 * (c2 * s1 * s3 + c3 * s1 * s2))
                   - s6 * ( s5 * (c1 * s4 - c4 * (c2 * s1 * s3 + c3 * s1 * s2))
                            - c5 * (s1 * s2 * s3 - c2 * c3 * s1) );
    
    float r23 = c5 * (c1 * s4 - c4 * (c2 * s1 * s3 + c3 * s1 * s2))
                   + s5 * (s1 * s2 * s3 - c2 * c3 * s1);

    float r24 = a3_val * (s1 * c2 * s3 + s1 * c3 * s2)
                   - d4_val * (s1 * s2 * s3 - c2 * c3 * s1)
                   + d2_val * c1 + d3_val * c1 + a1_val * s1 + a2_val * s1 * s2;
    
    float r31 = -c6 * (s23 * c5 + c23 * c4 * s5) - c23 * s4 * s6;
    
    float r32 = s6 * (s23 * c5 + c23 * c4 * s5) - c23 * s4 * c6;
    
    float r33 = s23 * s5 - c23 * c4 * c5;
    
    float r34 = d1_val + a3_val * c23 - d4_val * s23 + a2_val * c2;
    
    float r41 = 0.0f, r42 = 0.0f, r43 = 0.0f, r44 = 1.0f;
    
    // 填充齐次变换矩阵（按行存放）
    end_trans[0]  = r11;  end_trans[1]  = r12;  end_trans[2]  = r13;  end_trans[3]  = r14;
    end_trans[4]  = r21;  end_trans[5]  = r22;  end_trans[6]  = r23;  end_trans[7]  = r24;
    end_trans[8]  = r31;  end_trans[9]  = r32;  end_trans[10] = r33;  end_trans[11] = r34;
    end_trans[12] = r41;  end_trans[13] = r42;  end_trans[14] = r43;  end_trans[15] = r44;
		
		arm_mat_mult_f32(&END_trans,&T_xyz_6,&T_xyz_0);
}













