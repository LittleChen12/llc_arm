#include "IK.h"

/////////////////////////////////////////////////////////////////已知值用于IK运算/////////////////////////////////////////////////////////////////////////////////

//已知T0_6
float T0_6[16] = {0.583360,   -0.492574,  0.645804,   0.046752,
0.346615,   -0.568099,  -0.746406,   -0.009192,
0.734540,   0.659268,   -0.160672,   0.568912,
0.000000,   0.000000,   0.000000,   1.000000};

//0度角
float theta_0[6] = {0,0,0,0,0,0};

///////////////////////////////////////////////////////////////////存放IK运算结果////////////////////////////////////////////////////////////////////////////////////////
float theta123_1[6],theta123_2[6],theta123_3[6],theta123_4[6];
float theta456_1_1[3],theta456_1_2[3];
float theta456_2_1[3],theta456_2_2[3];
float theta456_3_1[3],theta456_3_2[3];
float theta456_4_1[3],theta456_4_2[3];
float theta123456[8][6];

////////////////////////////////////////////////////////////////////限制关节角度//////////////////////////////////////////////////////////////////////////////////////

/////////////////////////////////////////////////////////////////////可执行角度/////////////////////////////////////////////////////////////////////////////////////////

/////////////////////////////////////////////////////////////////运算矩阵/////////////////////////////////////////////////////////////////////////////////
//矩阵累乘的临时矩阵									 
arm_matrix_instance_f32	 T_temp_IK;
float t_temp_IK[16] = {0};

//T0_3变换矩阵
arm_matrix_instance_f32 T0_3_MATRIX;
float T0_3_matrix[16] = {0};

//T3_4变换矩阵
arm_matrix_instance_f32 T3_4_MATRIX;
float T3_4_matrix[16] = {0};

//R0_3旋转矩阵
arm_matrix_instance_f32 R0_3_MATRIX;
float R0_3_matrix[9] = {0};

//R3_4旋转矩阵
arm_matrix_instance_f32 R3_4_MATRIX;
float R3_4_matrix[9] = {0};

//R0_E旋转矩阵
arm_matrix_instance_f32 R0_E_MATRIX;
float R0_E_matrix[9] = {0};

//R0_E_T转置矩阵
arm_matrix_instance_f32 R0_E_T_MATRIX;
float R0_E_T_matrix[9] = {0};

//R0_6旋转矩阵
arm_matrix_instance_f32 R0_6_MATRIX;
float R0_6_matrix[9] ={0};

//R_end最后的欧拉角旋转矩阵
arm_matrix_instance_f32 R_END_MATRIX;
float R_end_matrix[9] ={0};

/////////////////////////////////////////////////////////////////自定义函数//////////////////////////////////////////////////////////////////////////////////////
//IK矩阵初始化
void IK_matrix_init(void)
{
	arm_mat_init_f32(&T0_3_MATRIX, 4, 4, T0_3_matrix); 
	arm_mat_init_f32(&T_temp_IK, 4, 4, t_temp_IK);     
  arm_mat_init_f32(&T3_4_MATRIX, 4, 4, T3_4_matrix);   
	
	arm_mat_init_f32(&R0_3_MATRIX, 3, 3, R0_3_matrix); 
	arm_mat_init_f32(&R3_4_MATRIX, 3, 3, R3_4_matrix);     
  arm_mat_init_f32(&R0_E_MATRIX, 3, 3, R0_E_matrix);   
	arm_mat_init_f32(&R0_E_T_MATRIX, 3, 3, R0_E_T_matrix); 
	arm_mat_init_f32(&R0_6_MATRIX, 3, 3, R0_6_matrix);     
  arm_mat_init_f32(&R_END_MATRIX, 3, 3, R_end_matrix);   
}



void IK(float T0_6_matrix[16],float dh_matrix[6][4],float IK_theta[6])
{
	solve_theta123(T0_6_matrix,dh_matrix,theta123_1,theta123_2,theta123_3,theta123_4);
	calc_wrist_angles(theta123_1,T0_6_matrix, dh_matrix,theta456_1_1,theta456_1_2);
	calc_wrist_angles(theta123_2,T0_6_matrix, dh_matrix,theta456_2_1,theta456_2_2);
	calc_wrist_angles(theta123_3,T0_6_matrix, dh_matrix,theta456_3_1,theta456_3_2);
	calc_wrist_angles(theta123_4,T0_6_matrix, dh_matrix,theta456_4_1,theta456_4_2);
	Assign_theta123456();
	
	//默认返回第一组解
	for(int i=0;i<6;i++)
	{
	  IK_theta[i] =  theta123456[0][i];
	}
}

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
                    float sol1[6], float sol2[6],
                    float sol3[6], float sol4[6])
{
    // 提取末端位姿平移部分（注意：C 数组下标从 0 开始）
    float x = T0_6[0*4 + 3];
    float y = T0_6[1*4 + 3];
    float z = T0_6[2*4 + 3];
    
    // 计算 r = x^2 + y^2 + z^2
    float r = x*x + y*y + z*z;
    
    //%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    // 从 dh_matrix 中提取所需 DH 参数：
    float a1 = dh_matrix[1][1];      // Joint2, a
    float a2 = dh_matrix[2][1];      // Joint3, a
    float a3 = dh_matrix[3][1];      // Joint4, a
    float d1 = dh_matrix[0][2];      // Joint1, d
    float d2 = dh_matrix[1][2];      // Joint2, d
    float d3 = dh_matrix[2][2];      // Joint3, d
    float d4 = dh_matrix[3][2];      // Joint4, d
    
    //%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    // 利用模型参数计算中间量
    // 虚拟连杆长度 L (结合 a3 与 d4)
    float L = sqrt(a3 * a3 + d4 * d4);
    
    // 计算 R_x = sqrt(r - (d2+d3)^2 - z^2)
    float sum_d2d3 = d2 + d3;
    float R_x = sqrt(r - sum_d2d3 * sum_d2d3 - z*z);
    
    // 定义水平面变量 X 与竖直方向变量 Z（消除 a1 和 d1 的影响）
    float X = R_x - a1;
    float Z = z - d1;
    
    // 辅助变量 D，依据余弦定律计算（D = cos(D_angle)）
    float D = (a2*a2 + L*L - (Z*Z + X*X)) / (2 * a2 * L);
    
    //%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    // 第一组解（肘上构型1）
    float theta2_sol_1 = PI/2 - atan2(Z, X) - atan2( L * sqrt(1 - D*D), a2 - L*D );
    float theta3_sol_1 = -( acos(D) - atan2(a3, d4) - PI/2 );
    float theta1_sol_1 = atan2(y, x) - asin(sum_d2d3 / sqrt(x*x + y*y));
    
    //%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    // 第二组解（肘下构型1）
    float theta2_sol_2 = atan2( L * sqrt(1 - D*D), a2 - L*D ) + atan2(X, Z);
    float theta3_sol_2 = -( PI - ( acos(D) - atan2(d4, a3) ) );
    float theta1_sol_2 = theta1_sol_1;
    
    //%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    // 为计算第三、第四组解构造新的中间量
    float X_ = X + 2 * a1;  // 新的 X
    float D_ = (a2*a2 + L*L - (Z*Z + X_*X_)) / (2 * a2 * L);
    
    // 第三组解（肘上构型2）
    float theta1_sol_3 = theta1_sol_1 - 2 * acos((d2 + d3) / sqrt(x*x + y*y));
    float theta2_sol_3 = -( atan2(X_, Z) - atan2( L * sqrt(1 - D_*D_), a2 - L*D_ ) );
    float theta3_sol_3 = -( PI - ( acos(D_) - atan2(d4, a3) ) );
    
    // 第四组解（肘下构型2）
    float theta1_sol_4 = theta1_sol_3;
    float theta2_sol_4 = -( atan2(X_, Z) + atan2( L * sqrt(1 - D_*D_), a2 - L*D_ ) );
    float theta3_sol_4 = PI - acos(D_) - atan2(d4, a3);
    
    //%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    // 将四组解写入输出数组（每组解为 {θ1, θ2, θ3}）
    sol1[0] = theta1_sol_1;
    sol1[1] = theta2_sol_1;
    sol1[2] = theta3_sol_1;
    
    sol2[0] = theta1_sol_2;
    sol2[1] = theta2_sol_2;
    sol2[2] = theta3_sol_2;
    
    sol3[0] = theta1_sol_3;
    sol3[1] = theta2_sol_3;
    sol3[2] = theta3_sol_3;
    
    sol4[0] = theta1_sol_4;
    sol4[1] = theta2_sol_4;
    sol4[2] = theta3_sol_4;
}

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
 *    sol2 - 第二组腕部关节角 [theta4; theta5; theta6]
 */


void calc_wrist_angles(float theta[6],
                       float T0_6[16], float dh[6][4],
                       float sol1[3], float sol2[3])
{
    
    
    // 计算前三轴的正运动学变换
	  // 得到T0_3
	  forward_kinematics(dh,3,theta,T0_3_matrix,&T0_3_MATRIX);

    // 提取 T0_3 的旋转部分 R0_3 (3×3)
    float R0_3[9] = {0};
    for(uint8_t i = 0; i < 3; i++){
        for(uint8_t j = 0; j < 3; j++){
            R0_3[i*3 + j] = T0_3_matrix[i*4 + j];
        }
    }
		memcpy(R0_3_matrix,R0_3,sizeof(R0_3));
    
    // 计算 T3_4 = transformation_matrix(0, alpha3, a3, d4)
    compute_modified_dh_matrix(dh, 3, theta_0 ,T3_4_matrix);
    
    // 提取 T3_4 的旋转部分 R3_4 (3×3)
    float R3_4[9] = {0};
    for(uint8_t i = 0; i < 3; i++){
        for(uint8_t j = 0; j < 3; j++){
            R3_4[i*3 + j] = T3_4_matrix[i*4 + j];
        }
    }
		memcpy(R3_4_matrix,R3_4,sizeof(R3_4));
    
    // 计算 R0_E = R0_3 * R3_4
    float R0_E[9] = {0};
    arm_mat_mult_f32(&R0_3_MATRIX, &R3_4_MATRIX, &R0_E_MATRIX);
		memcpy(R0_E,R0_E_matrix,sizeof(R0_E_matrix));
    
    // 提取 T0_6 的旋转部分 R0_6 (3×3)
    float R0_6[9];
    for(uint8_t i = 0; i < 3; i++){
        for(uint8_t j = 0; j < 3; j++){
            R0_6[i*3 + j] = T0_6[i*4 + j];
        }
    }
		memcpy(R0_6_matrix,R0_6,sizeof(R0_6_matrix));
    
    // 计算 R_end = Transpose(R0_E) * R0_6
    float R_end[9] = {0};
    arm_mat_trans_f32(&R0_E_MATRIX,&R0_E_T_MATRIX);
    arm_mat_mult_f32(&R0_E_T_MATRIX, &R0_6_MATRIX, &R_END_MATRIX);
    memcpy(R_end,R_end_matrix,sizeof(R_end_matrix));
		
    // 利用标准 Z–Y–Z 分解（非奇异情况）提取腕部角度
    double sol1_gamma;
    double sol1_beta;
		double sol1_alpha;
		
		sol1_beta = atan2( sqrt(pow(R_end[6],2) + pow(R_end[7],2)) , R_end[8]);
		if (sol1_beta != 0 && sol1_beta != PI) {
        sol1_alpha = atan2f( (R_end[5]/ sinf(sol1_beta)) , (R_end[2]/ sinf(sol1_beta)) );
        sol1_gamma = atan2f( (R_end[7]/ sinf(sol1_beta)) , (-R_end[6]/ sinf(sol1_beta)) );
    }
    else if (sol1_beta == 0) {
        sol1_alpha = 0;
        sol1_gamma = atan2f( -R_end[1], R_end[0] );
    }
    else if (sol1_beta == PI) {
        sol1_alpha = 0;
        sol1_gamma = atan2f( R_end[1], -R_end[0] );
    }
		

    // 第一组腕部解
    float sol1_theta4 = Theta_Normalization(sol1_alpha);
    float sol1_theta5 = sol1_beta - (PI / 2);
    float sol1_theta6 = Theta_Normalization(sol1_gamma);
		
			
    sol1[0] = sol1_theta4;
    sol1[1] = sol1_theta5;
    sol1[2] = sol1_theta6;
    
    // 第二组腕部解
    float sol2_theta4 = sol1_theta4 + PI;
    float sol2_theta5 = - sol1_beta - PI/2;
    float sol2_theta6 = sol1_theta6 + PI;
		
		
    sol2[0] = sol2_theta4;
    sol2[1] = sol2_theta5;
    sol2[2] = sol2_theta6;
}

//给theta123456[8][6]数组赋值，整理八组解
void Assign_theta123456(void)
{
    /*
       组合方式 (组号从 0 开始编号)：
         Group 0: theta123_1 + theta456_1_1
         Group 1: theta123_1 + theta456_1_2
         Group 2: theta123_2 + theta456_2_1
         Group 3: theta123_2 + theta456_2_2
         Group 4: theta123_3 + theta456_3_1
         Group 5: theta123_3 + theta456_3_2
         Group 6: theta123_4 + theta456_4_1
         Group 7: theta123_4 + theta456_4_2
    */

    // 用一个指针数组来选择前三轴解（只取前三个角）
    const float *theta123_arr[4] = { theta123_1, theta123_2, theta123_3, theta123_4 };

    // 用两个指针数组分别保存对应的腕部解
    const float *theta456_first_arr[4]  = { theta456_1_1, theta456_2_1, theta456_3_1, theta456_4_1 };
    const float *theta456_second_arr[4] = { theta456_1_2, theta456_2_2, theta456_3_2, theta456_4_2 };

    // 每个组由一个前三轴解和一组腕部解组合而成，共 8 组解
    for (int i = 0; i < 4; i++) {
        // 对于每个 theta123 组有两个腕部解
        for (int j = 0; j < 2; j++) {
            int sol_index = i * 2 + j;  // 对应的组合组编号

            // 复制前三轴解（前三个角），注意只复制 3 个 float
            memcpy(theta123456[sol_index], theta123_arr[i], 3 * sizeof(float));

            // 根据 j 的值选择腕部解：j==0使用第一组腕部解，j==1使用第二组腕部解
            if (j == 0) {
                memcpy(&(theta123456[sol_index][3]), theta456_first_arr[i], 3 * sizeof(float));
            } else {
                memcpy(&(theta123456[sol_index][3]), theta456_second_arr[i], 3 * sizeof(float));
            }
        }
    }
}

float Theta_Normalization(float theta)
{
	if(theta > 0.0f)
		return (theta - PI);
	else
		return (theta + PI);
}


