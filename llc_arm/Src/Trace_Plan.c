#include "Trace_Plan.h"

float position_init[6] = {0.2450f,0.3f,0.2572f,-PI,0,0};
float position_end[6] = {0.2450f,-0.3f,0.2572f,-PI,0,0 };
float theta123456_output[6];
float theta123456_output_end[6];

//计算出当前时间点规划的位置
uint8_t count_tick = 0;
double output;
double Linear_trajectory_planning(double current_time, double p0, double pf, double T)
{
    // 静态变量保存上一组参数以及预计算系数
    static int initialized = 0;
    static double tacc, v_mid;
    static double a0, a3, a4;              // 加速段系数
    static double a0_, a1_, a2_, a3_, a4_;  // 减速段系数
    static double last_p0, last_pf, last_T;
    
    // 如果参数发生变化或第一次调用，则重新计算系数
    if (!initialized || p0 != last_p0 || pf != last_pf || T != last_T) {
        initialized = 1;
        last_p0 = p0; 
        last_pf = pf; 
        last_T = T;
        
        // 计算加速时间（10% 的总时间）
        tacc = 0.1 * T;
        // 计算匀速阶段的平均速度
        v_mid = (pf - p0) / T;
        
        // 链式计算各幂次
        const double tacc_2 = tacc * tacc;
        const double tacc_3 = tacc_2 * tacc;
        const double tacc_4 = tacc_2 * tacc_2;
        const double T_2 = T * T;
        const double T_3 = T_2 * T;
        const double T_4 = T_2 * T_2;
        
        // 加速段（前段）多项式系数
        a0 = 0.5 * v_mid * tacc + p0;
        // 加速段中 a1 与 a2 均为 0，因此只需存储 t3 与 t? 项系数
        a3 = v_mid / tacc_2;
        a4 = -v_mid / (2.0 * tacc_3);
        
        // 减速段（后段）多项式系数，对应 MATLAB 中的公式：
        //   a0_ = (0.5*(v_mid*T^4 - 2*v_mid*T^3*tacc + 2*v_mid*T*tacc^3 - v_mid*tacc^4 + 2*p0*tacc^3)) / tacc^3;
        //   a1_ = -(2*v_mid*T^3 - 3*tacc*v_mid*T^2) / tacc^3;
        //   a2_ = (3*(v_mid*T^2 - tacc*v_mid*T)) / tacc^3;
        //   a3_ = -(v_mid*(2*T - tacc)) / tacc^3;
        //   a4_ = (0.5*v_mid) / tacc^3;
        a0_ = (0.5 * (v_mid * T_4 - 2.0 * v_mid * T_3 * tacc 
                 + 2.0 * v_mid * T * tacc_3 - v_mid * tacc_4 
                 + 2.0 * p0 * tacc_3)) / tacc_3;
        a1_ = -(2.0 * v_mid * T_3 - 3.0 * tacc * v_mid * T_2) / tacc_3;
        a2_ = (3.0 * (v_mid * T_2 - tacc * v_mid * T)) / tacc_3;
        a3_ = -(v_mid * (2.0 * T - tacc)) / tacc_3;
        a4_ = (0.5 * v_mid) / tacc_3;
    }
    
    double output = 0.0;
    
    if (current_time <= tacc) {
        // 加速阶段：只需计算 current_time 的三次与四次幂
        double t2 = current_time * current_time;
        double t3 = t2 * current_time;
        double t4 = t3 * current_time;
        output = a0 + a3 * t3 + a4 * t4;
    } 
    else if (current_time <= (T - tacc)) {
        // 匀速阶段（t > tacc 且 t <= T-tacc）
        output = p0 + v_mid * current_time;
    } 
    else {
        // 减速阶段：由于高次方项的累加，在减速阶段算出来的数据会在趋近0的时候变得不稳定，具体表现为正负来回跳动，导致减速阶段电机加速度异常，故采用霍纳法则求值多项式以提高数值稳定性
        output = a0_ + current_time * (a1_ + current_time * (a2_ + current_time * (a3_ + current_time * a4_)));
    }
    
    return output;
}





double Positon_output[6];
float alpha,beta,gamma;
float temp_matrix[16];
float temp_IK_theta[6]={0};
//调用一次，输出一组123456角度
void Linear_trajectory_ouput(float current_time,float p_init[6],float p_end[6],float T,float theta_output[6])
{
	//调用直线轨迹规划函数
	for(int i=0;i<6;i++)
	{
		Positon_output[i] = Linear_trajectory_planning(current_time,p_init[i],p_end[i],T);
	}
	
	alpha = Positon_output[3];
	beta = Positon_output[4];
	gamma = Positon_output[5];

	// 逐项赋值
	temp_matrix[0] = arm_cos_f32(beta) * arm_cos_f32(gamma);
	temp_matrix[1] = arm_cos_f32(gamma) * arm_sin_f32(alpha) * arm_sin_f32(beta) - arm_cos_f32(alpha) * arm_sin_f32(gamma);
	temp_matrix[2] = arm_cos_f32(alpha) * arm_cos_f32(gamma) * arm_sin_f32(beta) + arm_sin_f32(alpha) * arm_sin_f32(gamma);
	temp_matrix[3] = Positon_output[0];

	temp_matrix[4] = arm_cos_f32(beta) * arm_sin_f32(gamma);
	temp_matrix[5] = arm_cos_f32(alpha) * arm_cos_f32(gamma) + arm_sin_f32(alpha) * arm_sin_f32(beta) * arm_sin_f32(gamma);
	temp_matrix[6] = -arm_cos_f32(gamma) * arm_sin_f32(alpha) + arm_cos_f32(alpha) * arm_sin_f32(beta) * arm_sin_f32(gamma);
	temp_matrix[7] = Positon_output[1];

	temp_matrix[8] = -arm_sin_f32(beta);
	temp_matrix[9] = arm_cos_f32(beta) * arm_sin_f32(alpha);
	temp_matrix[10] = arm_cos_f32(alpha) * arm_cos_f32(beta);
	temp_matrix[11] = Positon_output[2];

	temp_matrix[12] = 0.0f;
	temp_matrix[13] = 0.0f;
	temp_matrix[14] = 0.0f;
	temp_matrix[15] = 1.0f; 
	
	//逆解得到角度输出
	IK(temp_matrix,dh_matrix,temp_IK_theta);
	
	//整理
	theta_output[0] = temp_IK_theta[0];
	theta_output[1] = temp_IK_theta[1];
	theta_output[2] = temp_IK_theta[2];
	theta_output[3] = temp_IK_theta[3];
	theta_output[4] = temp_IK_theta[4];
	theta_output[5] = temp_IK_theta[5];
	
}

float flag=0;
//开始执行轨迹(T(s)/numPoints=2ms)
void Trace_run(float T,int numPoints)
{
	//初始化开始时间
	float current_time = 0;
	
	//走到第一个点
	Linear_trajectory_ouput(current_time,position_init ,position_end,T,theta123456_output);
	
	theta123456_output_end[0] = theta123456_output[0] * 50.0f + 165.5f;
	theta123456_output_end[1] = theta123456_output[1] * 50.0f + 79.3f;
	theta123456_output_end[2] = theta123456_output[2] * 50.0f - 54.9f;
	theta123456_output_end[3] = theta123456_output[3] * 4.0f + 3.12f;
	theta123456_output_end[4] = theta123456_output[4] * 4.0f + 6.44f;
	theta123456_output_end[5] = theta123456_output[5] * 30.0f + 56.0f;
	
	for(int i=1;i<=6;i++)
	{
		Motor_mode_setting(i,0x03);//轮廓位置模式
		osDelay(10);
		Contour_Position_Mode(i,0,0,theta123456_output_end[i-1]);
		osDelay(10);
	}
	osDelay(10000);
	
	for(int i=1;i<=6;i++)
	{
		Motor_mode_setting(i,0x06);//位置跟随模式
	}
	osDelay(100);//等待时间不可过短，否则第一个点位会出现抖动

	// 获取当前 tick count，作为第一个周期的参考时间
	TickType_t xLastWakeTime = xTaskGetTickCount();
	// 将 2ms 转换为系统 tick 数
	const TickType_t xFrequency = pdMS_TO_TICKS(2);
	//再开始走轨迹
	for(int i=0;i<=numPoints;i++)
	{
				flag++;
		Linear_trajectory_ouput(current_time,position_init ,position_end,T,theta123456_output);
		theta123456_output_end[0] = theta123456_output[0] * 50.0f + 165.5f;
		theta123456_output_end[1] = theta123456_output[1] * 50.0f + 79.3f;
		theta123456_output_end[2] = theta123456_output[2] * 50.0f - 54.9f;
		theta123456_output_end[3] = theta123456_output[3] * 4.0f + 3.12f;
		theta123456_output_end[4] = theta123456_output[4] * 4.0f + 6.44f;
		theta123456_output_end[5] = theta123456_output[5] * 30.0f + 56.0f;
		
		for(int j=1;j<=6;j++)
		{
			Follow_Position_Mode(j,0,1,theta123456_output_end[j-1]);
		}
		Multi_Axis_Synchronization();
		current_time += 0.002f;
    vTaskDelayUntil(&xLastWakeTime, xFrequency);
  } 
	
}

void trace_test(float current_time)
{
	Linear_trajectory_ouput(current_time,position_init,position_end,10,theta123456_output);
}




