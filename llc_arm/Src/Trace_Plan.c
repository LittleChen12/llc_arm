#include "Trace_Plan.h"

float position_init[16] = {   1.0000,         0,         0,    0.2450,
																	 0,   -1.0000,         0,    0.0120,
																	 0,         0,   -1.0000,    0.2572,
																	 0,         0,         0,    1.0000};
float position_end[16] = {  1,    0,   0,    0.2450,
														0,   -1,   0,    0.2,
														0,    0,   -1,   0.2572,
														0,    0,   0,    1.0000};
float theta123456_output[6];
float theta123456_output_end[6];

//计算出当前时间点规划的位置
float Linear_trajectory_planning(float current_time,float p0,float pf,float T)
{
	  float output;
	  float tacc = 0.1*T;
    float v_mid = (pf - p0)/T;
	
	  float a0 = 0.5 * v_mid * tacc + p0;
	  float a1 = 0;
		float a2 = 0;
		float a3 = v_mid/pow(tacc,2);
		float a4 = -v_mid/(2*pow(tacc,3));

		float a0_ = (0.5*(v_mid*pow(T,4) - 2*v_mid*pow(T,3)*tacc + 2*v_mid*T*pow(tacc,3) - v_mid*pow(tacc,4) + 2*p0*pow(tacc,3)))/pow(tacc,3);
		float a1_ = -(2*v_mid*pow(T,3) - 3*tacc*v_mid*pow(T,2))/pow(tacc,3);
		float a2_ = (3*(v_mid*pow(T,2) - tacc*v_mid*T))/pow(tacc,3);
		float a3_ = -(v_mid*(2*T - tacc))/pow(tacc,3);
		float a4_ = (0.5000*v_mid)/pow(tacc,3);
		

		if (current_time <= tacc) {
				output = a0 + a1 * current_time + a2 * pow(current_time, 2) + a3 * pow(current_time, 3) + a4 * pow(current_time, 4);
		} else if (current_time > tacc && current_time <= (T - tacc)) {
				output = p0 + v_mid * current_time;
		} else {
				output = a0_ + a1_ * current_time + a2_ * pow(current_time, 2) + a3_ * pow(current_time, 3) + a4_ * pow(current_time, 4);
		}
		return output;
}


//调用一次，输出一组123456角度
void Linear_trajectory_ouput(float current_time,float p_init[16],float p_end[16],float T,float theta_output[6])
{
	float xyz_456_init[6],xyz_456_end[6];
	float IK_theta123456_init[6],IK_theta123456_end[6];
	float xyz_456_output[6];
	float temp_matrix[16] = {1,0,0,0,
										0,1,0,0,
	                  0,0,1,0,
	                  0,0,0,1};
	float temp_IK_theta[6]={0};

 
	//IK解出123456角度
	IK(p_init,dh_matrix,IK_theta123456_init);
	IK(p_end,dh_matrix,IK_theta123456_end);
	
	//整合初末位置数组
	xyz_456_init[0] = p_init[3];
	xyz_456_init[1] = p_init[3 + 4];
	xyz_456_init[2] = p_init[3 + 4 + 4];
	xyz_456_init[3] = IK_theta123456_init[3];
	xyz_456_init[4] = IK_theta123456_init[4];
	xyz_456_init[5] = IK_theta123456_init[5];
	
	xyz_456_end[0] = p_end[3];
	xyz_456_end[1] = p_end[3 + 4];
	xyz_456_end[2] = p_end[3 + 4 + 4];
	xyz_456_end[3] = IK_theta123456_end[3];
	xyz_456_end[4] = IK_theta123456_end[4];
	xyz_456_end[5] = IK_theta123456_end[5];
	
	//调用直线轨迹规划函数
	for(int i=0;i<6;i++)
	{
		xyz_456_output[i] = Linear_trajectory_planning(current_time,xyz_456_init[i],xyz_456_end[i],T);
	}
	
	//逆解得到角度输出
	temp_matrix[3] = xyz_456_output[0];
	temp_matrix[3 + 4] = xyz_456_output[1];
	temp_matrix[3 + 4 + 4] = xyz_456_output[2];
	IK(temp_matrix,dh_matrix,temp_IK_theta);
	
	//整理
	theta_output[0] = temp_IK_theta[0];
	theta_output[1] = temp_IK_theta[1];
	theta_output[2] = temp_IK_theta[2];
	theta_output[3] = xyz_456_output[3];
	theta_output[4] = xyz_456_output[4];
	theta_output[5] = xyz_456_output[5];
	
}

//开始执行轨迹(T(s)/numPoints=2ms)
void Trace_run(float T,int numPoints)
{
	//初始化开始时间
	float current_time = 0;
	
	//走到第一个点
	Linear_trajectory_ouput(current_time,position_init ,position_end,T,theta123456_output);
	
	theta123456_output_end[0] = theta123456_output[0] * 50.0f + 165.3f;
	theta123456_output_end[1] = theta123456_output[1] * 50.0f + 80.0f;
	theta123456_output_end[2] = theta123456_output[2] * 50.0f - 54.5f;
	theta123456_output_end[3] = theta123456_output[3] * 4.0f + 3.07f;
	theta123456_output_end[4] = theta123456_output[4] * 4.0f + 12.73f;
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
	osDelay(10);

	//再开始走轨迹
	for(int i=0;i<numPoints;i++)
	{
		Linear_trajectory_ouput(current_time,position_init ,position_end,T,theta123456_output);
		theta123456_output_end[0] = theta123456_output[0] * 50.0f + 165.3f;
		theta123456_output_end[1] = theta123456_output[1] * 50.0f + 80.0f;
		theta123456_output_end[2] = theta123456_output[2] * 50.0f - 54.5f;
		theta123456_output_end[3] = theta123456_output[3] * 4.0f + 3.07f;
		theta123456_output_end[4] = theta123456_output[4] * 4.0f + 12.73f;
		theta123456_output_end[5] = theta123456_output[5] * 30.0f + 56.0f;
		
		for(int j=1;j<=6;j++)
		{
			Follow_Position_Mode(j,0,0,theta123456_output_end[j-1]);
		}
		osDelay(2);
		current_time += 0.002;
  }
	
}

void trace_test(float current_time)
{
	Linear_trajectory_ouput(current_time,position_init,position_end,10,theta123456_output);
}




