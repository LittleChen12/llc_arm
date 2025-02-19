#include "motor_control_task.h"

float position_test = 0.0f;
float speed_test = 0.0f;
float Torque_test = 0.0f;
float acc_test = 0.0f;
long int tick;
float time_base;

void Motor_Control_start(void *argument)
{
	Motor_origin_setting(1);//设定原点
	osDelay(10);
	Motor_mode_setting(1,0x04);//选择控制模式
	osDelay(10);
	Motor_stop(1);
	osDelay(100);
	Motor_start(1);//启动电机
	osDelay(1000);
  for(;;)
  {
		if(time_base <= 0.1f)
		{
			Torque_test = 0.05f+0.01f*time_base;
			Set_Follow_Torque(1,0x06,0,Torque_test);
		}
		else if(time_base == 10.0f)
		{
			Quick_Stop_Motor(1);
		}
		Read_Actual_Position(1);
		
		//时间1ms
		time_base = ((tick++) * 0.001f);
		osDelay(1);
  }
}
