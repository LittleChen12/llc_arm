#include "motor_control_task.h"

float position_test = 0.0f;
float speed_test = 0.0f;
float Torque_test = 0.0f;
float acc_test = 0.0f;
long int tick;
float time_base;


Dstp_Motor_Send dstp_motor_send[6];

void Motor_Control_start(void *argument)
{
	Motor_origin_setting(3);//设定原点
	osDelay(10);
	Motor_mode_setting(3,0x06);//选择控制模式
	osDelay(10);
	Motor_stop(3);
	osDelay(100);
	Motor_start(3);//启动电机
	osDelay(1000);
  for(;;)
  {
		if(time_base<=2.0f)
		{
			position_test = 0.5*1*time_base*time_base;
		}
		else
		{
			position_test = 2.0f + 2.0f*(time_base-2.0f);
		}
		if(position_test<60)
		{
			Follow_Position_Mode(3,0,0,position_test);
		}
//		Set_Contour_Position(1,0x06,0,position_test);
//		else if(time_base == 10.0f)
//		{
//			Quick_Stop_Motor(1);
//		}
		//Read_Actual_Position(1);
		
		//时间1ms
		time_base = ((tick++) * 0.001f);
		osDelay(1);
  }
}
