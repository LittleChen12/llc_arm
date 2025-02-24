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
	Motor_origin_setting(1);//设定原点
	Motor_origin_setting(2);//设定原点
	Motor_origin_setting(3);//设定原点
	Motor_origin_setting(4);//设定原点
	Motor_origin_setting(5);//设定原点
	Motor_origin_setting(6);//设定原点
	osDelay(10);
//	Motor_mode_setting(6,0x06);//选择控制模式
//	osDelay(10);
//	Motor_stop(6);
//	osDelay(100);
//	Motor_start(6);//启动电机
//	osDelay(1000);
  for(;;)
  {
//    Read_State(3);
//		Read_Bus_Voltage(6);
		Read_Actual_Position(1);
		Read_Actual_Position(2);
		Read_Actual_Position(3);
		Read_Actual_Position(4);
		Read_Actual_Position(5);
		Read_Actual_Position(6);
//		if(time_base<=2.0f)
//		{
//			position_test = 0.5*1*time_base*time_base;
//		}
//		else if(position_test<=3)
//		{
//			position_test = 2.0f + 2.0f*(time_base-2.0f);
//		}
//		Follow_Position_Mode(6,0x7f,0,position_test);

//		
//		//时间1ms
//		time_base = ((tick++) * 0.001f);
		osDelay(2);
  }
}
