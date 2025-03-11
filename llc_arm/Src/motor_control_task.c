#include "motor_control_task.h"

float position_test = 0.0f;
float speed_test = 0.0f;
float Torque_test = 0.0f;
float acc_test = 0.0f;
long int tick;
float time_base;
uint8_t stop_flag;//抱闸开关用



Dstp_Motor_Send dstp_motor_send[6];
float Motor_relay_position[6] = {165.3f,79.1f,-53.77f,3.07f,12.71,55.8};//存放电机初始位置


void Motor_Control_start(void *argument)
{
		dh_test();
//	Control_Relay_Switch(GPIO_PIN_RESET);//关闭所有抱闸开关	
//	for(uint8_t i=1; i<=6; i++)
//	{
//	  Motor_mode_setting(i,0x03);
//		Motor_start(i);
//	}
//	for(uint8_t id_count=1; id_count<=6; id_count++)
//	{
//	  Set_Contour_Speed(id_count,0,0,0);
//	}

//	Control_Relay_Switch(GPIO_PIN_SET);//打开所有抱闸开关
//	osDelay(6000);
//	Relay_Motor(Motor_relay_position);//所有电机复位
//	osDelay(10000);
//	
//	//2号关节测试用
//	Motor_mode_setting(2,0x06);//选择控制模式 
//	//2号关节测试用
  for(;;)
  {
//		if(stop_flag == 0)//测试代码写在这里面
//		{
//			if(time_base<=2.0f)
//			{
//				position_test = Motor_relay_position[1] + 0.5f*5.0f*time_base*time_base;
//			}
//			else if(time_base>2.0f&&time_base<=4.0f)
//			{
//				position_test = Motor_relay_position[1] + 10.0f + 10.0f*(time_base-2.0f);
//			}
//			Follow_Position_Mode(2,0x7f,0,position_test);		
//			
//			//读取限位开关
//			Read_Limit_Switch();

//			//时间1ms
//			time_base = ((tick++) * 0.001f);
//		}
		
		//调试抱闸用
//		if(HAL_GPIO_ReadPin(KEY1_GPIO_Port,KEY1_Pin) == 1)
//		{
//			stop_flag = 1;
//			Control_Relay_Switch(GPIO_PIN_RESET);//关闭所有抱闸开关
//			osDelay(1000);
//			for(uint8_t i=1;i<=6;i++)
//			{
//				Motor_stop(i);			
//			}
//		}
		
		osDelay(1);
  }
}
