#include "motor_control_task.h"

float position_test = 0.0f;
float speed_test = 0.0f;
float Torque_test = 0.0f;
float acc_test = 0.0f;
long int tick;
float time_base;
uint8_t stop_flag;//抱闸开关用



Dstp_Motor_Send dstp_motor_send[6];
float Motor_relay_position[6] = {165.3f,80.0f,-54.5f,3.07f,12.73f,56.0f};//存放电机初始位置

float test[6]={-0.448716222*50.0f + 165.3f,-0.449609607*50.0f + 80.0f,-0.450502543*50.0f - 54.5f,-0.4513950*4.0f + 3.07f,-0.4522870*4.0f + 12.73f,-0.4531786*30.0f + 56.0f};//写入电机角度

void Motor_Control_start(void *argument)
{
	  matrix_init();
  	IK_matrix_init();
	Control_Relay_Switch(GPIO_PIN_RESET);//关闭所有抱闸开关	
	for(uint8_t i=1; i<=6; i++)
	{
	  Motor_mode_setting(i,0x03);
		Motor_start(i);
	}
	for(uint8_t id_count=1; id_count<=6; id_count++)
	{
	  Set_Contour_Speed(id_count,0,0,0);
	}

	Control_Relay_Switch(GPIO_PIN_SET);//打开所有抱闸开关
//	osDelay(6000);
//	Relay_Motor(Motor_relay_position);//所有电机复位
//	osDelay(10000);
//	
//	//2号关节测试用
//	Motor_mode_setting(2,0x06);//选择控制模式 
//  Relay_Motor(test);

		
  for(;;)
  {
		
	//正解
		dh_test();	
		IK_test();
//		for(int i=1;i<=6;i++)
//		{
//			switch(i){
//				case 1:
//				case 2:
//				case 3:
//					Read_Actual_Position(i);
//					motor_theta[i-1]=(Motor[i-1].actual_position - Motor_relay_position[i-1])/50.0f;
//					break;
//				case 4:
//				case 5:
//					Read_Actual_Position(i);
//					motor_theta[i-1]=(Motor[i-1].actual_position - Motor_relay_position[i-1])/4.0f;
//					break;
//				case 6:
//					Read_Actual_Position(i);
//					motor_theta[i-1]=(Motor[i-1].actual_position - Motor_relay_position[i-1])/30.0f;
//					break;
//				default:
//					break;
//			}
//		}
	
		
//		if(stop_flag == 0)//测试代码写在这里面
//		{
//			if(time_base<=2.0f)
//			{
//				position_test = Motor_relay_position[1] + 0.5f*5.0f*time_base*time_base;//  Motor_relay_position是上电后电机复位的位置，此阶段为匀速运动阶段x1 + 1/2 * a * t^2
//			}
//			else if(time_base>2.0f&&time_base<=4.0f)
//			{
//				position_test = Motor_relay_position[1] + 10.0f + 10.0f*(time_base-2.0f);
//			}
//			Follow_Position_Mode(2,0x7f,0,position_test);		
//			
//			//读取限位开关
			Read_Limit_Switch();

//			//时间1ms
//			time_base = ((tick++) * 0.001f);
//		}
//		
//		//调试抱闸用
//		if(HAL_GPIO_ReadPin(KEY1_GPIO_Port,KEY1_Pin) == 1)
//		{
//      for(int i=1;i<=6;i++)
//      {
//				Quick_Stop_Motor(i);
//			}
//			stop_flag = 1;
//			Control_Relay_Switch(GPIO_PIN_RESET);//关闭所有抱闸开关
//			osDelay(1000);
//			for(uint8_t i=1;i<=6;i++)
//			{
//				Motor_stop(i);			
//			}
//		}
//		
//		//电机错误后保护(待测试)
//		for(int i=1;i<=6;i++)
//		{
//			Motor_Error_Project(&Motor[i]);
//		}
		
		osDelay(1);
  }
}
