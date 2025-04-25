#include "motor_control_task.h"

float position_test = 0.0f;
float speed_test = 0.0f;
float Torque_test = 0.0f;
float acc_test = 0.0f;
long int tick;
float time_base;
uint8_t stop_flag;//抱闸开关用


Dstp_Motor_Send dstp_motor_send[6];
float Motor_relay_position[6] = {165.5f,78.95f,-54.55f,3.12f,6.44f,56.0f};//存放电机初始位置

float test[6]={-0.448716222*50.0f + 165.5f,-0.449609607*50.0f + 78.95f,-0.450502543*50.0f -54.55f,-0.4513950*4.0f + 3.12f,-0.4522870*4.0f + 6.44f,-0.4531786*30.0f + 56.0f};//写入电机角度

float position_init[6] = {0.2450f,-0.045f,0.2955f,-PI,0,0};
float position_end[6] =  {0.2450f,0.045f,0.2962f,-PI,0,0};
float position_temp[6];
uint8_t x_count;



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
	osDelay(6000);
	Relay_Motor(Motor_relay_position);//所有电机复位
	osDelay(10000);

//	Trace_run(position_init,position_end,1);

	for(x_count = 0;x_count<20;x_count++)
	{
    if (x_count % 2 == 0)  // 判断是否为偶数
    {
        Trace_run(position_init, position_end,13,0.2);  // 直接传参
				memcpy(position_temp,position_end,sizeof(position_end));
				position_temp[0] = position_temp[0] + 0.003f;
				Trace_run(position_end,position_temp,0.8,0.055);
    }
    else  // 奇数的情况
    {
        Trace_run(position_end, position_init,13,0.2);  // 直接传参
				memcpy(position_temp,position_init,sizeof(position_init));
				position_temp[0] = position_temp[0] + 0.003f;
				Trace_run(position_init,position_temp,0.8,0.055);
    }
    // 更新位置值
    position_init[2] = position_init[2] + 0.00004f;
    position_end[2] = position_end[2] + 0.00004f;
    position_init[0] = position_init[0] + 0.003f;
    position_end[0] = position_end[0] + 0.003f;
	}
	
  for(;;)
  {
		//读取限位开关
		Read_Limit_Switch();

		//调试抱闸用
		if(HAL_GPIO_ReadPin(KEY1_GPIO_Port,KEY1_Pin) == 1)
		{
      for(int i=1;i<=6;i++)
      {
				Quick_Stop_Motor(i);
			}
			stop_flag = 1;
			Control_Relay_Switch(GPIO_PIN_RESET);//关闭所有抱闸开关
			osDelay(1000);
			for(uint8_t i=1;i<=6;i++)
			{
				Motor_stop(i);			
			}
		}
		
//		//电机错误后保护(待测试)
//		for(int i=1;i<=6;i++)
//		{
//			Motor_Error_Project(&Motor[i]);
//		}
			osDelay(10);
  }
}


