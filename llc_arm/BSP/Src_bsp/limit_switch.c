#include "limit_switch.h"

uint8_t Limit_switch_state[6];
/**
 * @brief  限位开关读取函数
 * 
 * 此函数负责读取6个限位开关的状态 1未到达位置 0为到达位置
 * 
 */

void Read_Limit_Switch(void)
{
	Limit_switch_state[0] = HAL_GPIO_ReadPin(Limit_Switch_1_GPIO_Port,Limit_Switch_1_Pin);
	Limit_switch_state[1] = HAL_GPIO_ReadPin(Limit_Switch_2_GPIO_Port,Limit_Switch_2_Pin);
	Limit_switch_state[2] = HAL_GPIO_ReadPin(Limit_Switch_3_GPIO_Port,Limit_Switch_3_Pin);
	Limit_switch_state[3] = HAL_GPIO_ReadPin(Limit_Switch_4_GPIO_Port,Limit_Switch_4_Pin);
	Limit_switch_state[4] = HAL_GPIO_ReadPin(Limit_Switch_5_GPIO_Port,Limit_Switch_5_Pin);
	Limit_switch_state[5] = HAL_GPIO_ReadPin(Limit_Switch_6_GPIO_Port,Limit_Switch_6_Pin);
}

/**
 * @brief  抱闸开关控制函数
 * 
 * 此函数负责控制抱闸开关 1为开 0为抱
 * 
 * @param  status 输入状态
 */
void Control_Relay_Switch(GPIO_PinState status)
{
	HAL_GPIO_WritePin(Relay_Switch_GPIO_Port,Relay_Switch_Pin,status);
}

/**
 * @brief  上电复位
 * 
 * 此函数负责机械臂所有电机复位 
 * 
 * @param  position 初始位置
 */
void Relay_Motor(float* position)
{
	for(uint8_t i=1; i<=6; i++)
	{
	  Motor_mode_setting(i,0x03);
	}
	for(uint8_t id_count=1; id_count<=6; id_count++)
	{
	  Set_Contour_Position(id_count,0,0,position[id_count-1]);
	}
}

/**
 * @brief  电机错误保护
 * 
 * 此函数当电机发生错误信息时，关闭抱闸开关
 * 
 * @param  Motor 电机结构体
 */
void Motor_Error_Project(Motor_parameters_HandleTypeDef* Motor)
{
	uint8_t error_flag = 0;
	
	//判断电机是否有错误信息
	if(Motor->state[2] != 0 ||  Motor->state[3] != 0)
		error_flag = 1;
	
	//执行保护措施
	for(int i=1;i<=6;i++)
	{
		if(error_flag==1)//发生错误
		{
			Clear_Error(i);//清除错误
			Motor_start(i);//启动电机
		}
  }
	
	Control_Relay_Switch(GPIO_PIN_RESET);//关闭所有抱闸开关
	
	for(int i=1;i<=6;i++)
	{
		if(error_flag==1)//发生错误
		{
			Quick_Stop_Motor(i);//快速停止电机
		}
  }		
	
}
