#include "limit_switch.h"

uint8_t Limit_switch_state[6];
/**
 * @brief  ��λ���ض�ȡ����
 * 
 * �˺��������ȡ6����λ���ص�״̬ 1δ����λ�� 0Ϊ����λ��
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
 * @brief  ��բ���ؿ��ƺ���
 * 
 * �˺���������Ʊ�բ���� 1Ϊ�� 0Ϊ��
 * 
 * @param  status ����״̬
 */
void Control_Relay_Switch(GPIO_PinState status)
{
	HAL_GPIO_WritePin(Relay_Switch_GPIO_Port,Relay_Switch_Pin,status);
}

/**
 * @brief  �ϵ縴λ
 * 
 * �˺��������е�����е����λ 
 * 
 * @param  position ��ʼλ��
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
