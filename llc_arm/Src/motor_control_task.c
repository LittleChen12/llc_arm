#include "motor_control_task.h"

float position_test = 0.0f;
float speed_test = 0.0f;
float Torque_test = 0.0f;
float acc_test = 0.0f;
long int tick;
float time_base;
uint8_t stop_flag;//��բ������



Dstp_Motor_Send dstp_motor_send[6];
float Motor_relay_position[6] = {165.3f,79.1f,-53.77f,3.07f,12.71,55.8};//��ŵ����ʼλ��


void Motor_Control_start(void *argument)
{
		dh_test();
//	Control_Relay_Switch(GPIO_PIN_RESET);//�ر����б�բ����	
//	for(uint8_t i=1; i<=6; i++)
//	{
//	  Motor_mode_setting(i,0x03);
//		Motor_start(i);
//	}
//	for(uint8_t id_count=1; id_count<=6; id_count++)
//	{
//	  Set_Contour_Speed(id_count,0,0,0);
//	}

//	Control_Relay_Switch(GPIO_PIN_SET);//�����б�բ����
//	osDelay(6000);
//	Relay_Motor(Motor_relay_position);//���е����λ
//	osDelay(10000);
//	
//	//2�Źؽڲ�����
//	Motor_mode_setting(2,0x06);//ѡ�����ģʽ 
//	//2�Źؽڲ�����
  for(;;)
  {
//		if(stop_flag == 0)//���Դ���д��������
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
//			//��ȡ��λ����
//			Read_Limit_Switch();

//			//ʱ��1ms
//			time_base = ((tick++) * 0.001f);
//		}
		
		//���Ա�բ��
//		if(HAL_GPIO_ReadPin(KEY1_GPIO_Port,KEY1_Pin) == 1)
//		{
//			stop_flag = 1;
//			Control_Relay_Switch(GPIO_PIN_RESET);//�ر����б�բ����
//			osDelay(1000);
//			for(uint8_t i=1;i<=6;i++)
//			{
//				Motor_stop(i);			
//			}
//		}
		
		osDelay(1);
  }
}
