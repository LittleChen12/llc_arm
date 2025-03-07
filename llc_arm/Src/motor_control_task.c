#include "motor_control_task.h"

float position_test = 0.0f;
float speed_test = 0.0f;
float Torque_test = 0.0f;
float acc_test = 0.0f;
long int tick;
float time_base;

uint8_t test[6];

Dstp_Motor_Send dstp_motor_send[6];
float Motor_relay_position[6] = {165.3f,79.1f,-53.77f,3.07f,12.71,55.8};//��ŵ����ʼλ��
	
void Motor_Control_start(void *argument)
{
	Control_Relay_Switch(GPIO_PIN_RESET);//�ر����б�բ����	
	for(uint8_t i=1; i<=6; i++)
	{
	  Motor_mode_setting(i,0x03);
		Motor_start(i);
	}
	for(uint8_t id_count=1; id_count<=6; id_count++)
	{
	  Set_Contour_Speed(id_count,0,0,0);
	}

	Control_Relay_Switch(GPIO_PIN_SET);//�����б�բ����
	osDelay(6000);
	Relay_Motor(Motor_relay_position);//���е����λ
	osDelay(10000);
	Motor_mode_setting(2,0x06);//ѡ�����ģʽ 
  for(;;)
  {

			
      //Read_State(6);
//		Read_Bus_Voltage(6);
//		Read_Actual_Position(1);
//		Read_Actual_Position(2);
//		Read_Actual_Position(3);
//		Read_Actual_Position(4);
//		Read_Actual_Position(5);
//		Read_Actual_Position(6);
		
		if(time_base<=2.0f)
		{
			position_test = Motor_relay_position[1] + 0.5f*5.0f*time_base*time_base;
		}
		else if(time_base>2.0f&&time_base<=4.0f)
		{
			position_test = Motor_relay_position[1] + 10.0f + 10.0f*(time_base-2.0f);
		}
		Follow_Position_Mode(2,0x7f,0,position_test);
//		Follow_Speed_Mode(6,0x7f,0,10.0f);
		
		Read_Limit_Switch();
		
		//ʱ��1ms
		time_base = ((tick++) * 0.001f);
		osDelay(1);
  }
}
