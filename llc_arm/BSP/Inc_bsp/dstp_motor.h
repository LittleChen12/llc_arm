#ifndef __DSTP_MOTOR__H
#define __DSTP_MOTOR__H

//ͷ�ļ�����

#include "fdcan.h"

//�Զ���ṹ��

/* Dstp_Motor_SendΪdstp������������Ʊ����ṹ�� */
typedef struct
{
	float acc;
	float speed;
	float position;
	float torque;
}Dstp_Motor_Send;
extern Dstp_Motor_Send dstp_motor_send[6];



//�Զ��庯��

void fdcanx_send_data(uint16_t id, uint8_t *data, uint32_t len, uint8_t func_code);
void Motor_origin_setting(uint16_t id);
void Motor_start(uint16_t id);
void Motor_stop(uint16_t id);
void Motor_mode_setting(uint16_t id, uint8_t mode);
void Contour_Position_Mode(uint16_t id,uint8_t echo, uint8_t sync, float targetPosition);
void Variable_Contour_Position_Mode(uint16_t id,uint8_t echo, uint8_t sync, float targetPosition, float speed, float acc, float deceleration);
void Follow_Position_Mode(uint16_t id,uint8_t echo, uint8_t sync, float Follow_Position);


/**
 * @brief  ����ֹͣ
 * 
 * �˺����������ֹͣ��� ���������ֽڳ��ȣ�0 �����룺 0x11
 * 
 * @param  id�����id�� 
 */
void Quick_Stop_Motor(uint16_t id);

/**
 * @brief  �������
 * 
 * �˺�������������� ���������ֽڳ��ȣ�0 �����룺 0x06
 * 
 * @param  id�����id�� 
 */
void Clear_Error(uint16_t id);

/**
 * @brief  ����ͬ��
 * 
 * �˺���������������Ķ���ͬ�� ���������ֽڳ��ȣ�0 �����룺 0x15
 * 
 * @param  ��
 */
void Multi_Axis_Synchronization(void);

/**
 * @brief  ��������ת��
 * 
 * �˺����������õ��ת�� ���������ֽڳ��ȣ�6 �����룺 0x21
 * 
 * @param  id�����id�� echo:���ݽ��� sync�����ͬ������ Targe_Torque��Ŀ��ת��
 */
void Set_Contour_Torque(uint16_t id,uint8_t echo, uint8_t sync, float Targe_Torque);

/**
 * @brief  ��������ת���Զ���
 * 
 * �˺����������õ��ת�غ�ת�ص������ٶ� ���������ֽڳ��ȣ�12 �����룺 0x22
 * 
 * @param  id�����id�� echo:���ݽ��� sync�����ͬ������ Targe_Torque��Ŀ��ת�� Targe_Speed��Ŀ�������ٶ�
 */
void Set_Custom_Contour_Torque(uint16_t id,uint8_t echo, uint8_t sync, float Targe_Torque, float Targe_Speed);

/**
 * @brief  ���ø���ת��
 * 
 * �˺��������Զ�������ת������ ���������ֽڳ��ȣ�6 �����룺 0x27
 * 
 * @param  id�����id�� echo:���ݽ��� sync�����ͬ������ Targe_Torque��Ŀ��ת�� 
 */
void Set_Follow_Torque(uint16_t id,uint8_t echo,uint8_t sync,float Targe_Torque);

/**
 * @brief  ��ȡʵ��λ��
 * 
 * �˺��������ȡ���ʵ��λ�� ���������ֽڳ��ȣ�0 �����룺 0x32
 * 
 * @param  id�����id��  
 */
void Read_Actual_Position(uint16_t id);

/**
 * @brief  ��ȡʵ���ٶȣ��Ѿ���֤��
 * 
 * �˺��������ȡ���ʵ���ٶ� ���������ֽڳ��ȣ�0 �����룺 0x33
 * 
 * @param  id�����id��  
 */
void Read_Actual_Speed(uint16_t id);
	

/**
 * @brief  ����λ��ģʽ
 * 
 * �˺���������������λ���˶� ���������ֽڳ��ȣ�6 �����룺0x28
 * 
 * @param  id�����id�� echo:���ݽ��� sync�����ͬ������ 
 */
void Follow_Speed_Mode(uint16_t id,uint8_t echo, uint8_t sync, float Follow_Speed);


/**
 * @brief  ���������ٶ�
 * 
 * �˺����������õ��ת�� ���������ֽڳ��ȣ�6 �����룺 0x23
 * 
 * @param  id�����id�� echo:���ݽ��� sync�����ͬ������ Targe_Torque��Ŀ��ת��
 */
void Set_Contour_Speed(uint16_t id,uint8_t echo, uint8_t sync, float Targe_Speed);

/**
 * @brief  ��������λ��
 * 
 * �˺����������õ��ת�� ���������ֽڳ��ȣ�6 �����룺 0x25
 * 
 * @param  id�����id�� echo:���ݽ��� sync�����ͬ������ Targe_Torque��Ŀ��ת��
 */
void Set_Contour_Position(uint16_t id,uint8_t echo, uint8_t sync, float Targe_Position);

#endif
