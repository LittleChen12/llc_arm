#ifndef __FDCAN_FILTER_H
#define __FDCAN_FILTER_H


//ͷ�ļ�����
#include "fdcan.h"
#include <stdio.h>
#include <string.h>
//�Զ��庯��
void FDCAN_Filter_Config(void);
/**
 * @brief  FDCAN���պ���
 * 
 * �˺��������������
 * 
 * @param  *rec_id�����յ��ĵ��id�� *buf�����յ�������
 *
 * @retval ����ֵΪ���ݳ���
 */
uint8_t fdcanx_receive(FDCAN_HandleTypeDef *hfdcan, uint16_t *rec_id, uint8_t *buf);

/**
 * @brief  echo������
 * 
 * �˺����������echo�󣬽����ݴ��ص���ṹ��
 * 
 * @param  *read_data ��ȡ��������
 */
void echo(uint8_t *read_data);

//�Զ���ṹ��
typedef struct{
	uint8_t state[4];       //״̬��Ϣ
  float actual_position;   //ʵ�ʾ��� ��λrad
	float actual_speed;      //ʵ���ٶ� ��λrad/s
	float actual_torque;     //ʵ��ת�� ��λNm
	float output_power;      //������� ��λW
	float follow_error;      //������� ��λrad
	float bus_voltage;       //���ߵ�ѹ ��λV
	float battery_voltage;   //��ص�ѹ ��λV
	uint8_t function_code;   //������
}Motor_parameters_HandleTypeDef;
//�Զ���궨��
#define  motor_offset 1

#endif
