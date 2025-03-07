#ifndef __LIMIT_SWITCH__H
#define __LIMIT_SWITCH__H

//ͷ�ļ�����
#include <stdint.h>
#include "main.h"
#include "dstp_motor.h"

//ȫ�ֱ���
extern uint8_t Limit_switch_state[6];

//�Զ��庯��
/**
 * @brief  ��λ���ض�ȡ����
 * 
 * �˺��������ȡ6����λ���ص�״̬ 1δ����λ�� 0Ϊ����λ��
 * 
 */
void Read_Limit_Switch(void);

/**
 * @brief  ��բ���ؿ��ƺ���
 * 
 * �˺���������Ʊ�բ���� 1Ϊ�� 0Ϊ��
 * 
 * @param  status ����״̬
 */
void Control_Relay_Switch(GPIO_PinState status);

/**
 * @brief  �ϵ縴λ
 * 
 * �˺��������е�����е����λ 
 * 
 * @param  position ��ʼλ��
 */
void Relay_Motor(float* position);													

#endif


