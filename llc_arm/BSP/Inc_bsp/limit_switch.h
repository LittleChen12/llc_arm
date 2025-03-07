#ifndef __LIMIT_SWITCH__H
#define __LIMIT_SWITCH__H

//头文件包含
#include <stdint.h>
#include "main.h"
#include "dstp_motor.h"

//全局变量
extern uint8_t Limit_switch_state[6];

//自定义函数
/**
 * @brief  限位开关读取函数
 * 
 * 此函数负责读取6个限位开关的状态 1未到达位置 0为到达位置
 * 
 */
void Read_Limit_Switch(void);

/**
 * @brief  抱闸开关控制函数
 * 
 * 此函数负责控制抱闸开关 1为开 0为抱
 * 
 * @param  status 输入状态
 */
void Control_Relay_Switch(GPIO_PinState status);

/**
 * @brief  上电复位
 * 
 * 此函数负责机械臂所有电机复位 
 * 
 * @param  position 初始位置
 */
void Relay_Motor(float* position);													

#endif


