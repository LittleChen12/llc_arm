#ifndef __FDCAN_FILTER_H
#define __FDCAN_FILTER_H


//头文件包含
#include "fdcan.h"
#include <stdio.h>
#include <string.h>
//自定义函数
void FDCAN_Filter_Config(void);
/**
 * @brief  FDCAN接收函数
 * 
 * 此函数负责接收数据
 * 
 * @param  *rec_id：接收到的电机id号 *buf：接收到的数据
 *
 * @retval 返回值为数据长度
 */
uint8_t fdcanx_receive(FDCAN_HandleTypeDef *hfdcan, uint16_t *rec_id, uint8_t *buf);

/**
 * @brief  echo处理函数
 * 
 * 此函数负责接收echo后，将数据传回电机结构体
 * 
 * @param  *read_data 读取到的数据
 */
void echo(uint8_t *read_data);

//自定义结构体
typedef struct{
	uint8_t state[4];       //状态信息
  float actual_position;   //实际距离 单位rad
	float actual_speed;      //实际速度 单位rad/s
	float actual_torque;     //实际转矩 单位Nm
	float output_power;      //输出功率 单位W
	float follow_error;      //跟随误差 单位rad
	float bus_voltage;       //总线电压 单位V
	float battery_voltage;   //电池电压 单位V
	uint8_t function_code;   //功能码
}Motor_parameters_HandleTypeDef;
//自定义宏定义
#define  motor_offset 1

#endif
