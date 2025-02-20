#ifndef __DSTP_MOTOR__H
#define __DSTP_MOTOR__H

//头文件包含

#include "fdcan.h"

//自定义结构体

/* Dstp_Motor_Send为dstp电机驱动器控制变量结构体 */
typedef struct
{
	float acc;
	float speed;
	float position;
	float torque;
}Dstp_Motor_Send;
extern Dstp_Motor_Send dstp_motor_send[6];



//自定义函数

void fdcanx_send_data(uint16_t id, uint8_t *data, uint32_t len, uint8_t func_code);
void Motor_origin_setting(uint16_t id);
void Motor_start(uint16_t id);
void Motor_stop(uint16_t id);
void Motor_mode_setting(uint16_t id, uint8_t mode);
void Contour_Position_Mode(uint16_t id,uint8_t echo, uint8_t sync, float targetPosition);
void Variable_Contour_Position_Mode(uint16_t id,uint8_t echo, uint8_t sync, float targetPosition, float speed, float acc, float deceleration);
void Follow_Position_Mode(uint16_t id,uint8_t echo, uint8_t sync, float Follow_Position);


/**
 * @brief  快速停止
 * 
 * 此函数负责快速停止电机 控制数据字节长度：0 功能码： 0x11
 * 
 * @param  id：电机id号 
 */
void Quick_Stop_Motor(uint16_t id);

/**
 * @brief  清除错误
 * 
 * 此函数负责清楚错误 控制数据字节长度：0 功能码： 0x06
 * 
 * @param  id：电机id号 
 */
void Clear_Error(uint16_t id);

/**
 * @brief  多轴同步
 * 
 * 此函数负责启动电机的多轴同步 控制数据字节长度：0 功能码： 0x15
 * 
 * @param  无
 */
void Multi_Axis_Synchronization(void);

/**
 * @brief  设置轮廓转矩
 * 
 * 此函数负责设置电机转矩 控制数据字节长度：6 功能码： 0x21
 * 
 * @param  id：电机id号 echo:数据接收 sync：电机同步功能 Targe_Torque：目标转矩
 */
void Set_Contour_Torque(uint16_t id,uint8_t echo, uint8_t sync, float Targe_Torque);

/**
 * @brief  设置轮廓转矩自定义
 * 
 * 此函数负责设置电机转矩和转矩的爬升速度 控制数据字节长度：12 功能码： 0x22
 * 
 * @param  id：电机id号 echo:数据接收 sync：电机同步功能 Targe_Torque：目标转矩 Targe_Speed：目标爬升速度
 */
void Set_Custom_Contour_Torque(uint16_t id,uint8_t echo, uint8_t sync, float Targe_Torque, float Targe_Speed);

/**
 * @brief  设置跟随转矩
 * 
 * 此函数负责自定义电机的转矩曲线 控制数据字节长度：6 功能码： 0x27
 * 
 * @param  id：电机id号 echo:数据接收 sync：电机同步功能 Targe_Torque：目标转矩 
 */
void Set_Follow_Torque(uint16_t id,uint8_t echo,uint8_t sync,float Targe_Torque);

/**
 * @brief  读取实际位置
 * 
 * 此函数负责读取电机实际位置 控制数据字节长度：0 功能码： 0x32
 * 
 * @param  id：电机id号  
 */
void Read_Actual_Position(uint16_t id);

/**
 * @brief  读取实际速度（已经验证）
 * 
 * 此函数负责读取电机实际速度 控制数据字节长度：0 功能码： 0x33
 * 
 * @param  id：电机id号  
 */
void Read_Actual_Speed(uint16_t id);
	

/**
 * @brief  跟随位置模式
 * 
 * 此函数负责电机的周期位置运动 控制数据字节长度：6 功能码：0x28
 * 
 * @param  id：电机id号 echo:数据接收 sync：电机同步功能 
 */
void Follow_Speed_Mode(uint16_t id,uint8_t echo, uint8_t sync, float Follow_Speed);


/**
 * @brief  设置轮廓速度
 * 
 * 此函数负责设置电机转矩 控制数据字节长度：6 功能码： 0x23
 * 
 * @param  id：电机id号 echo:数据接收 sync：电机同步功能 Targe_Torque：目标转矩
 */
void Set_Contour_Speed(uint16_t id,uint8_t echo, uint8_t sync, float Targe_Speed);

/**
 * @brief  设置轮廓位置
 * 
 * 此函数负责设置电机转矩 控制数据字节长度：6 功能码： 0x25
 * 
 * @param  id：电机id号 echo:数据接收 sync：电机同步功能 Targe_Torque：目标转矩
 */
void Set_Contour_Position(uint16_t id,uint8_t echo, uint8_t sync, float Targe_Position);

#endif
