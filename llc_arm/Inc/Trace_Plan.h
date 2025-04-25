#ifndef __TRACE_PLAN_H
#define __TRACE_PLAN_H


//头文件包含
#include "IK.h"
#include "dstp_motor.h"
#include "cmsis_os.h"
#include "controler.h"
#include "fdcan_fliter.h"
#include "led_task.h"
#include "Read_adc.h"

//自定义函数

//计算出当前时间点规划的位置
double Linear_trajectory_planning(double current_time, double p0, double pf, double T);

//整合六个角度当前时间点规划的位置
void Linear_trajectory_ouput(float current_time,float p_init[6],float p_end[6],float T,float theta_output[6],float a);

//开始执行轨迹
uint8_t Trace_run(float position_init[6],float position_end[6],float T_total,float a);

//自定义变量




#endif
