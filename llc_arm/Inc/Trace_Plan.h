#ifndef __TRACE_PLAN_H
#define __TRACE_PLAN_H


//头文件包含
#include "IK.h"
#include "dstp_motor.h"
#include "cmsis_os.h"



//自定义函数

//计算出当前时间点规划的位置
void Linear_trajectory_planning(float current_time,float p0,float pf,float T,int numPoints, float output);

//整合六个角度当前时间点规划的位置
void Linear_trajectory_ouput(float current_time,float p_init[16],float p_end[16],float T,int numPoints, float theta_output[6]);

//开始执行轨迹
void Trace_run(float T,int numPoints);

void trace_test(float current_time);

//自定义变量
extern float position_init[16];
extern float position_end[16];








#endif