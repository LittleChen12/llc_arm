#ifndef __TRACE_PLAN_H
#define __TRACE_PLAN_H


//头文件包含
#include "IK.h"
#include "dstp_motor.h"
#include "cmsis_os.h"
#include "controler.h"



//自定义函数

//计算出当前时间点规划的位置
double Linear_trajectory_planning(double current_time, double p0, double pf, double T);

//整合六个角度当前时间点规划的位置
void Linear_trajectory_ouput(float current_time,float p_init[16],float p_end[16],float T,float theta_output[6]);

//开始执行轨迹
void Trace_run(float T,int numPoints);

void trace_test(float current_time);

//自定义变量
extern float position_init[6];
extern float position_end[6];








#endif
