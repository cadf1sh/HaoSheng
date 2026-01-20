/**
  ******************************************************************************
  * 文件名称: 
  * 作    者: √
  * 版    本: V1.0
  * 编写日期: 
  * 功    能: PID控制器
  ******************************************************************************
  */
  
/* 包含头文件 ----------------------------------------------------------------*/
#include "pid_control.h"

/**
  * 函数功能: 参数初始化
  * 输入参数:
  * 返回参数:
  * 说    明: 
  */
void Pid_Control_Init(Pid_Control_Handle *p){
    p->sign = 1;
    p->sign_last = p->sign;
    p->err = 0;
    p->err_delta = 0;
    p->err_integral = 0;
    p->out = 0;
}

/**
  * 函数功能: 带积分和输出限幅以及抗积分饱和的PID控制器
  * 输入参数:
  * 返回参数:
  * 说    明: sign变量是一个标志，可用于在转向快速切换时清除积分项
  */
void Pid_Control_Cal(Pid_Control_Handle *p){
    
    p->err_integral = p->sign == p->sign_last ? p->err_integral : 0;
    p->sign_last = p->sign;
    
    if(fabsf(p->out) < p->out_lim){
        p->err_integral += p->ki * p->err;
    }
    p->err_integral = sat(p->err_integral, -p->integral_lim, p->integral_lim);
    p->err_delta = p->err - p->err_last;
    p->err_last = p->err;
    p->out = p->kp * p->err + p->err_integral + p->kd * p->err_delta;
    p->out = sat(p->out, -p->out_lim, p->out_lim);
}

