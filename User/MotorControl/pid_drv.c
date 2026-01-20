/**
  ******************************************************************************
  * 文件名程: 
  * 作    者: 浩然
  * 版    本: V1.0
  * 编写日期: 
  * 功    能: 
  ******************************************************************************
  */
/* 包含头文件 ----------------------------------------------------------------*/

#include "pid_drv.h"                                  

/**
  * 函数功能:PID控制 
  * 输入参数:
  * 返回参数:
  * 说    明:抗积分饱和PID
  */
void PID_Control(PID_STRUCT *p)
{
	p->Err = p->Ref - p->Fbk;	                            //计算误差	
		
	if(p->Out < p->OutMax && p->Out > p->OutMin)          //没达到输出限幅就一直积分
	{
		p->Integrate = p->Integrate + p->Err;
	}	
	
  p->Out = p->Kp * p->Err +                             //计算比例项
	         p->Ki * p->Integrate +                       //计算积分项
	         p->Kd * (p->Err - p->ErrLast);               //计算微分项
	
	if(p->Out >= p->OutMax)                               //输出上限幅
	{
	  p->Out = p->OutMax;
	}		
	if(p->Out <= p->OutMin)                               //输出下限幅
	{
	  p->Out = p->OutMin;
	}	
	
  p->ErrLast = p->Err;		
}

/**
  * 函数功能:重置所有相关变量为零 
  * 输入参数:
  * 返回参数:
  * 说    明:
  */
void PID_Clear(PID_STRUCT *p)
{
	p->Ref = 0;       //重置目标值
	p->Fbk = 0;				//重置反馈值
	p->Out = 0;       //重置输出值
	p->Err = 0;       //重置误差值
	p->ErrLast = 0;   //重置误差值
	p->Integrate = 0; //重置积分项
}




