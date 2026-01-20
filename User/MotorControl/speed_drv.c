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

#include "speed_drv.h"                              

/**
  * 函数功能:计算速度
  * 输入参数: 
  * 返 回 值: 
  * 说    明:
  */
void Calculate_Speed(SPEED_STRUCT *p)
{
  p->ElectricalPosChange = p->ElectricalPosThis - p->ElectricalPosLast;      //计算单位时间内位移
	p->ElectricalPosLast = p->ElectricalPosThis;
	if(p->ElectricalPosChange >= (p->ElectricalValMax * 0.5f))                 //越过编码器零点
	{
		p->ElectricalPosChange = p->ElectricalPosChange - p->ElectricalValMax;
	}
	if(p->ElectricalPosChange <= (-p->ElectricalValMax * 0.5f))
	{
		p->ElectricalPosChange = p->ElectricalPosChange + p->ElectricalValMax;   //越过编码器零点
	}
	p->ElectricalSpeedRaw = p->ElectricalPosChange * p->ElectricalSpeedFactor; //计算原始电角速度
}

/**
  * 函数功能:T形加减速
  * 输入参数: 
  * 返 回 值: 
  * 说    明:
  */
void T_Shaped_Acc_Dec(TSHAPEDACCDEC_STRUCT *p)
{
	if(p->FinishFlag == 0)
	{
    if((p->EndSpeed - p->StartSpeed) > 0)
		{
			p->SumSpeed = p->SumSpeed + p->AccSpeed;
			p->SpeedOut = p->StartSpeed + p->SumSpeed;
			if(p->SpeedOut >= p->EndSpeed)
			{
				p->SpeedOut = p->EndSpeed;
				p->FinishFlag = 1;
			}
		}
		if((p->EndSpeed - p->StartSpeed) < 0)
		{
			p->SumSpeed = p->SumSpeed - p->AccSpeed;
			p->SpeedOut = p->StartSpeed + p->SumSpeed;
			if(p->SpeedOut <= p->EndSpeed)
			{
				p->SpeedOut = p->EndSpeed;
				p->FinishFlag = 1;
			}
		}
    if(p->FinishFlag == 1)
		{
	  	p->SumSpeed = 0;
		}			
	}
		
}





