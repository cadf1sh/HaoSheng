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

#include "position_drv.h"                                

/**
  * 函数功能: 计算位置 
  * 输入参数:
  * 返回参数:
  * 说    明: 
  */
void Calculate_Position(POSITION_STRUCT *p)
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
	p->ElectricalPosSum = p->ElectricalPosSum + p->ElectricalPosChange;        //计算总位置
}










