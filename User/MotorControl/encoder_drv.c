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

#include "encoder_drv.h"                           

/**
  * 函数功能:电角度发生器
  * 输入参数:
  * 返回参数:
  * 说    明:根据设定的电角速度速度生成电角度 
  */
void Electrical_Angle_Generator(ENCODER_STRUCT *p)
{	
  p->ElectricalValSet += (0.00005f * p->ElectricalSpdSet * 0.01666f * p->EncoderValMax);//根据设定速度 计算电角度值
	if(p->ElectricalValSet >= p->EncoderValMax)                                           //越过编码器边界点
	{
		p->ElectricalValSet = p->ElectricalValSet - p->EncoderValMax;			
	}
	
	if(p->ElectricalValSet < 0)                                                           //越过编码器边界点
	{
		p->ElectricalValSet = p->ElectricalValSet + p->EncoderValMax;			
	}	
}

/**
  * 函数功能:计算外部编码器数据
  * 输入参数:
  * 返回参数:
  * 说    明: 
  */
void Calculate_Encoder_Data(ENCODER_STRUCT *p)
{
	if(p->Dir == 1)                                                         //判断编码器方向  
	{
		p->EncoderVal = p->EncoderValMax - p->EncoderVal;			                //方向取反
	}
	
/**********************************计算电角度********************************/	
	
	p->ElectricalVal = ((p->EncoderVal - p->CalibOffset) * p->PolePairs) % p->EncoderValMax; 
	
	if(p->ElectricalVal < 0)                                                //处理校准可能带来的负值
	{
		p->ElectricalVal = p->ElectricalVal + p->EncoderValMax;			          //计算电角度
	}
}



