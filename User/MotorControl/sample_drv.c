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

#include "sample_drv.h"                                                             

/**
  * 函数功能: 获取相电流基准值
  * 输入参数:
  * 返回参数:
  * 说    明: 取1024次采样的平均值
  */
void Calculate_Adc_Offset(SAMPLE_STRUCT *p)
{
	if(p->OffsetCnt == 0)
	{
		p->EndFlag = 0;
    p->IuOffset = 0;                                
    p->IwOffset = 0;   
    p->BusOffset = 0;		
		p->OffsetCnt = 0;		
	}
	if(p->OffsetCnt < 1024)
	{
    p->IuOffset += p->IuRaw;              
    p->IwOffset += p->IwRaw;
    p->BusOffset += p->BusRaw;		
		p->OffsetCnt++;                               
	}
  else 
	{
		p->IuOffset = p->IuOffset >> 10;  
		p->IwOffset = p->IwOffset >> 10;	
		p->BusOffset = p->BusOffset >> 10;
		p->BusCalibReal = p->BusOffset * p->BusFactor;
		p->OffsetCnt = 0;                                
		p->EndFlag = 1;                                  
	}		
}

/**
  * 函数功能:计算三相电流值 
  * 输入参数:
  * 返回参数:
  * 说    明:以流入中性点的方向为正
  */
void Calculate_Phase_Current(SAMPLE_STRUCT *p)
{
  p->IuReal =   p->CurrentDir * (p->IuRaw - p->IuOffset) * p->CurrentFactor;
  p->IwReal =   p->CurrentDir * (p->IwRaw - p->IwOffset) * p->CurrentFactor;
	p->IvReal = - p->IuReal - p->IwReal; 
}

/**
  * 函数功能:计算母线电压值 
  * 输入参数:
  * 返回参数:
  * 说    明: 
  */
void Calculate_Bus_Voltage(SAMPLE_STRUCT *p)
{
  p->BusReal   = p->BusRaw  * p->BusFactor;
	p->BusChange = p->BusReal - p->BusCalibReal;
}







