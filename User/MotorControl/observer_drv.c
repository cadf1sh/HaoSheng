/**
  ******************************************************************************
  * 文件名程: 
  * 作    者: 浩然
  * 版    本: V1.0
  * 编写日期: 
  * 功    能: 
  ******************************************************************************
  */

#include "observer_drv.h"  

/**
  * 函数功能:滑膜观测器
  * 输入参数:
  * 返回参数:
  * 说    明:
  */
void SMO_Calculate(SMO_STRUCT *p)
{ 	
	p->IalphaFore += p->Ts * (-p->Rs / p->Ld * p->IalphaFore + (p->Ualpha - p->EalphaForeLPF) / p->Ld);
	p->IbetaFore  += p->Ts * (-p->Rs / p->Ld * p->IbetaFore  + (p->Ubeta  - p->EbetaForeLPF)  / p->Ld);	
	
	if     ((p->IalphaFore - p->Ialpha) > 1.0f) p->EalphaFore = p->Gain;
	else if((p->IalphaFore - p->Ialpha) < -1.0f) p->EalphaFore = -p->Gain;	
	else     p->EalphaFore = p->Gain * (p->IalphaFore - p->Ialpha);
	
	if     ((p->IbetaFore - p->Ibeta) > 1.0f) p->EbetaFore = p->Gain;
	else if((p->IbetaFore - p->Ibeta) < -1.0f) p->EbetaFore = -p->Gain;	
	else     p->EbetaFore = p->Gain * (p->IbetaFore - p->Ibeta);
	
	p->EalphaForeLPF = p->EalphaFore * p->EabForeLPFFactor + p->EalphaForeLPF * (1 - p->EabForeLPFFactor); 
	p->EbetaForeLPF  = p->EbetaFore  * p->EabForeLPFFactor + p->EbetaForeLPF  * (1 - p->EabForeLPFFactor);	
}

/**
  * 函数功能:高频注入及信号解析
  * 输入参数:
  * 返回参数:
  * 说    明:
  */
void HFI_Calculate(HFI_STRUCT *p)
{
	/*****************************初始角度辨识*******************************/
	if(p->NSDFlag == 0)
	{	
		p->NSDCount++;	
		p->IdHigh = (p->Id - p->IdLast) * 0.5f;           //提取高频电流分量
		if(p->IdHigh < 0)
		{
			p->IdHigh = -p->IdHigh;			
		}
		if(p->NSDCount < 400)                             //0  等待观测角度收敛
		{
			p->IdRef = 0.0f;
		}
		else if(p->NSDCount >= 400 && p->NSDCount < 600)  //正
		{
			p->IdRef = 5.0f;
		}
		else if(p->NSDCount >= 600 && p->NSDCount < 610)  //正+采样
		{
			p->IdRef = 5.0f;
			p->NSDSum1 += p->IdHigh;
		}
		else if(p->NSDCount >= 610 && p->NSDCount < 810)  //0
		{
			p->IdRef = 0.0f;
		}
		else if(p->NSDCount >= 810 && p->NSDCount < 1010) //负
		{
			p->IdRef = -5.0f;
		}
		else if(p->NSDCount >= 1010 && p->NSDCount < 1020)//负+采样
		{
			p->IdRef = -5.0f;
			p->NSDSum2 += p->IdHigh;
		}
		else if(p->NSDCount == 1020)                      //0
		{
			p->IdRef = 0.0f;
		}	
		else if(p->NSDCount == 1021)                      //0
		{
			p->NSDFlag = 1;	
			p->NSDCount = 0;		
			if(p->NSDSum2 > p->NSDSum1)
			{
				p->NSDOut = 1;
			}
			else
			{
				p->NSDOut = 0;
			}
		}			
	}
	/*****************************信号注入与解析*******************************/	
	p->IdBase = (p->Id + p->IdLast) * 0.5f;                //提取D轴基频电流分量
	p->IqBase = (p->Iq + p->IqLast) * 0.5f;    			       //提取Q轴基频电流分量
	p->IdLast = p->Id;
	p->IqLast = p->Iq;		
	
  p->IalphaHighLast = p->IalphaHigh;
  p->IbetaHighLast  = p->IbetaHigh; 
	p->IalphaHigh = (p->Ialpha - p->IalphaLast) * 0.5f;    //提取α轴高频电流分量
	p->IbetaHigh  = (p->Ibeta  - p->IbetaLast)  * 0.5f;	   //提取β轴高频电流分量	                                                       	                                                       
	p->IalphaLast = p->Ialpha; 
	p->IbetaLast  = p->Ibeta; 
	
	if(p->Dir == 0)                                        
	{
		p->IalphaOut = p->IalphaHigh - p->IalphaHighLast;
		p->IbetaOut  = p->IbetaHigh  - p->IbetaHighLast;	
		if(p->Uin < 0)
		{
			p->Uin = -p->Uin;
		}
		p->Dir = 1;
	}
	else if(p->Dir == 1)
	{
		p->IalphaOut = p->IalphaHighLast - p->IalphaHigh;
		p->IbetaOut  = p->IbetaHighLast  - p->IbetaHigh;	
		if(p->Uin > 0)
		{
			p->Uin = -p->Uin;
		}
		p->Dir = 0;
	}	
}

/**
  * 函数功能:正交锁相环 
  * 输入参数:
  * 返回参数:
  * 说    明: 锁相跟随
  */
void PLL_Calculate(PLL_STRUCT *p)
{ 	 	
  p->ThetaErr = p->Dir * (p->CosVal * p->Ain + p->SinVal * p->Bin);
	p->PPart  = p->Kp * p->ThetaErr;
  p->IPart += p->Ki * p->ThetaErr;	
	p->WeFore = p->PPart + p->IPart;
	p->WeForeLPF = p->WeFore * p->WeForeLPFFactor + p->WeForeLPF * (1 - p->WeForeLPFFactor);
	p->ThetaFore += p->WeFore * p->Ts;
  if(p->ThetaFore > 6.28318f) 
	{
		p->ThetaFore -= 6.28318f;  		       //角度归一化  0-2Π
	}
	else if(p->ThetaFore < 0) 
	{
		p->ThetaFore += 6.28318f;   	       //角度归一化  0-2Π
	}	
	p->ThetaCompensate = p->WeForeLPF * p->Ts + p->ThetaFore;  //观测角度补偿
  if(p->ThetaCompensate > 6.28318f) 
	{
		p->ThetaCompensate -= 6.28318f;  		 //角度归一化  0-2Π
	}
	else if(p->ThetaCompensate < 0)
	{
		p->ThetaCompensate += 6.28318f;   	 //角度归一化  0-2Π
	}		
  p->ETheta	= p->ThetaFore * 651.7395f;  //单位转换 0-2Π转为 0-4095 与正余弦计算函数对应
}

	






