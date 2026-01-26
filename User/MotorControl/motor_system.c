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

#include "motor_system.h"


/**
  * 函数功能:电机系统初始化 
  * 输入参数:
  * 返回参数:
  * 说    明: 
  */
void Motor_System_Init(void)
{
	Motor_Struct_Init();                       //结构体参数初始化
}

/**
  * 函数功能:系统运行 
  * 输入参数:
  * 返回参数:
  * 说    明: 
  */
void Motor_System_Run()
{
	if(MC.Sample.EndFlag == 1)                 //已经校准过ADC
	{
		Calculate_Bus_Voltage(&MC.Sample);       //计算母线电压值		
		if(MC.Sample.BusReal <= 12 || MC.Sample.BusReal >= 40)
		{
			MC.Motor.RunState = MOTOR_ERROR;       //供电不正常
		}
		Calculate_Phase_Current(&MC.Sample);     //计算三相电流值	
		MC.SMO.Gain = MC.Sample.BusReal * 0.57735f;
    MC.IdPid.OutMax =  MC.Sample.BusReal * 0.57735f; 
		MC.IdPid.OutMin = -MC.Sample.BusReal * 0.57735f;
    MC.IqPid.OutMax =  MC.Sample.BusReal * 0.57735f; 
		MC.IqPid.OutMin = -MC.Sample.BusReal * 0.57735f;
	}	
	switch (MC.Motor.RunState)
	{		
		case ADC_CALIB:                          //ADC校准
		{
			Calculate_Adc_Offset(&MC.Sample);
			if(MC.Sample.EndFlag == 1)
			{
				MC.Motor.RunState = MOTOR_IDENTIFY;
			}		
		}break;
		
		case MOTOR_IDENTIFY:                     //参数辨识
		{
      Motor_Identify();
			if(MC.Identify.EndFlag == 1)           //辨识完成
			{
			  MC.SMO.Rs = MC.Identify.Rs;          //赋值给需要用到的地方
			  MC.SMO.Ld = MC.Identify.Ld;          //赋值给需要用到的地方
				MC.Motor.RunState = MOTOR_SENSORUSE;   
			}				
		}break;		

		case MOTOR_SENSORUSE:                     //有感控制
		{
		  Calculate_Encoder_Data(&MC.Encoder);    //计算编码器数据			
      Sensoruse_Control();
		}break;	

		case MOTOR_SENSORLESS:                    //无感控制
		{
      Sensorless_Control();
		}break;			
		
		case MOTOR_ERROR:                         //故障报错
		{
			MC.Foc.DutyCycleA = 0;
			MC.Foc.DutyCycleB = 0;
			MC.Foc.DutyCycleC = 0;
		}break;		
		
		case MOTOR_STOP:                          //停机
		{	  			
			MC.Foc.DutyCycleA = 0;
			MC.Foc.DutyCycleB = 0;
			MC.Foc.DutyCycleC = 0;
		}break;
		
		default :
		 break;			
	}		
}


