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

#include "motor_sensoruse.h"

/**
  * 函数功能:有感控制 
  * 输入参数:
  * 返回参数:
  * 说    明: 
  */
void Sensoruse_Control()
{
	Calculate_Sin_Cos(MC.Encoder.ElectricalVal,&MC.Foc.SinVal,&MC.Foc.CosVal);	
	switch(MC.Motor.RunMode)
	{
		case ENCODER_CALIB:                                                //编码器校准
		{	
			if(MC.Encoder.CalibFlag == 0)                                    //第一次定位到90度
			{
				MC.Foc.Ud += 0.0001f;
				MC.Foc.Uq = 0;
				MC.Foc.SinVal = 1;                                             //电角度给90度，对应正弦值为1
				MC.Foc.CosVal = 0;                                             //电角度给90度，对应余弦值为0		 								
				if(MC.Foc.Ud >= MC.Identify.VoltageSet[1])                     //校准用的电压与参数识别时一致
				{
					MC.Foc.Ud = 0;
					MC.Encoder.CalibFlag = 1;                                    //第一次定位完成，进行第二次
				}				
			} 	
			
			if(MC.Encoder.CalibFlag == 1)                                    //第二次定位到0度
			{
				MC.Foc.Ud += 0.0001f;
				MC.Foc.Uq = 0;
				MC.Foc.SinVal = 0;                                             //电角度给0，对应正弦值为0
				MC.Foc.CosVal = 1;                                             //电角度给0，对应余弦值为1		 				                  
				
				if(MC.Foc.Ud >= MC.Identify.VoltageSet[1])                     //校准用的电压与参数识别时一致
				{
					MC.Encoder.CalibOffset = MC.Encoder.EncoderVal;              //获得偏置值
					MC.Encoder.CalibFlag = 0;
					MC.Foc.Ud = 0;
					MC.Motor.RunMode = POS_SPEED_CURRENT_LOOP;                   //完成转子校准，进入控制模式
				}	  
			}
	  IPack_Transform(&MC.Foc);                                          //反PACK变换			
		}break;				
		
		case CURRENT_OPEN_LOOP:                                            //电流开环，给定Uq值电机转动
		{   	
			IPack_Transform(&MC.Foc);                                        //反PACK变换
		}break;	
		
		case CURRENT_CLOSE_LOOP:                                           //电流闭环，给定Iq_ref电机转动
		{						
			MC.Foc.Iu = MC.Sample.IuReal;
			MC.Foc.Iv = MC.Sample.IvReal;			
			Clark_Transform(&MC.Foc);                                        //克拉克变换
		
			Pack_Transform(&MC.Foc);                                         //派克变换
			
			MC.Foc.IdLPF = MC.Foc.Id * MC.Foc.IdLPFFactor + MC.Foc.IdLPF * (1 - MC.Foc.IdLPFFactor); //Id低通滤波
			MC.Foc.IqLPF = MC.Foc.Iq * MC.Foc.IqLPFFactor + MC.Foc.IqLPF * (1 - MC.Foc.IqLPFFactor); //Iq低通滤波 

			MC.IqPid.Fbk = MC.Foc.IqLPF;
			MC.IdPid.Fbk = MC.Foc.IdLPF;			
			PID_Control(&MC.IqPid);                                          //Iq闭环
			PID_Control(&MC.IdPid);                                          //Id闭环		


			MC.Foc.Uq = MC.IqPid.Out;			
			MC.Foc.Ud = MC.IdPid.Out;
      IPack_Transform(&MC.Foc);                                        //反PACK变换			
		}break;	
		
		case SPEED_CURRENT_LOOP:
		{		
			MC.Speed.SpeedCalculateCnt++;  			
			if(MC.Speed.SpeedCalculateCnt >= SPEED_DIVISION_FACTOR)          //每SPEED_DIVISION_FACTOR次 执行一次速度闭环
			{
				MC.Speed.SpeedCalculateCnt = 0;
				MC.Speed.ElectricalPosThis = MC.Encoder.ElectricalVal;         //获取当前电角度
				Calculate_Speed(&MC.Speed);                                  	 //根据当前电角度和上次电角度计算电角速度
				MC.Speed.ElectricalSpeedLPF = MC.Speed.ElectricalSpeedRaw * MC.Speed.ElectricalSpeedLPFFactor
				                            + MC.Speed.ElectricalSpeedLPF * (1 - MC.Speed.ElectricalSpeedLPFFactor);//低通滤波	
				MC.Speed.MechanicalSpeed = MC.Speed.ElectricalSpeedLPF / MC.Encoder.PolePairs;				
				
				if(MC.Speed.MechanicalSpeedSet != MC.Speed.MechanicalSpeedSetLast)               //给定了新的目标速度
				{                                                      						
					MC.TAccDec.StartSpeed = MC.Speed.MechanicalSpeedSetLast * MC.Encoder.PolePairs;//设置初速度
					MC.TAccDec.EndSpeed   = MC.Speed.MechanicalSpeedSet     * MC.Encoder.PolePairs;//设置末速度
					T_Shaped_Acc_Dec(&MC.TAccDec);                                                 //T形加减速计算
					if(MC.TAccDec.FinishFlag == 1)                                                 //执行完加减速
					{
						MC.Speed.MechanicalSpeedSetLast = MC.Speed.MechanicalSpeedSet;               //更新上次目标速度
						MC.TAccDec.FinishFlag = 0;
					}					
				}		
				MC.SpdPid.Ref = MC.TAccDec.SpeedOut;					                 //获得目标值   
				MC.SpdPid.Fbk = MC.Speed.ElectricalSpeedLPF;	 					       //反馈速度值	
				if(MC.SpdPid.Fbk > -2000 && MC.SpdPid.Fbk < 2000) 
				{
					MC.SpdPid.Kp = MC.SpdPid.KpMax;
				}
				else
				{
					MC.SpdPid.Kp = MC.SpdPid.KpMin;
				}			
				PID_Control(&MC.SpdPid);                            					 //速度闭环
				MC.IqPid.Ref = MC.SpdPid.Out;	
			}
			MC.Foc.Iu = MC.Sample.IuReal;
			MC.Foc.Iv = MC.Sample.IvReal;			
			Clark_Transform(&MC.Foc);            														 //克拉克变换
	
			Pack_Transform(&MC.Foc);             														 //派克变换

			MC.Foc.IdLPF = MC.Foc.Id * MC.Foc.IdLPFFactor + MC.Foc.IdLPF * (1 - MC.Foc.IdLPFFactor); //Id低通滤波
			MC.Foc.IqLPF = MC.Foc.Iq * MC.Foc.IqLPFFactor + MC.Foc.IqLPF * (1 - MC.Foc.IqLPFFactor); //Iq低通滤波 

			MC.IqPid.Fbk = MC.Foc.IqLPF;
			MC.IdPid.Fbk = MC.Foc.IdLPF;		
		
			PID_Control(&MC.IqPid);               													 //Iq闭环
			PID_Control(&MC.IdPid);              														 //Id闭环

			MC.Foc.Uq = MC.IqPid.Out;			
			MC.Foc.Ud = MC.IdPid.Out;
    	IPack_Transform(&MC.Foc);            														 //反PACK变换			
		}break;	
		
		case POS_SPEED_CURRENT_LOOP:
		{
			MC.Position.PosCalculateCnt++;
			MC.Speed.SpeedCalculateCnt++;			
			if(MC.Position.PosCalculateCnt >= POS_DIVISION_FACTOR)           //POS_DIVISION_FACTOR 执行一次位置闭环
			{											
				MC.Position.PosCalculateCnt = 0;			
				MC.Position.ElectricalPosThis = MC.Encoder.ElectricalVal;			 //获取当前位置
				Calculate_Position(&MC.Position);                              //计算总位置
				MC.PosPid.Fbk = MC.Position.ElectricalPosSum;								   //反馈实际位置
				MC.PosPid.Ref = MC.Position.MechanicalPosSet * POLEPAIRS;			 //给定目标位置
				MC.Position.MechanicalPosRaw = MC.Position.ElectricalPosSum / POLEPAIRS;
				PID_Control(&MC.PosPid);                                       //位置闭环
			}
					
			if(MC.Speed.SpeedCalculateCnt >= SPEED_DIVISION_FACTOR)					 //SPEED_DIVISION_FACTOR 执行一次速度闭环
			{
				MC.Speed.SpeedCalculateCnt = 0;
				MC.Speed.ElectricalPosThis = MC.Encoder.ElectricalVal;
				Calculate_Speed(&MC.Speed);                                    //计算速度
				MC.Speed.ElectricalSpeedLPF = MC.Speed.ElectricalSpeedRaw * MC.Speed.ElectricalSpeedLPFFactor
				                            + MC.Speed.ElectricalSpeedLPF * (1 - MC.Speed.ElectricalSpeedLPFFactor);//低通滤波
				MC.Speed.MechanicalSpeed = MC.Speed.ElectricalSpeedLPF / POLEPAIRS;	
				
				MC.SpdPid.Ref = MC.PosPid.Out;					
				MC.SpdPid.Fbk = MC.Speed.ElectricalSpeedLPF;	 					       //反馈速度值	
				if(MC.SpdPid.Fbk > -2000 && MC.SpdPid.Fbk < 2000) 
				{
					MC.SpdPid.Kp = MC.SpdPid.KpMax;
				}
				else
				{
					MC.SpdPid.Kp = MC.SpdPid.KpMin;
				}							
				PID_Control(&MC.SpdPid);                            					 //速度闭环
				MC.IqPid.Ref = MC.SpdPid.Out;										
			}                                                   
			MC.Foc.Iu = MC.Sample.IuReal;
			MC.Foc.Iv = MC.Sample.IvReal;			
			Clark_Transform(&MC.Foc);                                        //克拉克变换
	
			Pack_Transform(&MC.Foc);                                         //派克变换

			MC.Foc.IdLPF = MC.Foc.Id * MC.Foc.IdLPFFactor + MC.Foc.IdLPF * (1 - MC.Foc.IdLPFFactor); //Id低通滤波 
			MC.Foc.IqLPF = MC.Foc.Iq * MC.Foc.IqLPFFactor + MC.Foc.IqLPF * (1 - MC.Foc.IqLPFFactor); //Iq低通滤波 
			
			MC.IqPid.Fbk = MC.Foc.IqLPF;
			MC.IdPid.Fbk = MC.Foc.IdLPF;			
			PID_Control(&MC.IqPid);                                          //Iq闭环
			PID_Control(&MC.IdPid);                                          //Id闭环			

			MC.Foc.Uq = MC.IqPid.Out;			
			MC.Foc.Ud = MC.IdPid.Out;
			IPack_Transform(&MC.Foc);                                        //反PACK变换
		}break;	
	}
	
	MC.Foc.Ubus = MC.Sample.BusReal;						
  Calculate_SVPWM(&MC.Foc);						
}






