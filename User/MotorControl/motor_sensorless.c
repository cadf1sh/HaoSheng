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

#include "motor_sensorless.h"

/**
  * 函数功能:无感控制 
  * 输入参数:
  * 返回参数:
  * 说    明: 
  */
void Sensorless_Control()
{
	static u8  step = 0;                	 	  //运行步骤
	static u8  errorcnt = 0;               	  //失败次数
	static u16 runtim = 0;                 	  //运行时间
  static u16 comparecnt = 0;              	//比较计数		
	static u16 successcnt = 0;             		//正确次数
	static u16 errortimout = 0;            	  //超时报错
	switch(MC.Motor.RunMode)
	{
		case STRONG_DRAG_CURRENT_OPEN:             //电流开环强拖         
		{	
			MC.Foc.Ud = 0.6f;                        //设置开环强拖时的电压
			MC.Encoder.ElectricalSpdSet = 1000;      //设置强拖时的电角速度（RPM）
			Electrical_Angle_Generator(&MC.Encoder); //根据设定的电角速度实时计算电角度
			Calculate_Sin_Cos(MC.Encoder.ElectricalValSet,&MC.Foc.SinVal,&MC.Foc.CosVal);	//计算正余弦值		
			IPack_Transform(&MC.Foc);                //反PACK变换	
		}	
    break;	
		
		case STRONG_DRAG_CURRENT_CLOSE:            //电流闭环强拖         
		{	
			MC.IdPid.Ref = 1;                        //设置闭环强拖时的电流
			MC.Encoder.ElectricalSpdSet = 1000;      //设置强拖时的电角速度（RPM）
			Electrical_Angle_Generator(&MC.Encoder); //根据设定的电角速度实时计算电角度
			Calculate_Sin_Cos(MC.Encoder.ElectricalValSet,&MC.Foc.SinVal,&MC.Foc.CosVal); //计算正余弦值		
			MC.Foc.Iu = MC.Sample.IuReal;
			MC.Foc.Iv = MC.Sample.IvReal;			
			Clark_Transform(&MC.Foc);           		 //克拉克变换
			
			Pack_Transform(&MC.Foc);                 //派克变换

			MC.Foc.IdLPF = MC.Foc.Id * MC.Foc.IdLPFFactor + MC.Foc.IdLPF * (1 - MC.Foc.IdLPFFactor); //Id低通滤波
			MC.Foc.IqLPF = MC.Foc.Iq * MC.Foc.IqLPFFactor + MC.Foc.IqLPF * (1 - MC.Foc.IqLPFFactor); //Iq低通滤波 

			MC.IqPid.Fbk = MC.Foc.IqLPF;
			MC.IdPid.Fbk = MC.Foc.IdLPF;			
			PID_Control(&MC.IqPid);                  //Iq闭环
			PID_Control(&MC.IdPid);                  //Id闭环			


			MC.Foc.Uq = MC.IqPid.Out;			
			MC.Foc.Ud = MC.IdPid.Out;	 
      IPack_Transform(&MC.Foc);                //反PACK变换		
		}	
    break;
		
		case STRONG_DRAG_SMO_SPEED_CURRENT_LOOP:     //强拖+滑膜 速度电流闭环         
		{
			if(step == 0)                         //第一步数据清零 
			{
				MC.IqPid.Ref = 0;                       
				MC.IdPid.Ref = 0;   
				MC.SpdPid.Integrate = 0;
				MC.Encoder.ElectricalValSet = 0;
				MC.Speed.MechanicalSpeedSetLast = 0;
				MC.TAccDec.FinishFlag = 0;
				MC.TAccDec.SpeedOut = 0;
				MC.TAccDec.StartSpeed = 0;
				MC.TAccDec.EndSpeed = 0;
        MC.SPLL.WeForeLPF = 0;
        MC.SPLL.IPart = 0;
				if(MC.Speed.MechanicalSpeedSet >= 100 / POLEPAIRS || MC.Speed.MechanicalSpeedSet <= -100 / POLEPAIRS)
				{
          step = 1;				                    //切换第二步						
				}
			}			
			else if(step == 1)                    //第二步强拖启动 启动观测 
			{
				MC.IqPid.Ref = 0;
				MC.IdPid.Ref = 5;                   //d轴强拖
				MC.Encoder.ElectricalSpdSet = MC.Speed.MechanicalSpeedSet * POLEPAIRS;
			  Electrical_Angle_Generator(&MC.Encoder);
			  Calculate_Sin_Cos(MC.Encoder.ElectricalValSet,&MC.Foc.SinVal,&MC.Foc.CosVal);					
								
				if(MC.Encoder.ElectricalSpdSet >= 0) MC.SPLL.Dir = -1;
        if(MC.Encoder.ElectricalSpdSet < 0)  MC.SPLL.Dir =  1;
				if(MC.Encoder.ElectricalValSet > 500 && MC.Encoder.ElectricalValSet < 3500)	
				{
					if(MC.SPLL.ETheta > 500 && MC.SPLL.ETheta < 3500)          //在两者线性区间内进行比较
					{
						comparecnt++;
						if(MC.Encoder.ElectricalValSet - MC.SPLL.ETheta <= MC.Encoder.EncoderValMax * 0.1f) //观测值与强拖给定值误差小于±%10
						{
							if(MC.Encoder.ElectricalValSet -MC.SPLL.ETheta >= -MC.Encoder.EncoderValMax * 0.1f)
							{
								successcnt++;							
							}
						}
					}			
				}	
				if(comparecnt >= 300)                     //多次比较
				{
					if(successcnt >= comparecnt * 0.9f)     //百分之80相似则认为观测角度正确可用
					{
            step = 2;                             //切换下一步	
            MC.IdPid.Ref = 0;						
						MC.Encoder.ElectricalValSet = 0;
					}
					successcnt = 0;
					comparecnt = 0;                    //不满足条件则清零重新判断
				}
        errortimout++;
        if(errortimout >= 40000)             //两秒还没有切换到观测器,就认为启动失败
				{
					step = 0;                          //回到第一步重新启动
					errortimout = 0;
					errorcnt++;					
				}					
			}			
			else if(step == 2)                     //第三步切观测器角度运行 
			{
				MC.Speed.SpeedCalculateCnt++;				
				if(MC.Speed.SpeedCalculateCnt >= SPEED_DIVISION_FACTOR)          //每SPEED_DIVISION_FACTOR次 执行一次速度闭环
				{
					MC.Speed.SpeedCalculateCnt = 0;
					MC.Speed.ElectricalPosThis = MC.SPLL.ETheta;                   //获取当前电角度 由高频注入得到
					Calculate_Speed(&MC.Speed);                                  	 //根据当前电角度和上次电角度计算电角速度
					MC.Speed.ElectricalSpeedLPF = MC.Speed.ElectricalSpeedRaw * MC.Speed.ElectricalSpeedLPFFactor
																			+ MC.Speed.ElectricalSpeedLPF * (1 - MC.Speed.ElectricalSpeedLPFFactor);//低通滤波	
					MC.Speed.MechanicalSpeed = MC.Speed.ElectricalSpeedLPF / MC.Encoder.PolePairs;				
					
					if(MC.Speed.MechanicalSpeedSet >= 0 && MC.Speed.MechanicalSpeedSet <= 800) MC.Speed.MechanicalSpeedSet =  800; //限制滑膜最低速度
					if(MC.Speed.MechanicalSpeedSet <= 0 && MC.Speed.MechanicalSpeedSet >= -800)MC.Speed.MechanicalSpeedSet = -800; //限制滑膜最低速度
					
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
					MC.SpdPid.Kp = MC.SpdPid.KpMin;	
					PID_Control(&MC.SpdPid);                            					 //速度闭环
					MC.IqPid.Ref = MC.SpdPid.Out;	
				}					
        Calculate_Sin_Cos(MC.SPLL.ETheta,&MC.Foc.SinVal,&MC.Foc.CosVal);
				if(runtim <  20000) 
				{
				  runtim++;
				}
				if(runtim >= 20000)                   //观测器正常运行1S后开启堵转检测
				{
					if(MC.Speed.ElectricalSpeedLPF <= 1000 && MC.Speed.ElectricalSpeedLPF >= -1000)   //外力导致转速太慢了不适合观测器运行
					{
						step = 0;                         //重新启动
						runtim = 0;
						errorcnt++;
					}				
				}
			}	
			
			MC.Foc.Iu = MC.Sample.IuReal;
			MC.Foc.Iv = MC.Sample.IvReal;			
			Clark_Transform(&MC.Foc);              //克拉克变换
		
			Pack_Transform(&MC.Foc);               //派克变换

			MC.Foc.IdLPF = MC.Foc.Id * MC.Foc.IdLPFFactor + MC.Foc.IdLPF * (1 - MC.Foc.IdLPFFactor); //Id低通滤波
			MC.Foc.IqLPF = MC.Foc.Iq * MC.Foc.IqLPFFactor + MC.Foc.IqLPF * (1 - MC.Foc.IqLPFFactor); //Iq低通滤波 

			MC.IqPid.Fbk = MC.Foc.IqLPF;
			MC.IdPid.Fbk = MC.Foc.IdLPF;			
			PID_Control(&MC.IqPid);               //Iq闭环
			PID_Control(&MC.IdPid);               //Id闭环

			MC.Foc.Uq = MC.IqPid.Out;			
			MC.Foc.Ud = MC.IdPid.Out;
	    IPack_Transform(&MC.Foc);             //反PACK变换				
      
			
			MC.SMO.Rs = MC.Identify.Rs;
			MC.SMO.Ld = MC.Identify.Ld;
			MC.SMO.Ialpha = MC.Foc.Ialpha;
			MC.SMO.Ibeta  = MC.Foc.Ibeta;
			MC.SMO.Ualpha = MC.Foc.Ualpha;
			MC.SMO.Ubeta  = MC.Foc.Ubeta;
			SMO_Calculate(&MC.SMO);	
			MC.SPLL.Ain = MC.SMO.EalphaForeLPF;
			MC.SPLL.Bin = MC.SMO.EbetaForeLPF;			
			PLL_Calculate(&MC.SPLL);			  
			Calculate_Sin_Cos(MC.SPLL.ETheta,&MC.SPLL.SinVal,&MC.SPLL.CosVal);	
		}	
    break;
			
		case HFI_CURRENT_CLOSE:                    //单电流闭环高频注入 （纯HFI不可高速运行，需要用手捏住电机控速）        
		{						
			MC.Foc.Iu = MC.Sample.IuReal;
			MC.Foc.Iv = MC.Sample.IvReal;			
			Clark_Transform(&MC.Foc);                //克拉克变换		

      Calculate_Sin_Cos(MC.HPLL.ETheta,&MC.Foc.SinVal,&MC.Foc.CosVal);		
			Pack_Transform(&MC.Foc);                 //派克变换
			
			MC.HFI.Id = MC.Foc.Id;
			MC.HFI.Iq = MC.Foc.Iq;							
			MC.HFI.Ialpha = MC.Foc.Ialpha;
			MC.HFI.Ibeta  = MC.Foc.Ibeta;  	
			HFI_Calculate(&MC.HFI);
      if(MC.HFI.NSDFlag == 0)
			{
				MC.IdPid.Ref = MC.HFI.IdRef;
			}	
      if(MC.HFI.NSDOut == 1)
			{
				MC.HFI.NSDOut = 0;
				MC.HPLL.ThetaFore += ONE_PI;
				if(MC.HPLL.ThetaFore > TWO_PI) 
				{
					MC.HPLL.ThetaFore -= TWO_PI;  	      //角度归一化  0-2Π
				}				
			}				
      Calculate_Sin_Cos(MC.HPLL.ETheta,&MC.HPLL.SinVal,&MC.HPLL.CosVal);				
			MC.HPLL.Ain = MC.HFI.IbetaOut;
			MC.HPLL.Bin = -MC.HFI.IalphaOut;
			PLL_Calculate(&MC.HPLL);							
			
			MC.IqPid.Fbk = MC.HFI.IqBase;
			MC.IdPid.Fbk = MC.HFI.IdBase;			
			PID_Control(&MC.IqPid);              //Iq闭环
			PID_Control(&MC.IdPid);              //Id闭环			
			
			MC.Foc.Uq = MC.IqPid.Out;			
			MC.Foc.Ud = MC.IdPid.Out + MC.HFI.Uin;			
      IPack_Transform(&MC.Foc);            //反PACK变换		
		}	
    break;	
		
		case HFI_SPEED_CURRENT_CLOSE:          //零低速区域无感速度闭环（纯HFI不可高速运行 针对4006无刷电机限速2500RPM）        
		{					
			MC.Speed.SpeedCalculateCnt++;  			
			if(MC.Speed.SpeedCalculateCnt >= SPEED_DIVISION_FACTOR)          //每SPEED_DIVISION_FACTOR次 执行一次速度闭环
			{
				MC.Speed.SpeedCalculateCnt = 0;
				MC.Speed.ElectricalPosThis = MC.HPLL.ETheta;                   //获取当前电角度 由高频注入得到
				Calculate_Speed(&MC.Speed);                                  	 //根据当前电角度和上次电角度计算电角速度
				MC.Speed.ElectricalSpeedLPF = MC.Speed.ElectricalSpeedRaw * MC.Speed.ElectricalSpeedLPFFactor
				                            + MC.Speed.ElectricalSpeedLPF * (1 - MC.Speed.ElectricalSpeedLPFFactor);//低通滤波	
				MC.Speed.MechanicalSpeed = MC.Speed.ElectricalSpeedLPF / MC.Encoder.PolePairs;				
				
				if(MC.Speed.MechanicalSpeedSet >=  2500 )MC.Speed.MechanicalSpeedSet =  2500; //限速（纯HFI不可高速运行）
				if(MC.Speed.MechanicalSpeedSet <= -2500 )MC.Speed.MechanicalSpeedSet = -2500; //限速（纯HFI不可高速运行）
				
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
				MC.SpdPid.Kp = MC.SpdPid.KpMin;	
				PID_Control(&MC.SpdPid);                            					 //速度闭环
				MC.IqPid.Ref = MC.SpdPid.Out;	
			}			
		
			MC.Foc.Iu = MC.Sample.IuReal;
			MC.Foc.Iv = MC.Sample.IvReal;			
			Clark_Transform(&MC.Foc);                                        //克拉克变换		

      Calculate_Sin_Cos(MC.HPLL.ETheta,&MC.Foc.SinVal,&MC.Foc.CosVal);		
			Pack_Transform(&MC.Foc);                                         //派克变换
			
			MC.HFI.Id = MC.Foc.Id;
			MC.HFI.Iq = MC.Foc.Iq;							
			MC.HFI.Ialpha = MC.Foc.Ialpha;
			MC.HFI.Ibeta  = MC.Foc.Ibeta;  	
			HFI_Calculate(&MC.HFI);
      if(MC.HFI.NSDFlag == 0)
			{
				MC.IdPid.Ref = MC.HFI.IdRef;
			}	
      if(MC.HFI.NSDOut == 1)
			{
				MC.HFI.NSDOut = 0;
				MC.HPLL.ThetaFore += ONE_PI;
				if(MC.HPLL.ThetaFore > TWO_PI) 
				{
					MC.HPLL.ThetaFore -= TWO_PI;  	                             //角度归一化  0-2Π
				}				
			}				
      Calculate_Sin_Cos(MC.HPLL.ETheta,&MC.HPLL.SinVal,&MC.HPLL.CosVal);				
			MC.HPLL.Ain = MC.HFI.IbetaOut;
			MC.HPLL.Bin = -MC.HFI.IalphaOut;
			PLL_Calculate(&MC.HPLL);							
			
			MC.IqPid.Fbk = MC.HFI.IqBase;
			MC.IdPid.Fbk = MC.HFI.IdBase;			
			PID_Control(&MC.IqPid);                                          //Iq闭环
			PID_Control(&MC.IdPid);                                          //Id闭环			
			
			MC.Foc.Uq = MC.IqPid.Out;			
			MC.Foc.Ud = MC.IdPid.Out + MC.HFI.Uin;			
      IPack_Transform(&MC.Foc);                                        //反PACK变换

//			MC.SMO.Ialpha = MC.Foc.Ialpha;
//			MC.SMO.Ibeta  = MC.Foc.Ibeta;
//			MC.SMO.Ualpha = MC.Foc.Ualpha;
//			MC.SMO.Ubeta  = MC.Foc.Ubeta;
//			SMO_Calculate(&MC.SMO);	
//			MC.SPLL.Ain = MC.SMO.EalphaForeLPF;
//			MC.SPLL.Bin = MC.SMO.EbetaForeLPF;			
//			PLL_Calculate(&MC.SPLL);			  
//			Calculate_Sin_Cos(MC.SPLL.ETheta,&MC.SPLL.SinVal,&MC.SPLL.CosVal);			
		}break;	

		case HFI_POS_SPEED_CURRENT_CLOSE:                                  //零低速HFI位置闭环        
		{
			MC.Speed.SpeedCalculateCnt++;  
			MC.Position.PosCalculateCnt++;	
			
			if(MC.Position.PosCalculateCnt >= POS_DIVISION_FACTOR)           //POS_DIVISION_FACTOR 执行一次位置闭环
			{											
				MC.Position.PosCalculateCnt = 0;			
				MC.Position.ElectricalPosThis = MC.HPLL.ETheta;			           //获取当前位置
				Calculate_Position(&MC.Position);                              //计算总位置
				MC.PosPid.Fbk = MC.Position.ElectricalPosSum;								   //反馈实际位置
				MC.PosPid.Ref = MC.Position.MechanicalPosSet * POLEPAIRS;			 //给定目标位置 
				MC.Position.MechanicalPosRaw = MC.Position.ElectricalPosSum / POLEPAIRS;
				PID_Control(&MC.PosPid);                                       //位置闭环
			}
		
			if(MC.Speed.SpeedCalculateCnt >= SPEED_DIVISION_FACTOR)          //每SPEED_DIVISION_FACTOR次 执行一次速度闭环
			{
				MC.Speed.SpeedCalculateCnt = 0;
				MC.Speed.ElectricalPosThis = MC.HPLL.ETheta;                   //获取当前电角度 由高频注入得到
				Calculate_Speed(&MC.Speed);                                  	 //根据当前电角度和上次电角度计算电角速度
				MC.Speed.ElectricalSpeedLPF = MC.Speed.ElectricalSpeedRaw * MC.Speed.ElectricalSpeedLPFFactor
				                            + MC.Speed.ElectricalSpeedLPF * (1 - MC.Speed.ElectricalSpeedLPFFactor);//低通滤波	
				MC.Speed.MechanicalSpeed = MC.Speed.ElectricalSpeedLPF / MC.Encoder.PolePairs;				
				MC.SpdPid.Ref = MC.PosPid.Out;			
				MC.SpdPid.Fbk = MC.Speed.ElectricalSpeedLPF;	 					       //反馈速度值	
				MC.SpdPid.Kp  = MC.SpdPid.KpMin * 2;	
				PID_Control(&MC.SpdPid);                            					 //速度闭环
				MC.IqPid.Ref = MC.SpdPid.Out;	
			}			
		
			MC.Foc.Iu = MC.Sample.IuReal;
			MC.Foc.Iv = MC.Sample.IvReal;			
			Clark_Transform(&MC.Foc);                                        //克拉克变换		

      Calculate_Sin_Cos(MC.HPLL.ETheta,&MC.Foc.SinVal,&MC.Foc.CosVal);		
			Pack_Transform(&MC.Foc);                                         //派克变换
			
			MC.HFI.Id = MC.Foc.Id;
			MC.HFI.Iq = MC.Foc.Iq;							
			MC.HFI.Ialpha = MC.Foc.Ialpha;
			MC.HFI.Ibeta  = MC.Foc.Ibeta;  	
			HFI_Calculate(&MC.HFI);
      if(MC.HFI.NSDFlag == 0)
			{
				MC.IdPid.Ref = MC.HFI.IdRef;
			}	
      if(MC.HFI.NSDOut == 1)
			{
				MC.HFI.NSDOut = 0;
				MC.HPLL.ThetaFore += ONE_PI;
				if(MC.HPLL.ThetaFore > TWO_PI) 
				{
					MC.HPLL.ThetaFore -= TWO_PI;  	                             //角度归一化  0-2Π
				}				
			}				
      Calculate_Sin_Cos(MC.HPLL.ETheta,&MC.HPLL.SinVal,&MC.HPLL.CosVal);				
			MC.HPLL.Ain = MC.HFI.IbetaOut;
			MC.HPLL.Bin = -MC.HFI.IalphaOut;
			PLL_Calculate(&MC.HPLL);							
			
			MC.IqPid.Fbk = MC.HFI.IqBase;
			MC.IdPid.Fbk = MC.HFI.IdBase;			
			PID_Control(&MC.IqPid);                                          //Iq闭环
			PID_Control(&MC.IdPid);                                          //Id闭环			
			
			MC.Foc.Uq = MC.IqPid.Out;			
			MC.Foc.Ud = MC.IdPid.Out + MC.HFI.Uin;			
      IPack_Transform(&MC.Foc);                                        //反PACK变换						
		}break;	
		
	}				
	MC.Foc.Ubus = MC.Sample.BusReal;		
	Calculate_SVPWM(&MC.Foc);	                                            //SVPWM	
}	


