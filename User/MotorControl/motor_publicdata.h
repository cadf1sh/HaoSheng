#ifndef __MOTOR_PUBLICDATA_H
#define __MOTOR_PUBLICDATA_H

#include "main.h"
#include "foc_drv.h"
#include "pid_drv.h"
#include "math_drv.h"
#include "speed_drv.h"
#include "sample_drv.h"
#include "encoder_drv.h"
#include "position_drv.h"
#include "observer_drv.h"

#define LOW_RESITOR        4.7f     //母线电压检测下端电阻
#define HIGH_RESITOR       100.0f   //母线电压检测上端电阻
#define SAMPLING_RESITOR   0.005f   //相电流采样电阻
#define MAGNIFICATION      10.0f    //采样放大倍数
#define ADC_RESOLUTION     4096.0f  //ADC分辨率
#define ADC_VREF           3.3f     //ADC基准电压
#define PWM_LIMLT          7800     //限制最大占空比
#define PWM_CYCLE          8500     //PWM周期占空比
#define TS                 0.00005f //FOC执行间隔

#define VBUS_FACTOR            ((ADC_VREF / ADC_RESOLUTION) / (LOW_RESITOR / (LOW_RESITOR+HIGH_RESITOR))) // 母线电压计算系数
#define PHASE_CURRENT_FACTOR   ((ADC_VREF / ADC_RESOLUTION) / MAGNIFICATION / SAMPLING_RESITOR)           // 相电流计算系数

/****************参数辨识状态*******************/  
#define RESISTANCE_IDENTIFICATION           0X00  // 相电阻识别
#define INDUCTANCE_IDENTIFICATION           0X01  // 相电感识别

/******************运行状态*********************/
#define ADC_CALIB                           0X00  // ADC校准
#define MOTOR_STOP                          0X01  // 停机
#define MOTOR_ERROR                         0X02  // 故障报错
#define MOTOR_IDENTIFY                      0X03  // 参数辨识
#define MOTOR_SENSORUSE                     0X04  // 有感控制
#define MOTOR_SENSORLESS                    0X05  // 无感控制

/****************有感运行模式*******************/
#define ENCODER_CALIB                       0X00  // 编码器校准
#define CURRENT_OPEN_LOOP		                0X01  // 电流开环
#define CURRENT_CLOSE_LOOP	                0X02  // 电流闭环
#define SPEED_CURRENT_LOOP                  0X03  // 速度闭环
#define POS_SPEED_CURRENT_LOOP	            0X04  // 位置闭环

/****************无感运行模式*******************/
#define STRONG_DRAG_CURRENT_OPEN            0X05  // 电流开环强拖
#define STRONG_DRAG_CURRENT_CLOSE		        0X06  // 电流闭环强拖
#define STRONG_DRAG_SMO_SPEED_CURRENT_LOOP  0X07  // 强拖切滑膜速度电流闭环
#define HFI_CURRENT_CLOSE                   0X08  // 电流闭环高频注入（测试HFI角度收敛效果）
#define HFI_SPEED_CURRENT_CLOSE             0X09  // 高频注入速度电流闭环
#define HFI_POS_SPEED_CURRENT_CLOSE         0X0A  // 高频注入位置速度电流闭环


#define HALF_PI  1.5707963f
#define ONE_PI   3.1415926f
#define TWO_PI   6.2831853f

#define CW  0                        //编码器顺时针方向
#define CCW 1                        //编码器逆时针方向
                       
#define ENCODER_LINE 1024            //编码器线数
#define PUL_MAX (4*ENCODER_LINE -1)  //单圈脉冲最大值

#define PUL_ANGLE_FACTOR (4095.0f/PUL_MAX)  //角度系数

#define POLEPAIRS   7                //默认极对数
#define ACCELERATION 3               //默认加速度

#define SPEED_DIVISION_FACTOR  2     //速度环分频系数
#define POS_DIVISION_FACTOR    4     //位置环分频系数

typedef struct
{	
  u8    RunState;          // 运行状态
	u8    RunMode;           // 运行模式
}MOTOR_STRUCT;

typedef struct
{	
	u8    Flag;
	u8    State;	
	u8    EndFlag;           // 辨识完成标志	
	u16   Count;             // 计数
	u16   WaitTim;           // 等待时间
	float Rs;                // 相电阻
	float Ls;                // 相电感
	float Lq;                // q轴电感
	float Ld;                // d轴电感	
	float Flux;              // 磁链
	float LsSum;             // 电感累计值
	float CurMax;            // 最大识别电流
	float CurSum;            // 电流累计值
	float CurAverage[2];     // 电流平均值
	float VoltageSet[2];     // 电压给定值
}IDENTIFY_STRUCT;

typedef struct
{
	MOTOR_STRUCT   					  Motor;            
  IDENTIFY_STRUCT	          Identify;         
	ENCODER_STRUCT 					  Encoder;                                                            
	SAMPLE_STRUCT 					  Sample;                                  
	FOC_STRUCT   					    Foc;	                                  	                                                                 
	PID_STRUCT    						IqPid;  	                              
	PID_STRUCT    					  IdPid;
	PID_STRUCT      					SpdPid; 
	PID_STRUCT     					  PosPid;	
	SPEED_STRUCT  					  Speed; 
	TSHAPEDACCDEC_STRUCT      TAccDec;		
	POSITION_STRUCT 				 	Position;      		
	SMO_STRUCT                SMO;
	HFI_STRUCT                HFI;
  PLL_STRUCT	              SPLL;	         
  PLL_STRUCT	              HPLL;          
}MOTORCONTROL_STRUCT;

extern MOTORCONTROL_STRUCT MC;

void Motor_Struct_Init(void);


//typedef  uint8_t          u8;
//typedef  uint16_t         u16;
//typedef  uint32_t         u32;
//typedef  uint64_t         u64;

//typedef  int8_t           s8;
//typedef  int16_t          s16;
//typedef  int32_t          s32;
//typedef  signed long long s64;

#endif 


