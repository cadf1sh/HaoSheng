#ifndef __SPEED_DRV_H
#define __SPEED_DRV_H

#include "main.h"

typedef struct
{
	s16   CurrentSpeedDir;
	u16   SpeedCalculateCnt;          // 速度计算计数	
	u16   ElectricalValMax;           // 电角度最大值
	s32   ElectricalPosThis;          // 本次电角位置
	s32   ElectricalPosLast;          // 上次电角位置
	s32   ElectricalPosChange;        // 单位时间位移
	float ElectricalSpeedFactor;      // 电角速度系数
	float ElectricalSpeedRaw;         // 原始电角速度
	
	float ElectricalSpeedLPF;         // 原始电角速度滤波值	
	float ElectricalSpeedLPFFactor;   // 原始电角速度滤波系数	
	
	float MechanicalSpeed;	          // 滤波后机械速度
  float MechanicalSpeedSet;         // 目标机械速度
  float MechanicalSpeedSetLast;     // 上次目标机械速度		
}SPEED_STRUCT;

typedef struct
{  
  float StartSpeed;   //初始速度    
	float EndSpeed;     //末速度
	float AccSpeed;     //加速度
	float SumSpeed;     //速度增量
	float DecSpeed;     //减速度
	float SpeedOut;     //输出目标速度
	u8    FinishFlag;   //加减速完成标志
}TSHAPEDACCDEC_STRUCT;


void Calculate_Speed(SPEED_STRUCT *p);
void T_Shaped_Acc_Dec(TSHAPEDACCDEC_STRUCT *p);


#endif 


