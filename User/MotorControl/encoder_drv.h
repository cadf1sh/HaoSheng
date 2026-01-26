#ifndef __ENCODER_DRV_H
#define __ENCODER_DRV_H

#include "main.h"

typedef struct
{
	u8    Dir;                        // 编码器方向	
  u8    Type;                       // 编码器类型	
	u8    PolePairs;                  // 转子极对数
	s32   EncoderVal;                 // 编码器原始数据
	s32   EncoderValMax;              // 编码器最大原始值
	s32   ElectricalVal;              // 电气角度
	u16   CalibFlag;                  // 校准完成标志
	u16   CalibOffset;                // 转子零位偏差	
	float ElectricalSpdSet;           // 给定的电角速度    	
	float ElectricalValSet;           // 给定的电角度	
}ENCODER_STRUCT; 

void Electrical_Angle_Generator(ENCODER_STRUCT *p);
void Calculate_Encoder_Data(ENCODER_STRUCT *p);

#endif 


