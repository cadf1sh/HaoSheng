/**
  ******************************************************************************
  * 文件名程: lcd_task.c
  * 作    者: 浩然
  * 版    本: V1.0
  * 编写日期: 2024-03-29
  * 功    能: 定时进行lcd的显示任务
  ******************************************************************************
  */
/* 包含头文件 ----------------------------------------------------------------*/
#include "lcd_task.h"
#include "lcd_drv.h"

volatile u16 LcdTaskId = 10;
volatile u16 LcdTaskTim = 0;

float machinebutton = 0;
float IIA = 0;
float IIB = 0;
s32 Encod = 0;

/**
  * 函数功能: 定时进行lcd的显示任务
  * 输入参数:
  * 返 回 值: 
  * 说    明:
  */
void Lcd_Task(void)
{
	switch(LcdTaskId)
	{
		case 10:
		{
			if(LcdTaskTim>=2000)        //100ms
			{
				LcdTaskTim = 0;
				LcdTaskId = 20;
			}
		}
		break;
		
		case 20:
		{ 

			LcdTaskId = 10; 
		}
		break;
		
    default:
      break;	
	}
//	LCD_ShowFloatNum1(0,16*0,machinebutton,5,BLACK,WHITE,16);
//	LCD_ShowFloatNum1(0,16*1,IIA,5,BLACK,WHITE,16);
//	LCD_ShowFloatNum1(0,16*2,IIB,5,BLACK,WHITE,16);
//	LCD_ShowIntNum(0,16*3,Encod,5,BLACK,WHITE,16);
}



