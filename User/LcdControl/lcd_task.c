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
}



