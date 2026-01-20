/**
  ******************************************************************************
  * 文件名程: led_task.c
  * 作    者: 浩然
  * 版    本: V1.0
  * 编写日期: 2024-03-29
  * 功    能: 进行led的周期闪烁，表示工作
  ******************************************************************************
  */
/* 包含头文件 ----------------------------------------------------------------*/
#include "led_task.h"

volatile u16 LedTaskId = 10;
volatile u16 LedTaskTim = 0;

/**
  * 函数功能: 进行LED的工作闪烁流程
  * 输入参数:
  * 返 回 值: 
  * 说    明:
  */
void Led_Task(void)
{
	switch(LedTaskId)
	{
		case 10:
		{
			if(LedTaskTim>=2000)        //100ms
			{
				LedTaskTim = 0;
				LedTaskId = 20;
			}
		}
		break;
		
		case 20:
		{ 
      HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_15);
			LedTaskId = 10; 
		}
		break;
		
    default:
      break;			
	}
}

