#include "keyscan.h"
 
extern u32 global_actuator_absolute_pos;              //执行机构绝对位置
extern u32 global_actuator_pos_zero;                  //执行机构位置零点
extern u8 showflag;

/**
  * 函数功能: 按键扫描函数
  * 输入参数: 
  * 返 回 值:
  * 说    明：
  */
void Key_Task(void)
{
	if(KEY1 == 0)
	{
		HAL_Delay(500);
	  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_3, GPIO_PIN_RESET);
	}
	
	if(KEY2 == 0)
	{
    global_actuator_pos_zero = global_actuator_absolute_pos;
	}

//	if(KEY3 == 0)
//	{
//	  if(showflag == 0)showflag = 1;
//		else showflag = 0;	
//	}
}

/*********************************END OF FILE***********************************/

