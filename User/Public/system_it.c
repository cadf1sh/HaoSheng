/**
  ******************************************************************************
  * 文件名称: 
  * 作    者: √
  * 版    本: V1.0
  * 编写日期: 
  * 功    能: ADC中断，系统运行流程控制
  ******************************************************************************
  */
  
/* 包含头文件 ----------------------------------------------------------------*/
#include "system_it.h"
#include "motor_task.h"

int16_t spend_time = 0;             //执行时间

/**
  * 函数功能:ADC注入中断回调
  * 输入参数:
  * 返回参数:
  * 说    明: 
  */
void HAL_ADCEx_InjectedConvCpltCallback(ADC_HandleTypeDef *hadc){

    if(hadc ->Instance == ADC2){
        TIM6->CNT = 0;
        Motor_Run_Task();
        spend_time = TIM6 -> CNT;
        
    }
}





