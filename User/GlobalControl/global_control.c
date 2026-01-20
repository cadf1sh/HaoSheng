
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

#include "global_control.h"
#include "motor_system.h"
#include "led_task.h"
#include "lcd_task.h"
#include "lcd_drv.h"
#include "usart_task.h"

extern volatile u16 LedTaskTim;
extern volatile u16 LcdTaskTim;
extern volatile u16 UsartTaskTim;

/**
  * 函数功能: 全局初始化
  * 输入参数:
  * 返回参数:
  * 说    明: 
  */
void Global_Init(void)
{
	HAL_Delay(500);
  LCD_Init();
	
	Motor_System_Init();
	
	HAL_TIM_Encoder_Start(&htim3, TIM_CHANNEL_ALL);  //启动编码器接口 

	__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_1,0);   //设置初始占空比
	__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_2,0);   //设置初始占空比
	__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_3,0);	 //设置初始占空比
	__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_4,0);	 //设置初始占空比
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);        //开启对应通道PWM输出
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);        //开启对应通道PWM输出
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);        //开启对应通道PWM输出
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_4);        //开启对应通道PWM输出
	HAL_TIM_Base_Start_IT(&htim1);	                 //开启定时器中断
	
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_SET); //使能SD1
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, GPIO_PIN_SET); //使能SD2
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_2, GPIO_PIN_SET); //使能SD3
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0, GPIO_PIN_SET); //使能SD4
}

/**
  * 函数功能: 主循环
  * 输入参数:
  * 返回参数:
  * 说    明: 
  */
void Global_Loop(void)
{
	Led_Task();
	Lcd_Task();	
  Usart_Task();	
}

/**
  * 函数功能: 定时器中断回调函数
  * 输入参数:
  * 返回参数:
  * 说    明: 
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	if(htim->Instance == htim1.Instance)                      //20KHZ   50US
	{	
	  HAL_ADCEx_InjectedStart_IT(&hadc2);	                  	//开启ADC注入通道中断
		LedTaskTim++;                                           //LED任务计时
		LcdTaskTim++;	                                          //LCD任务计时
		UsartTaskTim++;		                                      //串口任务计时
	}	
}

/**
  * 函数功能: ADC注入中断回调函数
  * 输入参数:
  * 返回参数:
  * 说    明: 20KHZ频率即50US执行一次
  */
void HAL_ADCEx_InjectedConvCpltCallback(ADC_HandleTypeDef *hadc)
{		
	MC.Sample.IuRaw = ADC2->JDR1;          	   //获取相电流
	MC.Sample.IwRaw = ADC2->JDR2;              //获取相电流
	MC.Sample.BusRaw = ADC2->JDR3;          	 //获取母线电压
	MC.Encoder.EncoderVal = TIM3->CNT;         //获取编码器值		
 	MC.Speed.MechanicalSpeedSet  =  ADC2->JDR4;//使用波轮电位器给电机目标转速（速度闭环模式下）
	MC.Position.MechanicalPosSet = -ADC2->JDR4;//使用波轮电位器给电机目标位置（位置闭环模式下）
	
 	Motor_System_Run();                        //电机系统运行
            
	__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_1,MC.Foc.DutyCycleA);     //更新PWM比较值             
	__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_2,MC.Foc.DutyCycleB);     //更新PWM比较值
	__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_3,MC.Foc.DutyCycleC); 		 //更新PWM比较值
	__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_4,2149);		               //更新PWM比较值
}





