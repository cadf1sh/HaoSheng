/**
  ******************************************************************************
  * 文件名称: 
  * 作    者: √
  * 版    本: V1.0
  * 编写日期: 
  * 功    能: pwm通道配置
  ******************************************************************************
  */
  
/* 包含头文件 ----------------------------------------------------------------*/
#include "pwm_ctrl.h"

All_Channel_Config all_channel;

/**
  * 函数功能:PWM通道初始化
  * 输入参数:
  * 返回参数:
  * 说    明: 后续会赋值到motor_control结构体
  */
void All_Pwm_Channel_Init(void){
    all_channel.a_positive.timer = &htim1;
    all_channel.a_positive.channel = TIM_CHANNEL_1;
    all_channel.a_negative.timer = &htim1;
    all_channel.a_negative.channel = TIM_CHANNEL_2;
    all_channel.b_positive.timer = &htim1;
    all_channel.b_positive.channel = TIM_CHANNEL_3;
    all_channel.b_negative.timer = &htim1;
    all_channel.b_negative.channel = TIM_CHANNEL_4;
    
}

/**
  * 函数功能:开启PWM输出
  * 输入参数:
  * 返回参数:
  * 说    明: 
  */
void PWM_OUT_ON(All_Channel_Config *p){
    HAL_TIM_PWM_Start(p->a_positive.timer, p->a_positive.channel);
    //HAL_TIMEx_PWMN_Start(p->a_positive.timer, p->a_positive.channel);
    HAL_TIM_PWM_Start(p->a_negative.timer, p->a_negative.channel);
    //HAL_TIMEx_PWMN_Start(p->a_negative.timer, p->a_negative.channel);
    HAL_TIM_PWM_Start(p->b_positive.timer, p->b_positive.channel);
    //HAL_TIMEx_PWMN_Start(p->b_positive.timer, p->b_positive.channel);
    HAL_TIM_PWM_Start(p->b_negative.timer, p->b_negative.channel);
    //HAL_TIMEx_PWMN_Start(p->b_negative.timer, p->b_negative.channel);
//    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_SET); 
//    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, GPIO_PIN_SET);
//    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_2, GPIO_PIN_SET);
//    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0, GPIO_PIN_SET);

}

/**
  * 函数功能:关闭PWM输出
  * 输入参数:
  * 返回参数:
  * 说    明: 
  */
void PWM_OUT_OFF(All_Channel_Config *p){
    HAL_TIM_PWM_Stop(p->a_positive.timer, p->a_positive.channel);
    //HAL_TIMEx_PWMN_Stop(p->a_positive.timer, p->a_positive.channel);
    HAL_TIM_PWM_Stop(p->a_negative.timer, p->a_negative.channel);
    //HAL_TIMEx_PWMN_Stop(p->a_negative.timer, p->a_negative.channel);
    HAL_TIM_PWM_Stop(p->b_positive.timer, p->b_positive.channel);
    //HAL_TIMEx_PWMN_Stop(p->b_positive.timer, p->b_positive.channel);
    HAL_TIM_PWM_Stop(p->b_negative.timer, p->b_negative.channel);
    //HAL_TIMEx_PWMN_Stop(p->b_negative.timer, p->b_negative.channel);
//    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_RESET); 
//    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, GPIO_PIN_RESET);
//    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_2, GPIO_PIN_RESET);
//    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0, GPIO_PIN_RESET);
}



