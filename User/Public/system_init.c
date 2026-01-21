/**
  ******************************************************************************
  * 文件名称: 
  * 作    者: √
  * 版    本: V1.0
  * 编写日期: 
  * 功    能: 系统初始化
  ******************************************************************************
  */
  
/* 包含头文件 ----------------------------------------------------------------*/
#include "system_init.h"
#include "encoder.h"
#include "motion_control.h"
#include "current_sample.h"
#include "param_identify.h"
#include "foc_control.h"
#include "motor_run_mode.h"
#include "pwm_ctrl.h"
#include "position_mode_test.h"
#include "ele_angle_adjust.h"

/**
  * 函数功能:外设初始化，相关句柄赋值给对应结构体
  * 输入参数:
  * 返回参数:
  * 说    明: 
  */
void All_Peripheral_Init(void){
    
    /*ADC采样初始化*/
    current_sample.adc_a = &hadc1;
    current_sample.adc_b = &hadc2;
    Adc_Init(&current_sample);
    
    HAL_Delay(100);
    
    /*ABZ编码器初始化(未开启读取)*/
    encoder.cnt_timer = &htim2;
    encoder.tim_timer = &htim3;
    encoder_position.cnt_timer = &htim2;
    Encoder_Init(&encoder);
    
    motion_control.run_mode = INT_SPEC;
    
    /*PWM通道初始化(未开启输出)*/
    All_Pwm_Channel_Init();
    Pwm_Init(&motor_control);
    
    /*运行时间监测定时器*/
    HAL_TIM_Base_Start(&htim6);
    HAL_TIM_Base_Start_IT(&htim6);
    
}

/**
  * 函数功能:系统各结构体初始化赋值
  * 输入参数:
  * 返回参数:
  * 说    明: 
  */
void All_Variable_Init(void){

    System_Run_Param_Init(&run_param);
    
    Micro_Varible_Init(&micro_varible);
    
    Current_Sample_Init(&current_sample);
    
    Param_Ident_Init(&param_ident);
    
    Encoder_Postion_Cal_Init(&encoder_position);
    Speed_Feedback_Cal_Init(&encoder);
    
    Position_Loop_Cal_Init(&position_loop);
    Speed_Loop_Cal_Init(&speed_loop);
    
    Position_Motion_Comb_Init(&motion_comb);
    
    Position_Control_Init(&position_control);
    Motion_Control_Init(&motion_control);
    
    Motor_Control_Init(&motor_control);
    
    /*控制编码器非线性校准是否执行（校准程序运行一次后会保存在片内flash中，后续不需要运行校准）*/
    if(run_param.reset_angle_data == 0){            //不运行校准
        ele_angle_adjust.adjust_enable = 0;
        run_param.enc_adjust_flag = 1;
    }else{
        ele_angle_adjust.adjust_enable = 1;
        run_param.enc_adjust_flag = 0;
    }
    /*变量初始化完成，开启PWM输出*/
    PWM_OUT_ON(&all_channel);

}

/**
  * 函数功能: 自定义实现编码器原始数据读取（函数原型使用weak修饰，此处为重载，注释此函数则使用默认模式）
  * 输入参数: 
  * 返回参数: 编码器原始位置信息
  * 说    明: 系统默认使用ABZ模式读取TIM2计数值，此函数为读取VCE2755Q的14位绝对值数据
  */
uint16_t Encoder_Data_Get(void){
    uint16_t angle_reg = 0x83;
    uint16_t reg_org_data[2] = {0};
    HAL_GPIO_WritePin(SPI2_NSS_GPIO_Port, SPI2_NSS_Pin, GPIO_PIN_RESET);
    HAL_SPI_TransmitReceive(&hspi2, (uint8_t*)&angle_reg, (uint8_t *)&reg_org_data[0], 3, HAL_MAX_DELAY);
    HAL_GPIO_WritePin(SPI2_NSS_GPIO_Port, SPI2_NSS_Pin, GPIO_PIN_SET);
    uint8_t angle_data_h = reg_org_data[0] >> 8 & 0xFF;
    uint8_t angle_data_l = reg_org_data[1] & 0xFF;
    return ((angle_data_h << 8 | (angle_data_l & 0xFC))& 0x7FFF) >> 1;
}

