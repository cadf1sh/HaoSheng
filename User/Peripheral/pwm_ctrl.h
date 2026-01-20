#ifndef PERIPHERAL_INIT_H
#define PERIPHERAL_INIT_H

#include "public.h"

typedef struct{
    TIM_HandleTypeDef *timer;
    uint32_t channel;
    uint16_t cmp_val;
}Channel_Config;

typedef struct{
    Channel_Config a_positive;              //A+ 通道配置
    Channel_Config a_negative;              //A- 通道配置
    Channel_Config b_positive;              //B+ 通道配置
    Channel_Config b_negative;              //B- 通道配置
}All_Channel_Config;

extern All_Channel_Config all_channel;


void All_Pwm_Channel_Init(void);
void PWM_OUT_ON(All_Channel_Config *p);
void PWM_OUT_OFF(All_Channel_Config *p);

#endif

