#ifndef CURRENT_SAMPLE_H
#define CURRENT_SAMPLE_H

#include "public.h"

typedef struct{
    ADC_HandleTypeDef *adc_a;                        //A相采样ADC
    ADC_HandleTypeDef *adc_b;                        //B相采样ADC
    uint16_t adc_channel[4];                        //ADC原始数据
    float volt_a;                                   //a相电压值
    float volt_b;                                   //b相电压值
    float volt_a_ref;                               //a相实际参考电压值
    float volt_b_ref;                               //b相实际参考电压值
    float curr_a;                                   //a相电流值
    float curr_b;                                   //b相电流值
    float curr_c;
    float volt_power;                               //电源电压原始值
    float volt_power_flt;                           //电源电压滤波值
    float volt_power_inv;                           //电源电压倒数
}Current_Sample;

typedef struct{
    float resistor_other;                           //分压电阻阻值
    float B_value;                                  //B值
    float temperature_ref;                          //参考温度值
    float resistor_ref;                             //参考温度下的阻值
    float curr_resistor;                            //当前阻值
    float ln_value;                                 //自然对数值
    uint16_t adc_value;                             //adc采样原始值
    float curr_temp;
}Rtc_Temperature;

extern Current_Sample current_sample;                   //电流采样
extern Rtc_Temperature rtc_temp;                        //热敏电阻

void Adc_Init(Current_Sample *p);
void Current_Sample_Init(Current_Sample *p);
void Adc_Value_Get(Current_Sample *p);
void Voltage_Value_Cal(Current_Sample *p);
void Voltage_Ref_Cal(Current_Sample *p);
void Current_Cal(Current_Sample *p);

#endif

