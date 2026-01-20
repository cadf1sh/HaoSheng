/**
  ******************************************************************************
  * 文件名称: 
  * 作    者: √
  * 版    本: V1.0
  * 编写日期: 
  * 功    能: 电流采样相关，包括电源电压、偏置计算等
  ******************************************************************************
  */
  
/* 包含头文件 ----------------------------------------------------------------*/
#include "current_sample.h"

Current_Sample current_sample;                  //电流采样
Rtc_Temperature rtc_temp;

/**
  * 函数功能:ADC初始化
  * 输入参数:
  * 返回参数:
  * 说    明: 
  */
void Adc_Init(Current_Sample *p){
    HAL_ADCEx_Calibration_Start(p->adc_a, ADC_SINGLE_ENDED);
    HAL_ADCEx_Calibration_Start(p->adc_b, ADC_SINGLE_ENDED);
    HAL_Delay(100);
    HAL_ADC_Start_DMA(p->adc_a, (uint32_t *)&p->adc_channel[2], 2);
    
    HAL_ADCEx_InjectedStart_IT(p->adc_a);
    HAL_ADCEx_InjectedStart_IT(p->adc_b);
}

/**
  * 函数功能:热敏电阻温度采集初始化
  * 输入参数:Rtc_Temperature
  * 返回参数:
  * 说    明:热敏电阻参数设置
  */
void Rtc_Temperature_Init(Rtc_Temperature *p){
    p->resistor_other = 10000.0f;
    p->B_value = 3434.0f;                                   //B值 (25℃/50℃)3380K  (25℃/85℃)3434K  (25℃/100℃)3455K
    p->temperature_ref = 298.15f;                           //参考温度值  25℃ + 273.15
    p->resistor_ref = 10000.0f;                             //参考温度下的阻值
    p->curr_resistor = 0;                                   //当前阻值
    p->ln_value = 0;                                        //自然对数值
    p->adc_value = 0;
    p->curr_temp = 0;
}

/**
  * 函数功能:结构体初始化
  * 输入参数:
  * 返回参数:
  * 说    明: 
  */
void Current_Sample_Init(Current_Sample *p){
    p->volt_a = 0;
    p->volt_b= 0;
    p->volt_a_ref= 0;
    p->volt_b_ref= 0;
    p->curr_a= 0;
    p->curr_b= 0;
    p->volt_power= 0;
    p->volt_power_flt= 0;
    p->volt_power_inv= 0;
    Rtc_Temperature_Init(&rtc_temp);
}

/**
  * 函数功能:ADC寄存器值获取
  * 输入参数:
  * 返回参数:
  * 说    明: 
  */
void Adc_Value_Get(Current_Sample *p){
    p->adc_channel[0] = p->adc_a->Instance->JDR1;       //A相
    p->adc_channel[1] = p->adc_b->Instance->JDR1;       //B相
}

/**
  * 函数功能:热敏电阻温度换算
  * 输入参数:
  * 返回参数:
  * 说    明:
  */
void Rtc_Temperature_Cal(Rtc_Temperature *p){
    p->curr_resistor = p->resistor_other * (4096.0f - p->adc_value) / p->adc_value;
    p->ln_value = logf(p->curr_resistor / p->resistor_ref);
    p->curr_temp = 1.0f / (1.0f / p->temperature_ref + 1.0f / p->B_value * p->ln_value) - 273.15f;
}

/**
  * 函数功能:电压计算
  * 输入参数:
  * 返回参数:
  * 说    明: 
  */
void Voltage_Value_Cal(Current_Sample *p){
    static uint16_t power_cnt = 0;
    p->volt_a = p->adc_channel[0] * 3.3f / 4096.0f;
    p->volt_b = p->adc_channel[1] * 3.3f / 4096.0f;
    p->volt_power = p->adc_channel[2] * 3.3f / 4096.0f * 16.0f;
    
    p->volt_power_flt += (0.01f * (p->volt_power - p->volt_power_flt));
    p->volt_power_inv = 1.0f / p->volt_power_flt;
    run_param.power_volt_inv = p->volt_power_inv;
    if(p->volt_power_flt < VOLT_MIN || p->volt_power_flt > VOLT_MAX){
        power_cnt++;
        if(power_cnt > 5000){
            run_param.sys_status  = FAULT;
            run_param.sys_err_code = POWER_ERR;
        }
    }else{
        power_cnt = 0;
    }
    rtc_temp.adc_value = p->adc_channel[3];
    Rtc_Temperature_Cal(&rtc_temp);
    run_param.mos_temperature = rtc_temp.curr_temp;
}

/**
  * 函数功能:偏置电压校验计算
  * 输入参数:
  * 返回参数:
  * 说    明: 
  */
void Voltage_Ref_Cal(Current_Sample *p){
    static uint16_t prepare_cnt = 0;
    prepare_cnt++;
    if(prepare_cnt == 100){//舍弃前100次采样数据
        p->volt_a_ref = 0;
        p->volt_b_ref = 0;
    }
    p->volt_a_ref += p->volt_a;
    p->volt_b_ref += p->volt_b;
    
    if(prepare_cnt == 199){
        p->volt_a_ref = p->volt_a_ref * 0.01f;
        p->volt_b_ref = p->volt_b_ref * 0.01f;
        if(fabsf(p->volt_a_ref - REF_VOLT) > 0.2f || fabsf(p->volt_b_ref - REF_VOLT) > 0.2f){           //采样基准电压偏移±0.2V报错
            run_param.sys_status  = FAULT;
            run_param.sys_err_code = REF_ERR;
        }else{
            prepare_cnt = 0;
            run_param.sys_status = IDENT;
        }
    }
}

/**
  * 函数功能:电流计算，软件过流保护
  * 输入参数:
  * 返回参数:
  * 说    明: 
  */
void Current_Cal(Current_Sample *p){
    static uint8_t over_curr_cnt = 0;
    p->curr_a = (p->volt_a - p->volt_a_ref) / 20.0f * 100.0f;
    p->curr_b = (p->volt_b - p->volt_b_ref) / 20.0f * 100.0f;

    if(fabsf(p->curr_a) > CURR_MAX || fabsf(p->curr_b) > CURR_MAX){
        over_curr_cnt++;
        if(over_curr_cnt > 5){//过流5个控制周期
            run_param.sys_status  = FAULT;
            run_param.sys_err_code = CURR_ERR;
        }
    }else {
        over_curr_cnt = 0;
    }

}

