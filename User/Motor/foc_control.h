#ifndef MOTOR_CONTROL_H
#define MOTOR_CONTROL_H

#include "public.h"
#include "pwm_ctrl.h"

typedef struct{
    float curr_a;                           //a相采样电流
    float curr_b;                           //b相采样电流
    float volt_power_flt;                   //电源电压
    float curr_kp_d;                        //pi参数
    float curr_ki_d;                        //pi参数
    float curr_kp_q;                        //pi参数
    float curr_ki_q;                        //pi参数
    uint8_t sector_num;                     //当前扇区（svpwm使用）
    Channel_Config a_positive;              //A+ 通道配置
    Channel_Config a_negative;              //A- 通道配置
    Channel_Config b_positive;              //B+ 通道配置
    Channel_Config b_negative;              //B- 通道配置
    float id_ref;                           //d轴参考电流
    float iq_ref;                           //q轴参考电流
    float id_fbk;                           //d轴反馈电流
    float iq_fbk;                           //q轴反馈电流
    float id_fbk_flt;                       //d轴反馈电流滤波
    float iq_fbk_flt;                       //q轴反馈电流滤波
    float out_d;                            //d轴pid输出
    float out_d_volt;                       //d轴输出电压
    float out_q;                            //q轴pid输出
    float out_q_volt;                       //q轴输出电压
    float alpha_val_in;                     //alpha电流输入值
    float beta_val_in;                      //beta电流输入值
    float alpha_val_out;                    //alpha电流输出值
    float beta_val_out;                     //beta电流输出值
    float out_a;                            //A相输出电压（0-1）
    float out_b;                            //B相输出电压（0-1）
    float out_c;
    float out_alpha_volt;                       //A相输出电压值 实际
    float out_beta_volt;                       //B相输出电压值 实际
    int16_t ua_cmp_val;                     //A相比较值
    int16_t ub_cmp_val;                     //B相比较值
    int16_t uc_cmp_val;
    float theta_target;                     //目标角度(0-1)电角度
    float out_d_offset;
    float out_q_offset;
}Motor_Control;


extern Motor_Control motor_control;

void Pwm_Init(Motor_Control *p);
void Motor_Control_Init(Motor_Control *p);
void Step_Vector_Curr_Control(Motor_Control *p);
#endif

