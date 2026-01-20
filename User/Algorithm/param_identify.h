#ifndef PARAM_IDENTIFY_H
#define PARAM_IDENTIFY_H

#include "public.h"


typedef enum{
    FIRST = 1,
    SECOND,
    THIRD,
    LAST
}Identify_Stage;

typedef struct{
    Identify_Stage ident_stage;                 //辨识阶段
    float volt_power_flt;                       //电源电压滤波值
    float ident_curr;                           //母线电流设定值
    float curr_a;
    float curr_b;
    float d_volt;
    float curr_d;
    float curr_d_flt;
    float curr_q;
    float curr_q_flt;
    float motor_power;
    float fir_volt;
    float fir_curr;
    float sec_volt;
    float sec_curr;
    float motor_r;
    float motor_ld;
    float motor_lq;
    float curr_kp;                              //电流环kp
    float curr_ki;                              //电流环ki
    float motor_rl;                             //motor_r/motor_l
    float ident_theta;                          //辨识阶段给定的角度(0-1)
    uint8_t pwm_sw;
    uint16_t wait_time;
    uint8_t motor_l_cal_cnt;
    uint8_t motor_l_cal_flag;
}Param_Identify;

extern Param_Identify param_ident;

void Param_Ident_Init(Param_Identify *p);
void Param_Ident(Param_Identify *p);
void Encoder_Adjust_Run(void);
#endif

