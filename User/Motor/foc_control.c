/**
  ******************************************************************************
  * 文件名称: 
  * 作    者: √
  * 版    本: V1.0
  * 编写日期: 
  * 功    能: 电流环运算
  ******************************************************************************
  */
  
/* 包含头文件 ----------------------------------------------------------------*/
#include "foc_control.h"
#include "pid_control.h"
#include "transform.h"

Motor_Control motor_control;                    //FOC

Pid_Control_Handle pid_handle_d;
Pid_Control_Handle pid_handle_q;
Pid_Control_Handle pid_handle_alpha;
Pid_Control_Handle pid_handle_beta;

Pack_Handle pack_handle;

/**
  * 函数功能:PWM通道初始化
  * 输入参数:
  * 返回参数:
  * 说    明: 
  */
void Pwm_Init(Motor_Control *p){
    
    p->a_positive = all_channel.a_positive;
    p->a_negative = all_channel.a_negative;
    p->b_positive = all_channel.b_positive;
    p->b_negative = all_channel.b_negative;
    
    __HAL_TIM_SetCompare(p->a_positive.timer, p->a_positive.channel, PWM_HALF_PRD);
    __HAL_TIM_SetCompare(p->a_negative.timer, p->a_negative.channel, PWM_HALF_PRD);
    __HAL_TIM_SetCompare(p->b_positive.timer, p->b_positive.channel, PWM_HALF_PRD);
    __HAL_TIM_SetCompare(p->b_negative.timer, p->b_negative.channel, PWM_HALF_PRD);
}


/**
  * 函数功能:结构体初始化
  * 输入参数:
  * 返回参数:
  * 说    明: 
  */
void Motor_Control_Init(Motor_Control *p){
    p->curr_a = 0;
    p->curr_b = 0;
    p->volt_power_flt = 0;
    p->curr_kp_d = run_param.curr_kp;
    p->curr_ki_d = run_param.curr_ki;
    p->curr_kp_q = run_param.curr_kp;
    p->curr_ki_q = run_param.curr_ki;
    p->id_ref = 1.0;
    p->iq_ref = 0;
    p->id_fbk = 0;
    p->iq_fbk = 0;
    p->id_fbk_flt = 0;
    p->iq_fbk_flt = 0;
    p->out_d = 0;
    p->out_d_volt = 0;
    p->out_q = 0;
    p->out_q_volt = 0;
    p->out_a = 0;
    p->out_b = 0;
    p->out_alpha_volt = 0;
    p->out_beta_volt = 0;
    p->ua_cmp_val = 0;
    p->ub_cmp_val = 0;
    p->uc_cmp_val = 0;
    p->theta_target = 0;
    p->out_d_offset = 0;
    p->out_q_offset = 0;
    
    pid_handle_d.integral_lim = 1.0f;//3
    pid_handle_q.integral_lim = 1.0f;
    pid_handle_alpha.integral_lim = 1.0f;
    pid_handle_beta.integral_lim = 1.0f;
    pid_handle_d.out_lim = 1.0f;//25
    pid_handle_q.out_lim = 1.0f;
    pid_handle_alpha.out_lim = 1.0f;
    pid_handle_beta.out_lim = 1.0f;
    Pid_Control_Init(&pid_handle_d);
    Pid_Control_Init(&pid_handle_q);
    Pid_Control_Init(&pid_handle_alpha);
    Pid_Control_Init(&pid_handle_beta);
}

/**
  * 函数功能:两相SVPWM
  * 输入参数:
  * 返回参数:
  * 说    明: 
  */
void Two_Phase_Space_Vector_Transform(Motor_Control *p){
    uint8_t a = p->alpha_val_out > 0 ? 1 : 0;
    uint8_t b = p->beta_val_out > 0 ? 1 : 0;
    
    p->sector_num = 2 * b + a +1;
    
    float Ug = p->alpha_val_out;
    float Uh = p->beta_val_out;
    
    float Taon = 0;
    float Tbon = 0;
    float Tcon = 0;
    float Tu,Tv,Tw,To;
    switch(p->sector_num){
        case 1: 
            Taon = (1 + Uh + Ug) / 2;
            Tbon = Taon - Uh;
            Tcon = Tbon - Ug;
            Tu = Tcon;
            Tv = Tbon;
            Tw = Tbon;
            To = Taon;
        break;
        case 2: 
            Taon = (1 + Uh - Ug) / 2;
            Tbon = Taon - Uh;
            Tcon = Tbon + Ug;
            Tu = Tbon;
            Tv = Tcon;
            Tw = Tbon;
            To = Taon;
        break;
        case 3: 
            Taon = (1 - Uh + Ug) / 2;
            Tbon = Taon + Uh;
            Tcon = Tbon - Ug;
            Tu = Tcon;
            Tv = Tbon;
            Tw = Taon;
            To = Tbon;
        break;
        case 4: 
            Taon = (1 - Uh - Ug) / 2;
            Tbon = Taon + Uh;
            Tcon = Tbon + Ug;
            Tu = Tbon;
            Tv = Tcon;
            Tw = Taon;
            To = Tbon;
        break;
        default:
            Tu = 0.5f;
            Tv = 0.5f;
            Tw = 0.5f;
            To = 0.5f;
    }
    p->a_positive.cmp_val = PWM_PRD - Tv * PWM_PRD;
    p->a_negative.cmp_val = PWM_PRD - Tu * PWM_PRD;
    p->b_positive.cmp_val = PWM_PRD - To * PWM_PRD;
    p->b_negative.cmp_val = PWM_PRD - Tw * PWM_PRD;
}


/**
  * 函数功能:电流环运算
  * 输入参数:
  * 返回参数:
  * 说    明: 
  */
void Step_Vector_Curr_Control(Motor_Control *p){
    
    if(run_param.ident_flag == 1){
        /*获取d、q反馈值*/
        pack_handle.cos_val = arm_cos_f32(p->theta_target * tpi);
        pack_handle.sin_val = arm_sin_f32(p->theta_target * tpi);
        
        p->alpha_val_in = p->curr_a;
        p->beta_val_in = p->curr_b;
        
        pack_handle.alpha_val = p->alpha_val_in;
        pack_handle.beta_val = p->beta_val_in;
        
        Pack_Transform(&pack_handle);
        
        p->id_fbk = pack_handle.d_val;
        p->iq_fbk = pack_handle.q_val;
        
        p->id_fbk_flt += 0.1f * (p->id_fbk - p->id_fbk_flt);
        p->iq_fbk_flt += 0.1f * (p->iq_fbk - p->iq_fbk_flt);
        
        /*PID控制d、q轴输出*/
        pid_handle_d.err = p->id_ref - p->id_fbk;
        pid_handle_d.kp = p->curr_kp_d;//p->curr_kp;
        pid_handle_d.ki = p->curr_ki_d;//p->curr_ki;
        Pid_Control_Cal(&pid_handle_d);
        
        p->out_d = pid_handle_d.out - p->out_d_offset;
        p->out_d_volt = p->volt_power_flt * p->out_d;
        
        pid_handle_q.err = p->iq_ref - p->iq_fbk;
        pid_handle_q.kp = p->curr_kp_q;//p->curr_kp;
        pid_handle_q.ki = p->curr_ki_q;//p->curr_ki;
        Pid_Control_Cal(&pid_handle_q);
        
        p->out_q = pid_handle_q.out + p->out_q_offset;
        p->out_q_volt = p->volt_power_flt * p->out_q;
        
        /*反PACK变换计算A、B相输出电压*/
        pack_handle.d_val = p->out_d;
        pack_handle.q_val = p->out_q;
        
        IPack_Transform(&pack_handle);
        
        p->out_a = pack_handle.alpha_val;
        p->out_b = pack_handle.beta_val;
        
    }
    
    p->out_a = sat(p->out_a, -1.0f, 1.0f);
    p->out_b = sat(p->out_b, -1.0f, 1.0f);
    
    p->alpha_val_out = p->out_a;
    p->beta_val_out = p->out_b;
    
    p->out_alpha_volt = p->out_a * p->volt_power_flt;
    p->out_beta_volt = p->out_b * p->volt_power_flt;

    /*计算A、B相定时器比较值*/
    p->ua_cmp_val = p->out_a * PWM_HALF_PRD;
    p->ub_cmp_val = p->out_b * PWM_HALF_PRD;
    
    p->a_negative.cmp_val = PWM_HALF_PRD + p->ua_cmp_val;
    p->a_positive.cmp_val = PWM_HALF_PRD - p->ua_cmp_val;
    p->b_negative.cmp_val = PWM_HALF_PRD + p->ub_cmp_val;
    p->b_positive.cmp_val = PWM_HALF_PRD - p->ub_cmp_val;
    
//    Two_Phase_Space_Vector_Transform(p);
    
    /*双电阻采样限制极限占空比*/
    p->a_positive.cmp_val = sat(p->a_positive.cmp_val, PWM_MIN, PWM_MAX);
    p->a_negative.cmp_val = sat(p->a_negative.cmp_val, PWM_MIN, PWM_MAX);
    p->b_positive.cmp_val = sat(p->b_positive.cmp_val, PWM_MIN, PWM_MAX);
    p->b_negative.cmp_val = sat(p->b_negative.cmp_val, PWM_MIN, PWM_MAX);

    __HAL_TIM_SetCompare(p->a_positive.timer, p->a_positive.channel, p->a_positive.cmp_val);
    __HAL_TIM_SetCompare(p->a_negative.timer, p->a_negative.channel, p->a_negative.cmp_val);
    __HAL_TIM_SetCompare(p->b_positive.timer, p->b_positive.channel, p->b_positive.cmp_val);
    __HAL_TIM_SetCompare(p->b_negative.timer, p->b_negative.channel, p->b_negative.cmp_val);
}

