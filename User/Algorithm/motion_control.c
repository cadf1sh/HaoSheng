/**
  ******************************************************************************
  * 文件名称: 
  * 作    者: √
  * 版    本: V1.0
  * 编写日期: 
  * 功    能: 运动控制（加减速，速度给定）
  ******************************************************************************
  */
  
/* 包含头文件 ----------------------------------------------------------------*/
#include "motion_control.h"

Position_Control position_control;               //位置梯形加减速
Motion_Control motion_control;                  //运动控制(速度、位置给定)

/**
  * 函数功能:运动控制指令给定计算初始化
  * 输入参数:
  * 返回参数:
  * 说    明:
  */
void Motion_Control_Init(Motion_Control *p){
    p->speed_ref_dir = 1;
    p->target_speed = 0;
    p->curr_ref_speed = 0;
    p->speed_constant_pulse_increment = 0;
    p->speed_change_pulse_increment = 0;
    p->speed_up = 2400;
    p->speed_increment = 0;
    p->step_temp = 0;
    
    p->curr_timer_cnt = 0;
    p->last_timer_cnt = 0;
    p->delta_timer_cnt = 0;
    
    p->pulse_ref_cnt = 0;
    p->pulse_abs_cnt = 0;
    p->pulse_enc_cnt = 0;
    p->consecutive_position = 0;
}

/**
  * 函数功能: 增量式位置梯形加减速结构体初始化
  * 输入参数:
  * 返回参数:
  * 说    明: 
  */
void Position_Control_Init(Position_Control *p){
    p->dir = 1;
    p->start_position = 0;
    p->end_position = 0;
    p->curr_position = 0;
    p->au_position = 0;
    p->position_delta = 4000;
    p->acc_displacement = 0;
    p->uni_displacement = 0;
    p->dec_displacement = 0;
    p->cal_complete = 1;
    p->run_end = 0;
    p->max_speed = 1200.0f;
    p->max_acc = 120000.0f;
    p->acc_time = 0;
    p->uni_time = 0;
    p->dec_time = 0;
    p->curr_speed = 0;
    p->curr_acc = 0;
}

/**
  * 函数功能: 增量式位置梯形加减速中间变量计算
  * 输入参数:
  * 返回参数:
  * 说    明: 
  */
void Position_Control_Cal(Position_Control *p, int32_t curr_position, int32_t encoder_position){
    if(p->cal_complete != 1){
        p->dir = p->position_delta >= 0 ? 1 : -1;
        p->acc_time = p->max_speed / p->max_acc;
        p->uni_time = (fabsf(p->position_delta / (MOTOR_POLES * micro_varible.micro_flt))) / (p->max_speed / 60.0f) - p->acc_time;
        p->dec_time = p->acc_time;
        p->acc_displacement = p->dir * 0.5f * (p->max_acc / 60.0f) * (p->acc_time * p->acc_time) * (MOTOR_POLES * micro_varible.micro_flt);
        p->uni_displacement = p->dir * (p->max_speed / 60.0f) * p->uni_time * (MOTOR_POLES * micro_varible.micro_flt);
        p->dec_displacement = p->acc_displacement;
        p->curr_position = curr_position;
        p->au_position = encoder_position;
        p->start_position = p->curr_position;
        p->end_position = p->start_position + p->position_delta;
        p->cal_complete = 1;
        p->run_end = 0;
    }
}

/**
  * 函数功能: 根据增量式位置梯形加减速计算的中间变量控制各阶段切换
  * 输入参数:
  * 返回参数:
  * 说    明: 
  */
uint8_t temp_end_flag = 0;
void Position_Pulse_Set(Position_Control *p, Motion_Control *motion_control){
    if(p->cal_complete != 0 && p->run_end != 1){
        if(p->dir > 0){
            if(fabsf(p->au_position - (p->end_position * micro_varible.encoder_flt * micro_varible.micro_inv)) <= 20.0f){           //接近目标位置
                motion_control->speed_up = 12000.0f;
                temp_end_flag = 1;
                motion_control->pulse_abs_cnt = p->end_position;
            }else if(temp_end_flag == 0 && p->curr_position < p->start_position + p->acc_displacement + p->uni_displacement){       //加速和匀速阶段
                motion_control->target_speed = p->max_speed;
                motion_control->speed_up = p->max_acc;
            }else if(temp_end_flag == 0 && p->curr_position >= p->end_position - p->dec_displacement){                              //减速阶段
                motion_control->target_speed = 0.0f;
                motion_control->speed_up = p->max_acc;
            }else if(temp_end_flag == 1 && fabsf(p->au_position - (p->end_position * micro_varible.encoder_flt * micro_varible.micro_inv)) > 20.0f){        //处理到位停止时的误差
                motion_control->pulse_abs_cnt = p->end_position;
            }
        }else{
            if(fabsf(p->au_position - (p->end_position * micro_varible.encoder_flt * micro_varible.micro_inv)) <= 20.0f){
                motion_control->speed_up = 12000.0f;
                temp_end_flag = 1;
                motion_control->pulse_abs_cnt = p->end_position;
            }else if(temp_end_flag == 0 && p->curr_position > p->start_position + p->acc_displacement + p->uni_displacement){       //加速和匀速阶段
                motion_control->target_speed = -p->max_speed;
                motion_control->speed_up = p->max_acc;
            }else if(temp_end_flag == 0 && p->curr_position <= p->end_position - p->dec_displacement){                              //减速阶段
                motion_control->target_speed = 0.0f;
                motion_control->speed_up = p->max_acc;
            }else if(temp_end_flag == 1 && fabsf(p->au_position - (p->end_position * micro_varible.encoder_flt * micro_varible.micro_inv)) > 20.0f){       //处理到位停止时的误差
                motion_control->pulse_abs_cnt = p->end_position;
            }
        }
        if(fabsf(p->au_position - (p->end_position * micro_varible.encoder_flt * micro_varible.micro_inv)) < 1.0f){         //位置误差小于1运行结束
            p->run_end = 1;
            temp_end_flag = 0;
            motion_control->pulse_abs_cnt = p->end_position;         //直接给定结束目标位置
            motion_control->speed_change_pulse_increment = 0;
            motion_control->target_speed = 0;
        }
    }
}


/**
  * 函数功能:运动控制计算，根据target_speed和speed_up计算当前实时参考速度
  * 输入参数:
  * 返回参数:
  * 说    明: 根据细分设置以及目标速度和加速度生成实时参考速度和参考位置
  */
uint16_t cnt_temp = 0;
void Internally_Specified(Motion_Control *p){
    p->speed_constant_pulse_increment = DEFAULT_DIR * p->target_speed * micro_varible.micro_slt * MOTOR_POLES / 60.0f / CURR_LOOP_FREQ;
    p->speed_increment = p->speed_up * micro_varible.micro_slt * MOTOR_POLES / 60.0f / CURR_LOOP_FREQ / (CURR_LOOP_FREQ / SPD_UP_DELTA_TIME);

    if(run_param.ident_flag == 1){
        cnt_temp++;
    }
    if(cnt_temp >= SPD_UP_DELTA_TIME){
        if(p->speed_change_pulse_increment < p->speed_constant_pulse_increment){
            p->speed_change_pulse_increment += p->speed_increment;
            p->speed_change_pulse_increment = sat(p->speed_change_pulse_increment, p->speed_change_pulse_increment, p->speed_constant_pulse_increment);
        }else if(p->speed_change_pulse_increment > p->speed_constant_pulse_increment){
            p->speed_change_pulse_increment -= p->speed_increment;
            p->speed_change_pulse_increment = sat(p->speed_change_pulse_increment, p->speed_constant_pulse_increment, p->speed_change_pulse_increment);
        }
        cnt_temp = 0;
        p->speed_increment_delta = fabsf(p->speed_change_pulse_increment) - p->speed_increment_last;
        p->speed_increment_last = fabsf(p->speed_change_pulse_increment);
        p->motion_state = p->speed_increment_delta > 0 ? ACCELERATE : (p->speed_increment_delta < 0 ? DECELERATE : UNIFORM);
    }
    p->speed_ref_dir = p->speed_change_pulse_increment > 0 ? -1 : 1;
    p->step_temp += p->speed_change_pulse_increment;

    if(p->step_temp >= 1.0f || p->step_temp <= -1.0f){
        p->pulse_ref_cnt = p->pulse_ref_cnt + (int32_t)p->step_temp;
        p->pulse_abs_cnt += (int32_t)p->step_temp;
        p->step_temp = thetafrac(p->step_temp);
    }

    p->curr_ref_speed = p->speed_change_pulse_increment / micro_varible.micro_slt / (MOTOR_POLES / 60.0f / CURR_LOOP_FREQ);
}

/**
  * 函数功能:脉冲指令转换以及溢出处理
  * 输入参数:
  * 返回参数:
  * 说    明: 
  */
void Pulse_Command_Cal(Motion_Control *p){
    
    int32_t full_step_cnt;
    int32_t pulse_sat;
    
    Internally_Specified(p);
    
    if(p->pulse_ref_cnt > 100000){
        full_step_cnt = 100000 / micro_varible.micro_slt;
        pulse_sat = full_step_cnt * micro_varible.micro_slt;
        p->pulse_ref_cnt = p->pulse_ref_cnt - pulse_sat;
        p->consecutive_position = p->consecutive_position - pulse_sat;
        pulse_sat = full_step_cnt * micro_varible.encoder_slt;
        p->pulse_enc_cnt = p->pulse_enc_cnt - pulse_sat;
    }
    if(p->pulse_ref_cnt < -100000){
        full_step_cnt = 100000 / micro_varible.micro_slt;
        pulse_sat = full_step_cnt * micro_varible.micro_slt;
        p->pulse_ref_cnt = p->pulse_ref_cnt + pulse_sat;
        p->consecutive_position = p->consecutive_position + pulse_sat;
        pulse_sat = full_step_cnt * micro_varible.encoder_slt;
        p->pulse_enc_cnt = p->pulse_enc_cnt + pulse_sat;
    }
    p->electrical_angle_ref = thetafrac(p->pulse_ref_cnt * micro_varible.micro_inv);
}



