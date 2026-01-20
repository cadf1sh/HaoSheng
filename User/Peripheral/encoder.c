/**
  ******************************************************************************
  * 文件名称: 
  * 作    者: √
  * 版    本: V1.0
  * 编写日期: 
  * 功    能: 编码器位置、速度反馈计算
  ******************************************************************************
  */
  
/* 包含头文件 ----------------------------------------------------------------*/
#include "encoder.h"

Encoder encoder;                                        //编码器相关(速度信息)
Encoder_Position encoder_position;                      //编码器相关(位置信息)

uint16_t *ele_angle_flash = (uint16_t*)0x08010000;      //校准后的编码器数据数组


/**
  * 函数功能:编码器定时器初始化
  * 输入参数:
  * 返回参数:
  * 说    明: 
  */
void Encoder_Init(Encoder *p){
    p->cnt_timer->Instance->CNT = 0;                        //编码器值清零
    p->tim_timer->Instance->CCR1 = 0;

    __HAL_TIM_ENABLE_IT(p->tim_timer, TIM_IT_CC1);
    HAL_TIM_IC_Start_IT(p->tim_timer, TIM_CHANNEL_1);
    __HAL_TIM_ENABLE_IT(p->tim_timer, TIM_IT_CC2);
    HAL_TIM_IC_Start_IT(p->tim_timer, TIM_CHANNEL_2);
}

/**
  * 函数功能:编码器反馈位置结构体初始化
  * 输入参数:
  * 返回参数:
  * 说    明: 
  */
void Encoder_Postion_Cal_Init(Encoder_Position *p){
    p->cur_pos = 0;
    p->last_pos = 0;
    p->delta_pos = 0; 
    p->abs_pos = 0;
    p->pos_fbk = 0;
    p->electrical_angle = 0;
}

/**
  * 函数功能:编码器反馈速度结构体初始化
  * 输入参数:
  * 返回参数:
  * 说    明: 
  */
void Speed_Feedback_Cal_Init(Encoder *p){
    p->in_0 = 0;
    p->in_1 = 0;
    p->in_2 = 0;
    p->out_0 = 0;
    p->out_1 = 0;
    p->out_2 = 0;
    p->angle_delta_sum = 0;
    p->cal_ts_num = 4;
    p->cur_speed = 0;
    p->last_speed = 0;
    p->delta_speed = 0;
    p->we = 0;
    p->delta_speed_times = 0;
    p->speed_zero_times = 0;
    p->speed_zero_flag = 0;
    p->speed_dir = 1;
    p->theta_compensation = 0;
}

/**
  * 函数功能:编码器反馈位置计算
  * 输入参数:
  * 返回参数:
  * 说    明: 
  */
void Encoder_Postion_Cal(Encoder_Position *p){
    p->cur_pos = Encoder_Data_Get();
    p->delta_pos = p->cur_pos - p->last_pos;
    p->last_pos = p->cur_pos;
    
    if(p->delta_pos > HALF_ENCODER_LINE){
        p->delta_pos = p->delta_pos - ENCODER_LINE;
    }else if(p->delta_pos < -HALF_ENCODER_LINE){
        p->delta_pos = p->delta_pos + ENCODER_LINE;
    }
    p->ele_angle_delta = p->delta_pos * MOTOR_POLES / ENCODER_LINE;
    p->pulse_enc_cnt = p->pulse_enc_cnt + p->delta_pos;
    p->abs_pos = p->abs_pos + p->delta_pos;
    p->pos_fbk = p->pulse_enc_cnt;
    
    p->electrical_angle = thetafrac(ele_angle_flash[p->cur_pos] * micro_varible.micro_inv);//thetafrac(ele_angle_flash[p->cur_pos] * micro_varible.micro_inv);//ele_angle_flash[p->cur_pos] * 0.0000152587890625;//ele_angle_cache[p->cur_pos];//thetafrac(p->cur_pos * micro_varible.encoder_inv);
}

void Abs_Encoder_Init(void){
    encoder_position.last_pos = Encoder_Data_Get();
}

/**
  * 函数功能:编码器反馈速度计算
  * 输入参数:
  * 返回参数:
  * 说    明: 
  */

void Speed_Feedback_Cal(Encoder *p){
    static uint8_t count_enc = 0;
    count_enc++;
    p->angle_delta_sum += encoder_position.ele_angle_delta;             //电角度变化量
    if(count_enc >= p->cal_ts_num){
        count_enc = 0;
        p->in_0 = p->angle_delta_sum * (5000.0f / 50.0f);
        //p->out_0 = 1.64705f * p->out_1 - 0.7006f * p->out_2 + 0.01339f * p->in_0 + 0.02678f * p->in_1 + 0.01339f * p->in_2;
        p->cur_speed = p->in_0 * 60.0f;
//        p->in_2 = p->in_1;
//        p->in_1 = p->in_0;
//        p->out_2 = p->out_1;
//        p->out_1 = p->out_0;
        p->angle_delta_sum = 0;
    }
    p->cur_speed_flt += 0.1f * (p->cur_speed - p->cur_speed_flt);
    if(p->cur_speed > 8000 || p->cur_speed < -8000){//飞车停机
        run_param.sys_status  = FAULT;
        run_param.sys_err_code = VDS_ERR;
    }
    p->we = p->cur_speed / 60.0f * MOTOR_POLES * tpi;
    p->last_speed = p->cur_speed;
    p->speed_dir = p->cur_speed < 0 ? -1 : 1;
    p->theta_compensation = sat(p->speed_dir * sat(fabsf(p->we), 0, 100000) * 1.5f * INVT_PRD / tpi, -0.25f, 0.25f);
}

/**
  * 函数功能: 编码器原始数据读取
  * 输入参数:
  * 返回参数:
  * 说    明: ABZ模式读取（在system_init.c中实现了绝对值模式读取会覆盖此函数）
  */
__weak uint16_t Encoder_Data_Get(void){
    return __HAL_TIM_GetCounter(encoder_position.cnt_timer);
}


