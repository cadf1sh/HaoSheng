#ifndef ENCODER_H_
#define ENCODER_H_

#include "public.h"

typedef struct{
    TIM_HandleTypeDef *cnt_timer;//计数定时器
    uint16_t cur_pos;            //本次编码器值
    uint16_t last_pos;           //上次编码器值
    int16_t delta_pos;          //编码器值变化量
    float ele_angle_delta;      //电角度变化量
    int32_t abs_pos;            //绝对位置
    int32_t pulse_enc_cnt;      //编码器位置
    int32_t pos_fbk;            //反馈位置（同绝对位置）
    float electrical_angle;     //编码器值转电角度(0-1)
}Encoder_Position;

typedef struct{
    TIM_HandleTypeDef *cnt_timer;//计数定时器
    TIM_HandleTypeDef *tim_timer;//计时定时器
    float in_0;
    float in_1;
    float in_2;
    float out_0;
    float out_1;
    float out_2;
    uint8_t cal_ts_num;         //测速周期数(指定多少个控制周期计算一次速度)
    int16_t delta_pos;          //编码器值变化量
    float angle_delta_sum;      //电角度变化量
    float cur_speed;            //本次计算速度  机械速度 rpm
    float cur_speed_flt;        //滤波后的速度  机械速度 rpm
    float we;                   //电角频率rad/s
    float last_speed;           //上次计算速度
    float delta_speed;          //速度变化量
    uint8_t delta_speed_times;  //速度突变计数
    uint8_t speed_zero_times;   //0速计数值
    uint8_t speed_zero_flag;    //0速标志
    int8_t speed_dir;           //速度方向 ±1
    float theta_compensation;   //电角度补偿值（0-1）
}Encoder;

extern Encoder encoder;
extern Encoder_Position encoder_position;

void Encoder_Init(Encoder *p);
void Encoder_Postion_Cal_Init(Encoder_Position *p);
void Abs_Encoder_Init(void);
void Speed_Feedback_Cal_Init(Encoder *p);
void Encoder_Postion_Cal(Encoder_Position *p);
void Speed_Feedback_Cal(Encoder *p);

uint16_t Encoder_Data_Get(void);

#endif

