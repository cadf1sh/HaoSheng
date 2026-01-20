#ifndef MOTION_CONTROL_H
#define MOTION_CONTROL_H

#include "public.h"

/*参考位置给定方式*/
typedef enum{
    EXT_PULSE = 0,      //外部脉冲捕获
    INT_SPEC = 1        //内部计算生成
}Run_Mode;

typedef struct{
    TIM_HandleTypeDef *cnt_timer;                    //计数定时器
    TIM_HandleTypeDef *tim_timer;                    //计数定时器
    Run_Mode run_mode;                              //运行模式 脉冲/给定
    Motion_State motion_state;
    int8_t speed_ref_dir;                           //参考速度方向
    float target_speed;                             //目标速度(rpm)
    float curr_ref_speed;                           //当前参考速度
    float speed_constant_pulse_increment;           //匀速阶段脉冲增量
    float speed_change_pulse_increment;             //变速阶段脉冲增量
    float speed_up;                                 //加速度(rpm/s)
    float speed_increment;                          //速度增量
    float open_speed_increment_max;
    float step_temp;                                //脉冲计数中间变量
    float speed_increment_last;
    float speed_increment_delta;
    
    int32_t curr_timer_cnt;                         //本次定时器cnt值
    int32_t last_timer_cnt;                         //上次定时器cnt值
    int32_t delta_timer_cnt;                        //定时器cnt值变化量
    
    int32_t pulse_abs_cnt;                          //参考绝对脉冲数量
    int32_t pulse_ref_cnt;                          //参考脉冲数量
    int32_t pulse_enc_cnt;                          //编码器脉冲数量
    float consecutive_position;                     //连续位置脉冲数量(浮点型，用于后续计算步进theta值)
    float electrical_angle_ref;                     //参考电角度值（0-1）
}Motion_Control;

typedef struct {
    int8_t dir;                     //方向
    int32_t start_position;         //起始位置
    int32_t end_position;           //结束位置
    int32_t curr_position;          //当前位置
    int32_t au_position;            //实际位置(编码器)
    int32_t position_delta;         //位置增量(设定值)
    int32_t acc_displacement;       //加速阶段脉冲数量
    int32_t uni_displacement;       //匀速阶段脉冲数量
    int32_t dec_displacement;       //减速阶段脉冲数量
    uint8_t cal_complete;           //中间变量计算完成标志
    uint8_t run_end;                //执行结束标志
    float max_speed;                //最大速度
    float max_acc;                  //最大加速度
    float acc_time;                 //加速阶段时间
    float uni_time;                 //匀速阶段时间
    float dec_time;                 //减速阶段时间
    float curr_speed;               //实时速度
    float curr_acc;                 //实时加速度
}Position_Control;

extern Position_Control position_control;
extern Motion_Control motion_control;

void Motion_Control_Init(Motion_Control *p);
void Internally_Specified(Motion_Control *p);
void Pulse_Command_Cal(Motion_Control *p);

void Position_Control_Init(Position_Control *p);
void Position_Control_Cal(Position_Control *p, int32_t curr_position, int32_t encoder_position);
void Position_Pulse_Set(Position_Control *p,Motion_Control *motion_control);


#endif


