/**
  ******************************************************************************
  * 文件名称: 
  * 作    者: √
  * 版    本: V1.0
  * 编写日期: 
  * 功    能: FOC三环位置模式自测
  ******************************************************************************
  */
  
/* 包含头文件 ----------------------------------------------------------------*/
#include "position_mode_test.h"

Position_Motion_Comb motion_comb;

/**
  * 函数功能: 位置模式测试序列速度和加速度设置
  * 输入参数:
  * 返回参数:
  * 说    明: 在位移较小速度较大的情况下，加速度必须足够大，程序中未做自适应最大速度处理
  */
void Position_Motion_Comb_Init(Position_Motion_Comb *p){
    p->curr_seq_idx = 0;
    p->curr_speed_idx = 0;
    p->speed_seq[0] = 60.0f;
    p->acc_seq[0] = 1200.0f;
    p->speed_seq[1] = 120.0f;
    p->acc_seq[1] = 2400.0f;
    p->speed_seq[2] = 300.0f;
    p->acc_seq[2] = 7200.0f;
    p->speed_seq[3] = 400.0f;
    p->acc_seq[3] = 12000.0f;
    p->speed_seq[4] = 600.0f;
    p->acc_seq[4] = 24000.0f;
    p->speed_seq[5] = 800.0f;
    p->acc_seq[5] = 48000.0f;
    p->speed_seq[6] = 900.0f;
    p->acc_seq[6] = 72000.0f;
    p->gap_cnt = 0;
    Position_Sequence_Init(p);
}

/**
  * 函数功能: 设置位置增量序列参数
  * 输入参数:
  * 返回参数:
  * 说    明:
  */
void Position_Sequence_Init(Position_Motion_Comb *p){
    if(p->curr_speed_idx <= 2){
        p->len = 14;
    }else if(p->curr_speed_idx <= 4){
        p->len = 16;
    }else if(p->curr_speed_idx < 7){
        p->len = 18;
    }
        for(uint8_t i = 0; i < 4; i++){
            p->pos_seq[i].position_delta = 12800;                       //移动的脉冲数量（根据细分设置 4.0f * MICRO_NUM * 50 为一整圈的脉冲数)，此处为256细分的1/4圈）
            p->pos_seq[i].max_speed = p->speed_seq[p->curr_speed_idx];
            p->pos_seq[i].max_acc= p->acc_seq[p->curr_speed_idx];
            p->pos_seq[i].time_gap = 10000;                             //到位后与下一动作的时间间隔 10000 为0.5s
            p->pos_seq[i+4].position_delta = -12800;
            p->pos_seq[i+4].max_speed = p->speed_seq[p->curr_speed_idx];
            p->pos_seq[i+4].max_acc= p->acc_seq[p->curr_speed_idx];
            p->pos_seq[i+4].time_gap = 10000;
        }
        for(uint8_t i = 8; i < 10; i++){
            p->pos_seq[i].position_delta = 25600;
            p->pos_seq[i].max_speed = p->speed_seq[p->curr_speed_idx];
            p->pos_seq[i].max_acc= p->acc_seq[p->curr_speed_idx];
            p->pos_seq[i].time_gap = 10000;
            p->pos_seq[i+2].position_delta = -25600;
            p->pos_seq[i+2].max_speed = p->speed_seq[p->curr_speed_idx];
            p->pos_seq[i+2].max_acc= p->acc_seq[p->curr_speed_idx];
            p->pos_seq[i+2].time_gap = 10000;
        }
        p->pos_seq[12].position_delta = 51200;
        p->pos_seq[12].max_speed = p->speed_seq[p->curr_speed_idx];
        p->pos_seq[12].max_acc= p->acc_seq[p->curr_speed_idx];
        p->pos_seq[12].time_gap = 10000;
        p->pos_seq[13].position_delta = -51200;
        p->pos_seq[13].max_speed = p->speed_seq[p->curr_speed_idx];
        p->pos_seq[13].max_acc= p->acc_seq[p->curr_speed_idx];
        p->pos_seq[13].time_gap = 10000;
        
        p->pos_seq[14].position_delta = 512000;
        p->pos_seq[14].max_speed = p->speed_seq[p->curr_speed_idx];
        p->pos_seq[14].max_acc= p->acc_seq[p->curr_speed_idx];
        p->pos_seq[14].time_gap = 10000;
        p->pos_seq[15].position_delta = -512000;
        p->pos_seq[15].max_speed = p->speed_seq[p->curr_speed_idx];
        p->pos_seq[15].max_acc= p->acc_seq[p->curr_speed_idx];
        p->pos_seq[15].time_gap = 10000;
        
        p->pos_seq[16].position_delta = 5120000;
        p->pos_seq[16].max_speed = p->speed_seq[p->curr_speed_idx];
        p->pos_seq[16].max_acc= p->acc_seq[p->curr_speed_idx];
        p->pos_seq[16].time_gap = 10000;
        p->pos_seq[17].position_delta = -5120000;
        p->pos_seq[17].max_speed = p->speed_seq[p->curr_speed_idx];
        p->pos_seq[17].max_acc= p->acc_seq[p->curr_speed_idx];
        p->pos_seq[17].time_gap = 10000;

    for(uint8_t i = 0; i < p->len; i++){                            //设置序列里的每个动作单元的初始状态
        p->pos_seq[i].cal_complete = 1;
        p->pos_seq[i].run_end = 1;
    }
    
}

/**
  * 函数功能: 自测运行流程控制
  * 输入参数:
  * 返回参数:
  * 说    明:
  */
void Position_Mode_Test_Cal(Position_Motion_Comb *motion_comb, Position_Control *pos_ctrl){
    if(pos_ctrl->cal_complete == 1 && pos_ctrl->run_end == 1){              //上一个单元执行完成标志置位
        motion_comb->gap_cnt++;
        if(motion_comb->gap_cnt >= motion_comb->pos_seq[motion_comb->curr_seq_idx].time_gap){
            motion_comb->gap_cnt = 0;
            pos_ctrl->position_delta = motion_comb->pos_seq[motion_comb->curr_seq_idx].position_delta;
            pos_ctrl->max_speed = motion_comb->pos_seq[motion_comb->curr_seq_idx].max_speed;
            pos_ctrl->max_acc = motion_comb->pos_seq[motion_comb->curr_seq_idx].max_acc;
            pos_ctrl->cal_complete = 0;
            motion_comb->curr_seq_idx++;
        }
    }
    if(motion_comb->curr_seq_idx > motion_comb->len - 1){       //循环执行
        motion_comb->curr_seq_idx = 0;
        motion_comb->curr_speed_idx++;
        if(motion_comb->curr_speed_idx > 6){
            motion_comb->curr_speed_idx = 0;
        }
        Position_Sequence_Init(motion_comb);
    }
}

