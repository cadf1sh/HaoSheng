#ifndef __POSITION_MODE_TEST_H
#define __POSITION_MODE_TEST_H

#include "motion_control.h"

/*位置模式执行单元结构体*/
typedef struct{
    float max_speed;            //最大速度
    float max_acc;              //加速度
    int32_t position_delta;     //位置增量
    int32_t time_gap;           //两个单元之间的时间间隔
    uint8_t cal_complete;       //中间变量计算完成标志
    uint8_t run_end;            //单元执行完成标志
}Position_Sequence;

/*位置模式执行序列结构体*/
typedef struct{
    uint8_t len;                //序列执行单元个数
    uint8_t curr_seq_idx;       //当前执行单元的下标
    uint8_t curr_speed_idx;     //当前执行单元速度下标
    uint32_t gap_cnt;           //两个单元之间的时间间隔计数
    Position_Sequence pos_seq[18];//执行单元序列（每个单元18个动作）
    float speed_seq[7];         //速度序列
    float acc_seq[7];           //加速度序列
}Position_Motion_Comb;

extern Position_Motion_Comb motion_comb;

void Position_Motion_Comb_Init(Position_Motion_Comb *p);
void Position_Sequence_Init(Position_Motion_Comb *p);
void Position_Mode_Test_Cal(Position_Motion_Comb *motion_comb, Position_Control *pos_ctrl);

#endif


