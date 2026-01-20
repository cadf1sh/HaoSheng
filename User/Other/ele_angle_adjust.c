/**
  ******************************************************************************
  * 文件名称: 
  * 作    者: √
  * 版    本: V1.0
  * 编写日期: 
  * 功    能: 编码器非线性校准(步进电机极对数多，编码器存在非线性，每个编码器值对应的电角度可能不准确，所以需要校准)
  ******************************************************************************
  */
  
/* 包含头文件 ----------------------------------------------------------------*/
#include "ele_angle_adjust.h"
#include "my_flash.h"
#include "encoder.h"
#include "current_sample.h"
#include "foc_control.h"

#define                         abs(x) ((x)>0? (x):(-(x)))
#define                         ADJUST_STEP_UNIT                    1           //校准速度（每个执行周期的脉冲增量）
#define                         ADJUST_CURR                         0.5f        //校准电流（A）

Electrical_Angle_Adjust ele_angle_adjust;

/**
  * 函数功能: 电流闭环 D轴强拖
  * 输入参数:
  * 返回参数:
  * 说    明:
  */
void Motor_Open_Run(int32_t step, float curr_mag){
    /*电流采样*/
    Voltage_Value_Cal(&current_sample);
    Current_Cal(&current_sample);
    float ele_theta_temp = thetafrac(step * micro_varible.micro_inv);               //根据细分设置将脉冲转为电角度
    /*电流环SPWM*/
    motor_control.id_ref = curr_mag;
    motor_control.iq_ref = 0;
    motor_control.theta_target = ele_theta_temp;
    motor_control.volt_power_flt = current_sample.volt_power_flt;
    motor_control.curr_a = current_sample.curr_a;
    motor_control.curr_b = current_sample.curr_b;
    motor_control.out_d_offset = 0;
    motor_control.out_q_offset = 0;
    Step_Vector_Curr_Control(&motor_control);
}

/**
  * 函数功能: 校准时的电机运动控制
  * 输入参数:
  * 返回参数:
  * 说    明:
  */
void Motor_Adjust(void){
    if(ele_angle_adjust.adjust_enable == 0){
        return;
    }
    Encoder_Postion_Cal(&encoder_position);
    switch (ele_angle_adjust.adjust_status) {
        case 0:
            //初始化电机  先将电机输出到一圈位置
            ele_angle_adjust.adjust_step_out = MICRO_NUM * STEP_NUM;
            Motor_Open_Run(ele_angle_adjust.adjust_step_out, ADJUST_CURR);
            ele_angle_adjust.adjust_status = 1;
            break;
        case 1:
            //先正转一圈
            ele_angle_adjust.adjust_step_out += ADJUST_STEP_UNIT;
            Motor_Open_Run(ele_angle_adjust.adjust_step_out, ADJUST_CURR);
            if(ele_angle_adjust.adjust_step_out == MICRO_NUM * STEP_NUM * 2){
              ele_angle_adjust.adjust_step_out = MICRO_NUM * STEP_NUM;
              ele_angle_adjust.adjust_status = 2;
            }
            break;
        case 2:
            //正转一圈，每隔Step采集一次数据
            if(ele_angle_adjust.adjust_step_out % MICRO_NUM==0){
                ele_angle_adjust.adjust_data[(ele_angle_adjust.adjust_step_out - MICRO_NUM * STEP_NUM) / MICRO_NUM] =  encoder_position.cur_pos;
            }
            ele_angle_adjust.adjust_step_out += ADJUST_STEP_UNIT;
            Motor_Open_Run(ele_angle_adjust.adjust_step_out, ADJUST_CURR);

            if(ele_angle_adjust.adjust_step_out > MICRO_NUM * STEP_NUM * 2){
                ele_angle_adjust.adjust_status = 3;
            }
            break;
        case 3:
            //多前进20步，消除死区
            ele_angle_adjust.adjust_step_out += ADJUST_STEP_UNIT;
            Motor_Open_Run(ele_angle_adjust.adjust_step_out, ADJUST_CURR);
            if(ele_angle_adjust.adjust_step_out == (MICRO_NUM * STEP_NUM * 2 + 20 * MICRO_NUM)){
                ele_angle_adjust.adjust_status = 4;
            }
            break;
        case 4:
            //回退到两圈位置
            ele_angle_adjust.adjust_step_out -= ADJUST_STEP_UNIT;
            Motor_Open_Run(ele_angle_adjust.adjust_step_out, ADJUST_CURR);
            if(ele_angle_adjust.adjust_step_out == MICRO_NUM * STEP_NUM * 2){
                ele_angle_adjust.adjust_status = 5;
            }
            break;
        case 5:
            //反向测量
            if(ele_angle_adjust.adjust_step_out % MICRO_NUM == 0){
                ele_angle_adjust.adjust_dataR[(ele_angle_adjust.adjust_step_out - MICRO_NUM * STEP_NUM) / MICRO_NUM] = encoder_position.cur_pos;
            }
            ele_angle_adjust.adjust_step_out -= ADJUST_STEP_UNIT;
            Motor_Open_Run(ele_angle_adjust.adjust_step_out, ADJUST_CURR);

            if(ele_angle_adjust.adjust_step_out < MICRO_NUM*STEP_NUM){
                ele_angle_adjust.adjust_status = 6;
            }
            break;
        case 6:
            //计算
            Motor_Open_Run(0, 0);
            break;
  }
}

/**
  * 函数功能: 取循环差
  * 输入参数:
  * 返回参数:
  * 说    明: 处理过零点数据
  */
int32_t Cycle_Sub(int32_t a, int32_t b, int32_t cyc){
    int32_t sub_data;
    sub_data = a - b;
    if(sub_data > (cyc >> 1)){
        sub_data -= cyc;
    }
    if(sub_data < (-cyc >> 1)){
        sub_data += cyc;
    }
    return sub_data;
}

/**
  * 函数功能: 取余数
  * 输入参数:
  * 返回参数:
  * 说    明: 
  */
uint32_t Cycle_Rem(uint32_t a, uint32_t b){
    return (a + b) % b;
}

/**
  * 函数功能: 取循环平均值
  * 输入参数:
  * 返回参数:
  * 说    明: 
  */
int32_t Cycle_Average(int32_t a, int32_t b, int32_t cyc){
    int32_t sub_data;
    int32_t ave_data;

    sub_data = a - b;
    ave_data = (a + b) >> 1;

    if(abs(sub_data) > (cyc >> 1)){
        if(ave_data >= (cyc >> 1)){
            ave_data -= (cyc >> 1);
        }
        else{
            ave_data += (cyc >> 1);
        }
    }
    return ave_data;
}


/**
  * 函数功能: 非线性校准解算、写入
  * 输入参数:
  * 返回参数:
  * 说    明: 参考https://zhuanlan.zhihu.com/p/672159726
  */
void Motor_Adjust_Data_Cal(void){
    uint16_t i = 0;
    int32_t sub_data;        //用于各个算差
    if(ele_angle_adjust.adjust_status != 6){
        return;
    }

    //获取正向和反向对应点位平均值保存到正向数组
    for(i = 0; i < STEP_NUM + 1; i++){
        ele_angle_adjust.adjust_data[i] = (uint16_t)Cycle_Average((int32_t)ele_angle_adjust.adjust_data[i], (int32_t)ele_angle_adjust.adjust_dataR[i], ENCODER_LINE);
    }
    for(i = 0; i < STEP_NUM; i++){
        sub_data = (int32_t)ele_angle_adjust.adjust_data[i+1] - (int32_t)ele_angle_adjust.adjust_data[i];
        if(sub_data < 0){                               //跨零点
            ele_angle_adjust.zero_x = i;                //过零点的前一个数据下标
            ele_angle_adjust.zero_y = (ENCODER_LINE - 1) - ele_angle_adjust.adjust_data[ele_angle_adjust.zero_x]; // 16383-零点对应的编码器校准数据
        }
    }
    //非线性校准
    int32_t subdata = 0;
    int32_t step_x, step_y;
    uint16_t data_u16 = 0;
    ele_angle_adjust.count = 0;
    Flash_Erase_Angle_Data();                                                                                           //擦除数据区
    Flash_Write_Data_Begin();                                                                                           //开始写数据区
    for(step_x = ele_angle_adjust.zero_x; step_x < ele_angle_adjust.zero_x + STEP_NUM + 1; step_x++){                   //从过零点前一个数据开始循环整个数组
        subdata = Cycle_Sub(ele_angle_adjust.adjust_data[Cycle_Rem(step_x+1, STEP_NUM)],ele_angle_adjust.adjust_data[Cycle_Rem(step_x, STEP_NUM)], ENCODER_LINE);
        if(step_x == ele_angle_adjust.zero_x){                                  //零点边缘校准
            for(step_y = ele_angle_adjust.zero_y; step_y < subdata; step_y++){
                data_u16 = Cycle_Rem(MICRO_NUM * step_x + MICRO_NUM * step_y / subdata, STEP_NUM * MICRO_NUM);
                Flash_Write_Data16(data_u16);
                ele_angle_adjust.count++;
            }
        }else if(step_x == ele_angle_adjust.zero_x + STEP_NUM){                 //结束边缘
            for(step_y = 0; step_y < ele_angle_adjust.zero_y; step_y++){
                data_u16 = Cycle_Rem(MICRO_NUM * step_x + MICRO_NUM * step_y / subdata, STEP_NUM * MICRO_NUM);
                Flash_Write_Data16(data_u16);
                ele_angle_adjust.count++;
            }
        }else{                                                                  //中间
            for(step_y = 0; step_y < subdata; step_y++){
                data_u16 = Cycle_Rem(MICRO_NUM * step_x + MICRO_NUM * step_y / subdata, STEP_NUM * MICRO_NUM);
                Flash_Write_Data16(data_u16);
                ele_angle_adjust.count++;
            }
        }
    }
    Flash_Write_Data_End();        //结束写数据区
    if(ele_angle_adjust.count != ENCODER_LINE){
        return;
    }
    //清理校准信号
    ele_angle_adjust.adjust_status = 0;
    ele_angle_adjust.adjust_enable = 0;            //清除校准触发
}
