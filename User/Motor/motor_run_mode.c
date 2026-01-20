/**
  ******************************************************************************
  * 文件名称: 
  * 作    者: √
  * 版    本: V1.0
  * 编写日期: 
  * 功    能: 电机运行模式控制
  ******************************************************************************
  */
  
/* 包含头文件 ----------------------------------------------------------------*/
#include "motor_run_mode.h"
#include "encoder.h"
#include "motion_control.h"
#include "current_sample.h"
#include "param_identify.h"
#include "foc_control.h"
#include "motor_run_mode.h"
#include "position_mode_test.h"

/*矢量控制相关结构体*/
Speed_Loop speed_loop;                          //速度环
Position_Loop position_loop;                    //位置环

Pid_Control_Handle pid_handle_speed_loop;
Pid_Control_Handle pid_handle_position_loop;

/**
  * 函数功能:电流环
  * 输入参数:
  * 返回参数:
  * 说    明: 
  */
void Step_Motor_Run_Current_Loop(void){
    /*电流采样*/
    Voltage_Value_Cal(&current_sample);
    Current_Cal(&current_sample);

    /*编码器位置和速度计算*/
    Speed_Feedback_Cal(&encoder);
    Encoder_Postion_Cal(&encoder_position);

    /*电流环SPWM*/
    motor_control.id_ref = 0;
    motor_control.iq_ref = motor_control.iq_ref;
    motor_control.theta_target = encoder_position.electrical_angle + encoder.theta_compensation;
    motor_control.volt_power_flt = current_sample.volt_power_flt;
    motor_control.curr_a = current_sample.curr_a;
    motor_control.curr_b = current_sample.curr_b;
    motor_control.out_d_offset = 0;
    motor_control.out_q_offset = 0;
    Step_Vector_Curr_Control(&motor_control);

}

/**
  * 函数功能:位置-电流环
  * 输入参数:
  * 返回参数:
  * 说    明: 
  */
void Step_Motor_Run_Pos_Current_Loop(void){
    /*电流采样*/
    Voltage_Value_Cal(&current_sample);
    Current_Cal(&current_sample);

    /*编码器位置和速度计算*/
    Speed_Feedback_Cal(&encoder);
    Encoder_Postion_Cal(&encoder_position);
    
//    /*脉冲位置给定*/
//    Pulse_Command_Cal(&motion_control);
    
    /*位置环*/
    position_loop.position_ref = position_loop.position_ref;
    position_loop.position_fbk = encoder_position.abs_pos;
    position_loop.position_dir = -motion_control.speed_ref_dir;
    Position_Loop_Cal(&position_loop);
    
    /*电流环SPWM*/
    motor_control.id_ref = 0;
    motor_control.iq_ref = position_loop.position_out;
    motor_control.theta_target = encoder_position.electrical_angle + encoder.theta_compensation;
    motor_control.volt_power_flt = current_sample.volt_power_flt;
    motor_control.curr_a = current_sample.curr_a;
    motor_control.curr_b = current_sample.curr_b;
    motor_control.out_d_offset = 0;
    motor_control.out_q_offset = 0;
    Step_Vector_Curr_Control(&motor_control);
}

/**
  * 函数功能:速度-电流环
  * 输入参数:
  * 返回参数:
  * 说    明: 
  */
void Step_Motor_Run_Speed_Current_Loop(void){
    /*电流采样*/
    Voltage_Value_Cal(&current_sample);
    Current_Cal(&current_sample);

    /*编码器位置和速度计算*/
    Encoder_Postion_Cal(&encoder_position);
    Speed_Feedback_Cal(&encoder);
    
    /*速度给定*/
    Pulse_Command_Cal(&motion_control);
    
    /*速度环*/
    speed_loop.speed_ref = motion_control.curr_ref_speed;
    speed_loop.speed_fbk = encoder.cur_speed_flt;
    speed_loop.speed_dir = -motion_control.speed_ref_dir;
    Speed_Loop_Cal(&speed_loop);

    /*电流环SPWM*/
    motor_control.id_ref = motion_control.curr_ref_speed == 0 ? 0.3f : 0;
    motor_control.iq_ref = speed_loop.speed_out;
    motor_control.theta_target = encoder_position.electrical_angle + encoder.theta_compensation;
    motor_control.volt_power_flt = current_sample.volt_power_flt;
    motor_control.curr_a = current_sample.curr_a;
    motor_control.curr_b = current_sample.curr_b;
    motor_control.out_d_offset = 0;
    motor_control.out_q_offset = 0;
    Step_Vector_Curr_Control(&motor_control);
}

/**
  * 函数功能:位置-速度-电流环
  * 输入参数:
  * 返回参数:
  * 说    明: 
  */
void Step_Motor_Run_Pos_Speed_Current_Loop(void){
    /*电流采样*/
    Voltage_Value_Cal(&current_sample);
    Current_Cal(&current_sample);

    /*编码器位置和速度计算*/
    Speed_Feedback_Cal(&encoder);
    Encoder_Postion_Cal(&encoder_position);
    
    /*位置模式自测程序*/
    Position_Mode_Test_Cal(&motion_comb, &position_control);
    
    /*位置梯形加减速设置*/
    Position_Control_Cal(&position_control, motion_control.pulse_abs_cnt, encoder_position.abs_pos);
    Position_Pulse_Set(&position_control, &motion_control);

    /*脉冲转换为位置和速度*/
    if(position_control.run_end != 1){
        Pulse_Command_Cal(&motion_control);
    }
    
    position_control.curr_position = motion_control.pulse_abs_cnt;
    position_control.au_position = encoder_position.abs_pos;

    /*位置环*/
    position_loop.position_ref = motion_control.pulse_abs_cnt * micro_varible.encoder_flt * micro_varible.micro_inv;
    position_loop.position_fbk = encoder_position.abs_pos;
    position_loop.position_dir = -motion_control.speed_ref_dir;
    Position_Loop_Cal(&position_loop);

    /*速度环*/
    speed_loop.speed_ref = position_loop.position_out;
    speed_loop.speed_fbk = encoder.cur_speed;
    speed_loop.speed_dir = -motion_control.speed_ref_dir;
    Speed_Loop_Cal(&speed_loop);

    /*电流环SPWM*/
    motor_control.id_ref = motion_control.curr_ref_speed == 0 ? 0 : 0;
    motor_control.iq_ref = speed_loop.speed_out;
    motor_control.theta_target = encoder_position.electrical_angle + encoder.theta_compensation;
    motor_control.volt_power_flt = current_sample.volt_power_flt;
    motor_control.curr_a = current_sample.curr_a;
    motor_control.curr_b = current_sample.curr_b;
    motor_control.out_d_offset = 0;
    motor_control.out_q_offset = 0;
    Step_Vector_Curr_Control(&motor_control);

}

/**
  * 函数功能:电流闭环D轴强拖
  * 输入参数:
  * 返回参数:
  * 说    明: 
  */
void Step_Motor_Run_Open_Loop(void){
    Voltage_Value_Cal(&current_sample);
    Current_Cal(&current_sample);
    
    Pulse_Command_Cal(&motion_control);

    /*电流环SPWM*/
    motor_control.id_ref = 1.5f;                                            //设置强拖电流
    motor_control.iq_ref = 0;
    motor_control.theta_target = encoder_position.electrical_angle;
    motor_control.volt_power_flt = current_sample.volt_power_flt;
    motor_control.curr_a = current_sample.curr_a;
    motor_control.curr_b = current_sample.curr_b;
    motor_control.out_d_offset = 0;
    motor_control.out_q_offset = 0;
    Step_Vector_Curr_Control(&motor_control);
}

/**
  * 函数功能:速度环参数初始化
  * 输入参数:
  * 返回参数:
  * 说    明: 
  */
void Speed_Loop_Cal_Init(Speed_Loop *p){
    p->speed_fbk = 0;
    p->speed_ref = 0;
    p->speed_kp = 0.00101f;//0.00172f;//0.002612f;//0.01f;//0.001f,0.005f
    p->speed_ki = 0.000005f;//0.00004f;//0.00001f
    p->speed_kd = 0;
    p->speed_out = 0;
    pid_handle_speed_loop.integral_lim = 400.0f;    //积分限幅
    pid_handle_speed_loop.out_lim = 2.5f;           //最大输出电流
    Pid_Control_Init(&pid_handle_speed_loop);
}

/**
  * 函数功能:速度环计算
  * 输入参数:
  * 返回参数:
  * 说    明: 
  */
void Speed_Loop_Cal(Speed_Loop *p){
    pid_handle_speed_loop.sign = p->speed_dir;
    pid_handle_speed_loop.err = p->speed_ref - p->speed_fbk;
    pid_handle_speed_loop.kp = p->speed_kp;
    pid_handle_speed_loop.ki = p->speed_ki;
    pid_handle_speed_loop.kd = p->speed_kd;
    if(run_param.run_mode != NONE_MODE){
        Pid_Control_Cal(&pid_handle_speed_loop);
        p->speed_out = pid_handle_speed_loop.out;
    }
}

/**
  * 函数功能:位置环参数初始化
  * 输入参数:
  * 返回参数:
  * 说    明: 
  */
void Position_Loop_Cal_Init(Position_Loop *p){
    p->position_fbk = 0;
    p->position_ref = 0;
    p->position_kp = run_param.run_mode == POS_CURR_CLOSE_LOOP ? 0.009f : 3.5f;
    p->position_ki = 0;
    p->position_kd = 0;
    p->position_out = 0;
    pid_handle_position_loop.integral_lim = 10000.0f;
    pid_handle_position_loop.out_lim = run_param.run_mode == POS_CURR_CLOSE_LOOP ? 1.0f : 900.0f;
    Pid_Control_Init(&pid_handle_position_loop);
}


/**
  * 函数功能:位置环计算
  * 输入参数:
  * 返回参数:
  * 说    明: 
  */
void Position_Loop_Cal(Position_Loop *p){
    pid_handle_position_loop.sign = p->position_dir;
    pid_handle_position_loop.err = p->position_ref - p->position_fbk;
    pid_handle_position_loop.kp = p->position_kp;
    pid_handle_position_loop.ki = p->position_ki;
    pid_handle_position_loop.kd = p->position_kd;
    Pid_Control_Cal(&pid_handle_position_loop);
    p->position_out = pid_handle_position_loop.out;
}
