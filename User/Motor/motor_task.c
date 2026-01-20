/**
  ******************************************************************************
  * 文件名称: 
  * 作    者: √
  * 版    本: V1.0
  * 编写日期: 
  * 功    能: 电机运行流程控制
  ******************************************************************************
  */
  
/* 包含头文件 ----------------------------------------------------------------*/
#include "motor_task.h"
#include "encoder.h"
#include "motion_control.h"
#include "current_sample.h"
#include "param_identify.h"
#include "foc_control.h"
#include "motor_run_mode.h"
#include "my_flash.h"
#include "position_mode_test.h"

/**
  * 函数功能:电机运行
  * 输入参数:
  * 返回参数:
  * 说    明: 
  */
void Motor_Run_Task(void){
        static uint32_t init_time = 0;
        Adc_Value_Get(&current_sample);
        switch(run_param.sys_status){
            case REF:{                                                                        //电流采样偏置电压计算
                Voltage_Value_Cal(&current_sample);
                Voltage_Ref_Cal(&current_sample);
            }break;
            case IDENT:{
                Voltage_Value_Cal(&current_sample);
                Current_Cal(&current_sample);
                if(run_param.ident_flag != 1){                                              //电机参数辨识+电流环参数整定
                    param_ident.volt_power_flt = current_sample.volt_power_flt;
                    param_ident.curr_a = current_sample.curr_a;
                    param_ident.curr_b = current_sample.curr_b;
                    Param_Ident(&param_ident);
                }else{                                                                      //整定完成，赋值给后续要使用的结构体
                    motor_control.curr_kp_d = run_param.curr_kp;
                    motor_control.curr_ki_d = run_param.curr_ki;
                    motor_control.curr_kp_q = run_param.curr_kp;
                    motor_control.curr_ki_q = run_param.curr_ki;
                    if(run_param.enc_adjust_flag != 1){                                          //编码器未校准
                        init_time++;
                        if(init_time < 20000){                                                  //强拖到 pi/2 位置
                            motor_control.id_ref = 0.5f;
                            motor_control.iq_ref = 0;
                            motor_control.theta_target = 0.25f;
                            motor_control.volt_power_flt = current_sample.volt_power_flt;
                            motor_control.curr_a = current_sample.curr_a;
                            motor_control.curr_b = current_sample.curr_b;
                            Step_Vector_Curr_Control(&motor_control);
                        }else if(init_time < 40000){                                            //强拖到 0 位置
                            motor_control.id_ref = 0.5f;
                            motor_control.iq_ref = 0;
                            motor_control.theta_target = 0;
                            motor_control.volt_power_flt = current_sample.volt_power_flt;
                            motor_control.curr_a = current_sample.curr_a;
                            motor_control.curr_b = current_sample.curr_b;
                            Step_Vector_Curr_Control(&motor_control);
                        }else{                                                                  //编码器校准
                            if(init_time <= 40000){
    //                            HAL_TIM_Base_Start(motion_control.cnt_timer);
    //                            motion_control.cnt_timer->Instance->CNT = 0;
    //                            HAL_TIM_Encoder_Start(encoder.cnt_timer, TIM_CHANNEL_ALL);
                            }
                            Encoder_Adjust_Run();                                                //校准函数
                            if(run_param.enc_adjust_flag == 1){
                                run_param.sys_status = RUN;
                                Abs_Encoder_Init();
                            }
                            init_time = 400001;
                        }
                    }else{
    //                        HAL_TIM_Base_Start(motion_control.cnt_timer);
    //                        motion_control.cnt_timer->Instance->CNT = 0;
    //                        HAL_TIM_Encoder_Start(encoder.cnt_timer, TIM_CHANNEL_ALL);
                            Encoder_Adjust_Run();                                                //校准函数
                            run_param.sys_status = RUN;
                            Abs_Encoder_Init();
                    }
                }
            }break;
            case RUN:{                                                                          //运行
                switch(run_param.run_mode){
                    case NONE_MODE:{
                        run_param.sys_status = STOP;
                        run_param.run_mode_last = NONE_MODE;
                        run_param.enable_flag = 0;
                    }break;
                    case OPEN_LOOP:{
                        run_param.run_mode = run_param.run_mode_last;
                        Step_Motor_Run_Open_Loop();
                    }break;
                    case CURR_CLOSE_LOOP:{
                        run_param.run_mode = run_param.run_mode_last;
                        Step_Motor_Run_Current_Loop();
                    }break;
                    case POS_CURR_CLOSE_LOOP:{
                        run_param.run_mode = run_param.run_mode_last;
                        Step_Motor_Run_Pos_Current_Loop();
                    }break;
                    case SPEED_CURR_CLOSE_LOOP:{
                        run_param.run_mode = run_param.run_mode_last;
                        Step_Motor_Run_Speed_Current_Loop();
                    }break;
                    case POS_SPEED_CURR_CLOSE_LOOP:{
                        run_param.run_mode = run_param.run_mode_last;
                        Step_Motor_Run_Pos_Speed_Current_Loop();
                    }break;
                }
                if(run_param.enable_flag == 0){
                    run_param.sys_status = STOP;
                }
    //            /*rtt打印数据*/
    //            rtt_data[0] = motor_control.alpha_val_out * 100;
    //            rtt_data[1] = motor_control.beta_val_out * 100;
    //            rtt_data[2] = motor_control.a_positive.cmp_val;
    //            rtt_data[3] = motor_control.a_negative.cmp_val;
    //            rtt_data[4] = motor_control.b_positive.cmp_val;
    //            SEGGER_RTT_Write(1, &rtt_data, 20);                
            }break;
            case FAULT:{                                                                          //异常
                Voltage_Value_Cal(&current_sample);
                Current_Cal(&current_sample);
                __HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_1,PWM_HALF_PRD);
                __HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_2,PWM_HALF_PRD);
                __HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_3,PWM_HALF_PRD);
                __HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_4,PWM_HALF_PRD);
            }break;
            case STOP:{                                                                          //失能
                __HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_1,PWM_HALF_PRD);
                __HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_2,PWM_HALF_PRD);
                __HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_3,PWM_HALF_PRD);
                __HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_4,PWM_HALF_PRD);
                if(run_param.enable_flag == 1){
                    run_param.sys_status = RUN;
                    if(run_param.run_mode_last != run_param.run_mode){                      //模式切换重新初始化变量
                        Change_Run_Mode_Init();
                    }
                    run_param.run_mode_last = run_param.run_mode;
                }
                Voltage_Value_Cal(&current_sample);
                Current_Cal(&current_sample);
                Encoder_Postion_Cal(&encoder_position);                
            }break;
            
        }    
}

void Change_Run_Mode_Init(void){
      Position_Loop_Cal_Init(&position_loop);
    Speed_Loop_Cal_Init(&speed_loop);
    
    Position_Motion_Comb_Init(&motion_comb);
    
    Position_Control_Init(&position_control);
    Motion_Control_Init(&motion_control);
    
    Motor_Control_Init(&motor_control);
}
