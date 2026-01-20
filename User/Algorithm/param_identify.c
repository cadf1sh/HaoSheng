/**
  ******************************************************************************
  * 文件名称: 
  * 作    者: √
  * 版    本: V1.0
  * 编写日期: 
  * 功    能: 电阻、电感辨识，电流环参数整定
  ******************************************************************************
  */
  
/* 包含头文件 ----------------------------------------------------------------*/
#include "param_identify.h"
#include "foc_control.h"
#include "transform.h"
#include "ele_angle_adjust.h"


Param_Identify param_ident;                     //参数辨识
Pack_Handle ident_pack_handle;

/**
  * 函数功能:结构体初始化
  * 输入参数:
  * 返回参数:
  * 说    明: 
  */
void Param_Ident_Init(Param_Identify *p){
    p->ident_stage = FIRST;
    p->volt_power_flt = 0;
    p->ident_curr = 0.3f;
    p->curr_a = 0;
    p->curr_b = 0;
    p->d_volt = 0;
    p->curr_d = 0;
    p->curr_d_flt = 0;
    p->curr_q = 0;
    p->curr_q_flt = 0;
    p->motor_power = 0;
    p->fir_volt = 0;
    p->fir_curr = 0;
    p->sec_volt = 0;
    p->sec_curr = 0;
    p->motor_r = 0;
    p->motor_ld = 0;
    p->motor_lq = 0;
    p->curr_kp = 0;
    p->curr_ki = 0;
    p->motor_rl = 0;
    p->ident_theta = 0;
    p->pwm_sw = 0;
    p->wait_time = 0;
    p->motor_l_cal_cnt = 0;
    p->motor_l_cal_flag = 0;
}


/**
  * 函数功能:参数辨识，电流环参数整定
  * 输入参数:
  * 返回参数:
  * 说    明: 
  */
void Param_Ident(Param_Identify *p){

    ident_pack_handle.alpha_val = p->curr_a;
    ident_pack_handle.beta_val = p->curr_b;
    
    ident_pack_handle.cos_val = arm_cos_f32(p->ident_theta * tpi);
    ident_pack_handle.sin_val = arm_sin_f32(p->ident_theta * tpi);
    
    Pack_Transform(&ident_pack_handle);
    
    p->curr_d = ident_pack_handle.d_val;
    p->curr_d_flt += 0.1f * (p->curr_d - p->curr_d_flt);
    p->curr_q = ident_pack_handle.q_val;
    p->curr_q_flt += 0.1f * (p->curr_q - p->curr_q_flt);
    
    p->motor_power = 1.5f * p->curr_d_flt * p->d_volt;                                      //P=3/2 * Ud * Id    P = I * U 
    
    if(p->ident_stage == FIRST){                                                            //一阶段逐渐增加电压直到电流达到0.6倍的设定电流
        if(p->motor_power / p->volt_power_flt >= 0.6f * p->ident_curr){
            p->wait_time++;
            p->fir_volt = p->d_volt;
            if(p->wait_time >= 3000 && p->wait_time <= 3099){                               //等待3000个控制周期后连续记录100次电流值
                p->fir_curr += p->curr_d_flt;
            }else if(p->wait_time > 3099){                                                  //求出平均电流
                p->fir_curr *= 0.01f;
                p->wait_time = 0;
                p->ident_stage = SECOND;
            }
        }else{
            p->d_volt += 0.001f;
        }
    }else if(p->ident_stage == SECOND){                                                     //二阶段逐渐增加电压直到电流达到设定电流
        if(p->motor_power / p->volt_power_flt >= p->ident_curr){
            p->wait_time++;
            p->sec_volt = p->d_volt;
            if(p->wait_time >= 3000 && p->wait_time <= 3099){
                p->sec_curr += p->curr_d_flt;
            }else if(p->wait_time > 3099){
                p->sec_curr *= 0.01f;
                p->motor_r = (p->sec_volt - p->fir_volt) / (p->sec_curr - p->fir_curr);     //根据电压变化量和电流变化量计算电阻值R = ΔU / ΔI
                p->wait_time = 0;
                p->ident_stage = THIRD;
            }
        }else{
            p->d_volt += 0.001f;
        }
    }else if(p->ident_stage == THIRD){
        if(p->motor_l_cal_flag == 0){                                                           //关闭电压输出
            p->d_volt = 0;
        }
        if((fabsf(p->curr_d) < 0.08f && p->motor_l_cal_flag == 0) || p->motor_l_cal_flag == 1){ //等待电流为0
            p->motor_l_cal_flag = 1;
            p->d_volt = p->sec_volt;                                                            //直接给定二阶段电压值
            p->wait_time++;
            if(p->curr_d >= p->sec_curr * 0.95f){                                               //等待电流值达到0.95倍的二阶段电流值，记录时间
                p->motor_ld += p->motor_r * p->wait_time * INVT_PRD / 3.0f;                     //根据公式计算电感值
                p->wait_time = 0;
                p->motor_l_cal_cnt++;
                p->motor_l_cal_flag = 0;
            }
            if(p->motor_l_cal_cnt >= 100){                                                        //取多次值求平均
                p->motor_ld *= 0.01f;
                p->motor_lq = p->motor_ld;
                p->motor_rl = p->motor_r / p->motor_ld;
                p->curr_kp = run_param.curr_loop_band * tpi * p->motor_ld / p->volt_power_flt;
                p->curr_ki = p->curr_kp * p->motor_rl * INVT_PRD * 0.01f;
                run_param.curr_kp = p->curr_kp;
                run_param.curr_ki = p->curr_ki;
                run_param.ident_flag = 1;
            }
        }
    }
    ident_pack_handle.d_val = p->d_volt;
    ident_pack_handle.q_val = 0;
    
    IPack_Transform(&ident_pack_handle);
    
    motor_control.out_a = ident_pack_handle.alpha_val / p->volt_power_flt;
    motor_control.out_b = ident_pack_handle.beta_val / p->volt_power_flt;
    Step_Vector_Curr_Control(&motor_control);
    
}

/**
  * 函数功能:编码器校准流程控制
  * 输入参数:
  * 返回参数:
  * 说    明: 
  */
void Encoder_Adjust_Run(void){
    if(run_param.enc_adjust_flag != 1){                                                     //未经过校准
        Motor_Adjust();
        if(ele_angle_adjust.adjust_status == 6){
            __HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_1,PWM_HALF_PRD);
            __HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_2,PWM_HALF_PRD);
            __HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_3,PWM_HALF_PRD);
            __HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_4,PWM_HALF_PRD);
            Motor_Adjust_Data_Cal();
            if(ele_angle_adjust.adjust_status == 0 && ele_angle_adjust.adjust_enable == 0){
                run_param.enc_adjust_flag = 1;
            }
            
        }
    }else{
        run_param.enc_adjust_flag = 1;
    }

}
