/**
  ******************************************************************************
  * 文件名称: 
  * 作    者: √
  * 版    本: V1.0
  * 编写日期: 
  * 功    能: 系统运行公共变量、函数等
  ******************************************************************************
  */
  
/* 包含头文件 ----------------------------------------------------------------*/
#include "public.h"

Micro_Varible micro_varible;
System_Run_Param run_param;

/**
  * 函数功能:取浮点类型的小数部分
  * 输入参数:
  * 返回参数:
  * 说    明: 
  */
float thetafrac(float theta_t){
    int32_t a;
    a = theta_t;
    theta_t -= a;
    return theta_t;
}

/**
  * 函数功能:数据限幅
  * 输入参数:
  * 返回参数:
  * 说    明: 
  */
float sat(float value, float min, float max){
    if(value >= max){
        return max;
    }
    if(value <= min){
        return min;
    }
    return value;
}

/**
  * 函数功能:开环细分量纲和编码器量纲标幺、转换中间变量初始化
  * 输入参数:
  * 返回参数:
  * 说    明: 
  */
void Micro_Varible_Init(Micro_Varible *p){
    p->micro_slt = 4.0f * MICRO_NUM;
    p->micro_flt = p->micro_slt;
    p->micro_inv = 1.0f / p->micro_flt;
    p->encoder_slt = (float)ENCODER_LINE / (float)MOTOR_POLES;
    p->encoder_flt = p->encoder_slt;
    p->encoder_inv = 1.0f / p->encoder_flt;
}

/**
  * 函数功能:系统运行状态变量初始化（System_Run_Param 结构体监测系统状态）
  * 输入参数:
  * 返回参数:
  * 说    明: 
  */
void System_Run_Param_Init(System_Run_Param *p){
    p->motor_type = STEP;
    p->sys_status = REF;
    p->sys_err_code = NONE;
    p->run_mode = POS_SPEED_CURR_CLOSE_LOOP;        //参数辨识完成后的默认运行模式设置（可修改）
    p->run_mode_last = p->run_mode;
    p->motor_state = NORMAL_RUN;
    p->enable_flag = 1;
    p->mos_temperature = 25.0f;
    p->enc_adjust_flag = 0;
    p->ident_flag = 0;
    p->curr_kp = 0.001f;
    p->curr_ki = 0.00001f;
    p->curr_loop_band = 1200.0f;//600;
    p->power_volt = 0;
    p->reset_angle_data = 0;                        //1 运行编码器非线性校准   0 不运行校准
}

