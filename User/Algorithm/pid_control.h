#ifndef PID_CONTROL_H
#define PID_CONTROL_H

#include "public.h"

typedef struct{
    int8_t sign;
    int8_t sign_last;
    float kp;
    float ki;
    float kd;
    float err;
    float err_last;
    float err_integral;
    float err_delta;
    float integral_lim;
    float out;
    float out_lim;
}Pid_Control_Handle;


void Pid_Control_Init(Pid_Control_Handle *p);
void Pid_Control_Cal(Pid_Control_Handle *p);
#endif
