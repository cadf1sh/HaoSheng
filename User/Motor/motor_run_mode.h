#ifndef __MOTOR_RUN_MODE_H 
#define __MOTOR_RUN_MODE_H 

#include "public.h"
#include "pid_control.h"

typedef struct{
    int8_t speed_dir;
    float speed_fbk;
    float speed_ref;
    float speed_kp;
    float speed_ki;
    float speed_kd;
    float speed_out;
}Speed_Loop;

typedef struct{
    int8_t position_dir;
    float position_fbk;
    float position_ref;
    float position_kp;
    float position_ki;
    float position_kd;
    float position_out;
}Position_Loop;

extern Speed_Loop speed_loop;
extern Position_Loop position_loop;

void Speed_Loop_Cal_Init(Speed_Loop *p);
void Speed_Loop_Cal(Speed_Loop *p);

void Position_Loop_Cal_Init(Position_Loop *p);
void Position_Loop_Cal(Position_Loop *p);

void Step_Motor_Run_Current_Loop(void);
void Step_Motor_Run_Pos_Current_Loop(void);
void Step_Motor_Run_Speed_Current_Loop(void);
void Step_Motor_Run_Pos_Speed_Current_Loop(void);
void Step_Motor_Run_Open_Loop(void);

#endif


