#ifndef __ELE_ANGLE_ADJUST_H
#define __ELE_ANGLE_ADJUST_H

#include "public.h"

#define STEP_NUM  200

typedef  struct {
    uint8_t adjust_enable;
    uint8_t adjust_status;
    uint16_t adjust_data[STEP_NUM+1];
    uint16_t adjust_dataR[STEP_NUM+1];
    uint32_t adjust_step_out;
    uint16_t zero_x;
    uint16_t zero_y;
    int32_t count;

}Electrical_Angle_Adjust;

extern Electrical_Angle_Adjust ele_angle_adjust;

void Motor_Open_Run(int32_t step, float curr_mag);
void Motor_Adjust(void);
void Motor_Adjust_Data_Cal(void);

#endif
