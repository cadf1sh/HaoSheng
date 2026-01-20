#ifndef __FLASH_RW_H__
#define __FLASH_RW_H__

#include "public.h"

#define                 FLASH_USER_START_ADDR               0x0800A000 
#define                 FLASH_PARAM_START_ADDR              0x0800E000

void Flash_Write_Data_Float(float fdata);

void Flash_Write_Data(uint32_t address, uint16_t *data, uint16_t data_num);
void Flash_Read_Data(uint32_t address, uint16_t *data, uint16_t data_num);

void Flash_Write_Angle_Data(uint32_t address, float *data, uint16_t data_num);
void Flash_Read_Angle_Data(float *data);
void Flash_Write_Param(uint16_t *data, uint16_t data_num);

void Flash_Write_One_Data(uint32_t address, uint16_t data[4]);
void Flash_Read_One_Data(uint32_t address, uint16_t data[4]);
uint16_t flash_read_dir(int addr);

#endif 
