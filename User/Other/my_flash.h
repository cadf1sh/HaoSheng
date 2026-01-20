#ifndef __MY_FLASH_H_
#define __MY_FLASH_H_
#include "public.h"

#define STM32_FLASH_SIZE    128     //所选STM32的FLASH容量大小(单位为K)
#define STM_SECTOR_SIZE	    2048    //2K字节

void Flash_Erase(uint32_t start_address, uint16_t page_num);
void Flash_Erase_Angle_Data(void);
void Flash_Erase_Param_Data(void);
void Flash_Write_Data_Begin(void);
void Flash_Write_Data16(uint16_t sdata);
void Flash_Write_Data_End(void);
void Flash_Write_Param(uint16_t *data, uint16_t data_num);

#endif

