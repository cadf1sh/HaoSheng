#include "flash_rw.h"

#pragma pack(4)
typedef struct
{
    uint16_t data1;
    uint16_t data2;
    uint16_t data3;
    uint16_t data4;
}Data_Unit, *P_Data_Unit;
#pragma pack()

Data_Unit data_unit;
P_Data_Unit p_data_unit;

void Flash_Erase(uint32_t address, uint8_t erase_page_num){
    FLASH_EraseInitTypeDef EraseInitStruct;
    EraseInitStruct.TypeErase = FLASH_TYPEERASE_PAGES;                                  //页擦除
    EraseInitStruct.Banks = FLASH_BANK_1;
    EraseInitStruct.Page = (address - FLASH_BASE) / FLASH_PAGE_SIZE;      //从第几个页开始擦除（当前地址FLASH_USER_START_ADDR开始）
    EraseInitStruct.NbPages = erase_page_num;                                                        //擦除多少个页
    uint32_t PageError = 0;                                                             //记录擦除出错时的起始地址
    
    HAL_FLASH_Unlock();
    for(uint16_t i = 0; i < 10; i++){
        if(HAL_FLASHEx_Erase(&EraseInitStruct, &PageError)){
            break;
        }
    }
    HAL_FLASH_Lock();
}

void Flash_Write_Data(uint32_t address, uint16_t *data, uint16_t data_num){
    Flash_Erase(FLASH_USER_START_ADDR, 5);
    
    HAL_FLASH_Unlock();
    for(uint16_t i = 0; i < data_num / 4; i++){
        data_unit.data1 = data[i * 4];
        data_unit.data2 = data[i * 4 + 1];
        data_unit.data3 = data[i * 4 + 2];
        data_unit.data4 = data[i * 4 + 3];
        for(uint16_t j = 0; j < 10; j++){                                                               //尝试写入10次失败退出
            if(HAL_FLASH_Program(TYPEPROGRAM_DOUBLEWORD, address, *(uint64_t *)&data_unit) == HAL_OK){
                break;
            }
        }
        address += sizeof(uint64_t);
    }
    HAL_FLASH_Lock();
}
uint16_t  pBuffer[4];
void Flash_Write_Data_Float(float fdata){
  static uint32_t Address = FLASH_USER_START_ADDR;
  static uint8_t i = 0;
  i++;
  pBuffer[i-1] = fdata / 0.0000152587890625f;

  if (i<4) return;
  uint64_t data = ((uint64_t)pBuffer[3]<<48) | ((uint64_t)pBuffer[2]<<32) | ((uint64_t)pBuffer[1]<<16)  | ((uint64_t)pBuffer[0]);
  if(HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD, Address, data) == HAL_OK)
  {
    Address = Address + 8;  //地址后移4个字节
    i = 0;
  }
}
void Flash_Read_Data(uint32_t address, uint16_t *data, uint16_t data_num){
    for(uint16_t i = 0; i < data_num / 4; i++){
        p_data_unit = (P_Data_Unit)(address + sizeof(uint64_t) * i);
        data[i * 4] = p_data_unit->data1;
        data[i * 4 + 1] = p_data_unit->data2;
        data[i * 4 + 2] = p_data_unit->data3;
        data[i * 4 + 3] = p_data_unit->data4;
    }
}

void Flash_Write_Angle_Data(uint32_t address, float *data, uint16_t data_num){
    Flash_Erase(FLASH_USER_START_ADDR, 5);
    HAL_FLASH_Unlock();
    for(uint16_t i = 0; i < data_num / 4; i++){
        data_unit.data1 = (uint16_t)(data[i * 4] / 0.0000152587890625f);
        data_unit.data2 = (uint16_t)(data[i * 4 + 1] / 0.0000152587890625f);
        data_unit.data3 = (uint16_t)(data[i * 4 + 2] / 0.0000152587890625f);
        data_unit.data4 = (uint16_t)(data[i * 4 + 3] / 0.0000152587890625f);
        for(uint16_t j = 0; j < 10; j++){                                                               //尝试写入10次失败退出
            if(HAL_FLASH_Program(TYPEPROGRAM_DOUBLEWORD, address, *(uint64_t *)&data_unit) == HAL_OK){
                break;
            }
        }
        address += sizeof(uint64_t);
    }
    HAL_FLASH_Lock();
}
void Flash_Read_Angle_Data(float *data){
    for(uint16_t i = 0; i < 1024; i++){
        p_data_unit = (P_Data_Unit)(FLASH_USER_START_ADDR + sizeof(uint64_t) * i);
        data[i * 4] = p_data_unit->data1 * 0.0000152587890625f;
        data[i * 4 + 1] = p_data_unit->data2 * 0.0000152587890625f;
        data[i * 4 + 2] = p_data_unit->data3 * 0.0000152587890625f;
        data[i * 4 + 3] = p_data_unit->data4 * 0.0000152587890625f;
    }
}

void Flash_Write_Param(uint16_t *data, uint16_t data_num){
    uint8_t write_times = data_num / 4;
    uint8_t over_num = data_num % 4 ;
    if(over_num != 0){
        write_times++;
    }
    uint16_t data_buff[4] = {0};
    for(uint16_t i = 0; i < write_times; i++){
        if(i < write_times - over_num -1){
            data_buff[0] = data[i * 4];
            data_buff[1] = data[i * 4 + 1];
            data_buff[2] = data[i * 4 + 2];
            data_buff[3] = data[i * 4 + 3];
        }else{
            for(uint8_t j = 0; j < 4; j++){
                if(j < over_num){
                    data_buff[j] = data[i * 4 + j];
                }else{
                    data_buff[j] = 0XFF;
                }
                
            }
        }
        Flash_Write_One_Data(FLASH_PARAM_START_ADDR,data_buff);
    }
}

/*写入一个uint64数据*/
void Flash_Write_One_Data(uint32_t address, uint16_t data[4]){
    HAL_FLASH_Unlock();
    data_unit.data1 = data[0];
    data_unit.data2 = data[1];
    data_unit.data3 = data[2];
    data_unit.data4 = data[3];
    for(uint16_t j = 0; j < 10; j++){                                                               //尝试写入10次失败退出
        if(HAL_FLASH_Program(TYPEPROGRAM_DOUBLEWORD, address, *(uint64_t *)&data_unit) == HAL_OK){
            break;
        }
    }
    HAL_FLASH_Lock();
}

/*读取一个uint64数据*/
void Flash_Read_One_Data(uint32_t address, uint16_t data[4]){
    p_data_unit = (P_Data_Unit)address;
    data[0] = p_data_unit->data1;
    data[1] = p_data_unit->data2;
    data[2] = p_data_unit->data3;
    data[3] = p_data_unit->data4;
}

