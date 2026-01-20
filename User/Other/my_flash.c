/**
  ******************************************************************************
  * 文件名称: 
  * 作    者: √
  * 版    本: V1.0
  * 编写日期: 
  * 功    能: 片内Flash擦写
  ******************************************************************************
  */
  
/* 包含头文件 ----------------------------------------------------------------*/
#include "my_flash.h"

FLASH_EraseInitTypeDef erase_struct;

/**
  * 函数功能: 擦除指定地址开始的指定页数的数据
  * 输入参数:
  * 返回参数:
  * 说    明:
  */
void Flash_Erase(uint32_t start_address, uint16_t page_num){
    uint32_t page_error = 0;
    erase_struct.TypeErase   = FLASH_TYPEERASE_PAGES;
    erase_struct.Page = (start_address - FLASH_BASE) / FLASH_PAGE_SIZE;
    erase_struct.NbPages = page_num;
    erase_struct.Banks = FLASH_BANK_1;
    __HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_OPTVERR);                             //清除错误标志
    for(uint16_t i = 0; i < 10; i++){                                       //防止擦除失败，多擦除几次
        if(HAL_FLASHEx_Erase(&erase_struct, &page_error)){
            break;
        }
    }
}

/**
  * 函数功能: 擦除编码器校准的数据
  * 输入参数:
  * 返回参数:
  * 说    明:起始地址 FLASH_USER_START_ADDR 16页大小为16 * 2048 = 32768Byte, 14位绝对值编码器数据个数为16384，每个数据需2Byte存储
  */
void Flash_Erase_Angle_Data(void){
  HAL_FLASH_Unlock();
  Flash_Erase(FLASH_USER_START_ADDR, 16);
  HAL_FLASH_Lock();
}

/**
  * 函数功能: 擦除系统参数数据
  * 输入参数:
  * 返回参数:
  * 说    明:起始地址 FLASH_USER_PARAM_ADDR 擦除一页数据 可保存1024个2Byte参数
  */
void Flash_Erase_Param_Data(void){
  HAL_FLASH_Unlock();
  Flash_Erase(FLASH_USER_PARAM_ADDR, 1);
  HAL_FLASH_Lock();
}

/**
  * 函数功能: 解锁Falsh
  * 输入参数: 
  * 返回参数: 
  * 说    明: 写操作前必须先解锁
  */
void Flash_Write_Data_Begin(void){
  HAL_FLASH_Unlock();
}

/**
  * 函数功能: 锁定Falsh
  * 输入参数: 
  * 返回参数: 
  * 说    明: 写操作后必须锁定
  */
void  Flash_Write_Data_End(void){
  HAL_FLASH_Lock();
}

/**
  * 函数功能: 连续写16位数据（只适合写入8Byte整数倍的数据）
  * 输入参数: 
  * 返回参数: 
  * 说    明: 调用前必须先调用Flash_Write_Data_Begin，结束时必须调用Flash_Write_Data_End
  */
uint16_t  pBuffer[4];
void Flash_Write_Data16(uint16_t sdata){
    static uint32_t address = FLASH_USER_START_ADDR;
    static uint8_t i = 0;
    i++;
    pBuffer[i-1] = sdata;
    if(i < 4){                  //不足8Byte不写入
        return;
    }
    uint64_t data = ((uint64_t)pBuffer[3]<<48) | ((uint64_t)pBuffer[2]<<32) | ((uint64_t)pBuffer[1]<<16)  | ((uint64_t)pBuffer[0]);
    for(uint16_t j = 0; j < 10; j++){                                                                                               //尝试写入10次失败退出
        if(HAL_FLASH_Program(TYPEPROGRAM_DOUBLEWORD, address, data) == HAL_OK){
            address = address + 8;
            i = 0;
            break;
        }
    }
}

/**
  * 函数功能: 写系统参数
  * 输入参数: 
  * 返回参数: 
  * 说    明: 内部已有解锁、锁定操作
  */
void Flash_Write_Param(uint16_t *data, uint16_t data_num){
    uint32_t address = FLASH_USER_PARAM_ADDR;
    uint8_t write_times = data_num / 4;                 //执行写入次数  G4系列必须以64位数据写入一次
    uint8_t over_num = data_num % 4 ;                   //结尾剩余数据量
    if(over_num != 0){
        write_times++;                                  //结尾有剩余则多执行一次写入
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
                    data_buff[j] = 0XFF;            //结尾数据不足64位的置为1
                }
                
            }
        }
        /*准备写入数据*/
        HAL_FLASH_Unlock();
        uint64_t temp_data = ((uint64_t)data_buff[3]<<48) | ((uint64_t)data_buff[2]<<32) | ((uint64_t)data_buff[1]<<16)  | ((uint64_t)data_buff[0]);
        for(uint16_t j = 0; j < 10; j++){                                                                                                               //尝试写入10次失败退出
            if(HAL_FLASH_Program(TYPEPROGRAM_DOUBLEWORD, address, temp_data) == HAL_OK){
                address = address + 8;
                break;
            }
        }
        HAL_FLASH_Lock();
    }
}

