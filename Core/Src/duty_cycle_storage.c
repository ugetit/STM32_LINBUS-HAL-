#include "duty_cycle_storage.h"

#define FLASH_USER_START_ADDR   ((uint32_t)0x0801FC00)   // STM32F103C8T6最后一页的起始地址

static DutyCycleData dutyCycleData;

void Flash_ReadDutyCycleData(DutyCycleData* data) {
    uint32_t address = FLASH_USER_START_ADDR;
    // 从 Flash 读取数据到结构体中
    data->initialized_flag = *(uint32_t*)address;
    data->max_duty_cycle = *(float*)(address + sizeof(uint32_t));
    data->min_duty_cycle = *(float*)(address + sizeof(uint32_t) + sizeof(float));
}

void Flash_WriteDutyCycleData(DutyCycleData* data) {
    HAL_FLASH_Unlock();

    // 擦除最后一页
    FLASH_EraseInitTypeDef eraseInitStruct;
    uint32_t pageError = 0;
    eraseInitStruct.TypeErase = FLASH_TYPEERASE_PAGES;
    eraseInitStruct.PageAddress = FLASH_USER_START_ADDR;
    eraseInitStruct.NbPages = 1;

    if (HAL_FLASHEx_Erase(&eraseInitStruct, &pageError) != HAL_OK) {
        // 处理错误
        HAL_FLASH_Lock();
        return;
    }

    uint32_t address = FLASH_USER_START_ADDR;

    // 将数据写入 Flash
    HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, address, data->initialized_flag);
    HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, address + sizeof(uint32_t), *(uint32_t*)&data->max_duty_cycle);
    HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, address + sizeof(uint32_t) + sizeof(float), *(uint32_t*)&data->min_duty_cycle);

    HAL_FLASH_Lock();
}

void InitDutyCycleData(void) {
    // 从 Flash 读取数据
    Flash_ReadDutyCycleData(&dutyCycleData);

    // 检查是否已经初始化过
    if (dutyCycleData.initialized_flag != INITIALIZED_FLAG) {
        // 数据未初始化，写入默认值
        dutyCycleData.initialized_flag = INITIALIZED_FLAG;
        dutyCycleData.max_duty_cycle = DEFAULT_MAX_DUTY_CYCLE;
        dutyCycleData.min_duty_cycle = DEFAULT_MIN_DUTY_CYCLE;
        Flash_WriteDutyCycleData(&dutyCycleData);
    }
}

void UpdateDutyCycleData(float maxDutyCycle, float minDutyCycle) {
    // 如果数据有变化，则写入新值
    if (dutyCycleData.max_duty_cycle != maxDutyCycle || dutyCycleData.min_duty_cycle != minDutyCycle) {
        dutyCycleData.max_duty_cycle = maxDutyCycle;
        dutyCycleData.min_duty_cycle = minDutyCycle;
        Flash_WriteDutyCycleData(&dutyCycleData);
    }
}
