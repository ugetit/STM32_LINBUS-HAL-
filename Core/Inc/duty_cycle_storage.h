#ifndef __DUTY_CYCLE_STORAGE_H
#define __DUTY_CYCLE_STORAGE_H

#include "stm32f1xx_hal.h"

typedef struct {
    uint32_t initialized_flag;
    float max_duty_cycle;
    float min_duty_cycle;
} DutyCycleData;

#define DEFAULT_MAX_DUTY_CYCLE 100.0f
#define DEFAULT_MIN_DUTY_CYCLE 0.0f
#define INITIALIZED_FLAG       0xA5A5A5A5

void InitDutyCycleData(void);
void UpdateDutyCycleData(float maxDutyCycle, float minDutyCycle);
void Flash_ReadDutyCycleData(DutyCycleData* data);
void Flash_WriteDutyCycleData(DutyCycleData* data);

#endif // __DUTY_CYCLE_STORAGE_H
