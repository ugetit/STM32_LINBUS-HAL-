/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    stm32f1xx_it.c
  * @brief   Interrupt Service Routines.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f1xx_it.h"
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "lin_driver.h"  
#include "stdio.h"
#include "linbuffer.h"
#include "stdlib.h"
#include "string.h"
#include "stdio.h"
#include "duty_cycle_storage.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN TD */

/* USER CODE END TD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
extern LIN_RxState LIN_RxStateGet;
extern LIN_BUFFER  LIN_RxDataBuff;
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
extern uint8_t buffer[256];
extern float My_Max_PWM,My_Min_PWM;
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/* External variables --------------------------------------------------------*/
extern DMA_HandleTypeDef hdma_usart1_rx;
extern UART_HandleTypeDef huart1;
extern UART_HandleTypeDef huart3;
/* USER CODE BEGIN EV */

/* USER CODE END EV */

/******************************************************************************/
/*           Cortex-M3 Processor Interruption and Exception Handlers          */
/******************************************************************************/
/**
  * @brief This function handles Non maskable interrupt.
  */
void NMI_Handler(void)
{
  /* USER CODE BEGIN NonMaskableInt_IRQn 0 */

  /* USER CODE END NonMaskableInt_IRQn 0 */
  /* USER CODE BEGIN NonMaskableInt_IRQn 1 */
   while (1)
  {
  }
  /* USER CODE END NonMaskableInt_IRQn 1 */
}

/**
  * @brief This function handles Hard fault interrupt.
  */
void HardFault_Handler(void)
{
  /* USER CODE BEGIN HardFault_IRQn 0 */

  /* USER CODE END HardFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_HardFault_IRQn 0 */
    /* USER CODE END W1_HardFault_IRQn 0 */
  }
}

/**
  * @brief This function handles Memory management fault.
  */
void MemManage_Handler(void)
{
  /* USER CODE BEGIN MemoryManagement_IRQn 0 */

  /* USER CODE END MemoryManagement_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_MemoryManagement_IRQn 0 */
    /* USER CODE END W1_MemoryManagement_IRQn 0 */
  }
}

/**
  * @brief This function handles Prefetch fault, memory access fault.
  */
void BusFault_Handler(void)
{
  /* USER CODE BEGIN BusFault_IRQn 0 */

  /* USER CODE END BusFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_BusFault_IRQn 0 */
    /* USER CODE END W1_BusFault_IRQn 0 */
  }
}

/**
  * @brief This function handles Undefined instruction or illegal state.
  */
void UsageFault_Handler(void)
{
  /* USER CODE BEGIN UsageFault_IRQn 0 */

  /* USER CODE END UsageFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_UsageFault_IRQn 0 */
    /* USER CODE END W1_UsageFault_IRQn 0 */
  }
}

/**
  * @brief This function handles System service call via SWI instruction.
  */
void SVC_Handler(void)
{
  /* USER CODE BEGIN SVCall_IRQn 0 */

  /* USER CODE END SVCall_IRQn 0 */
  /* USER CODE BEGIN SVCall_IRQn 1 */

  /* USER CODE END SVCall_IRQn 1 */
}

/**
  * @brief This function handles Debug monitor.
  */
void DebugMon_Handler(void)
{
  /* USER CODE BEGIN DebugMonitor_IRQn 0 */

  /* USER CODE END DebugMonitor_IRQn 0 */
  /* USER CODE BEGIN DebugMonitor_IRQn 1 */

  /* USER CODE END DebugMonitor_IRQn 1 */
}

/**
  * @brief This function handles Pendable request for system service.
  */
void PendSV_Handler(void)
{
  /* USER CODE BEGIN PendSV_IRQn 0 */

  /* USER CODE END PendSV_IRQn 0 */
  /* USER CODE BEGIN PendSV_IRQn 1 */

  /* USER CODE END PendSV_IRQn 1 */
}

/**
  * @brief This function handles System tick timer.
  */
void SysTick_Handler(void)
{
  /* USER CODE BEGIN SysTick_IRQn 0 */

  /* USER CODE END SysTick_IRQn 0 */
  HAL_IncTick();
  /* USER CODE BEGIN SysTick_IRQn 1 */

  /* USER CODE END SysTick_IRQn 1 */
}

/******************************************************************************/
/* STM32F1xx Peripheral Interrupt Handlers                                    */
/* Add here the Interrupt Handlers for the used peripherals.                  */
/* For the available peripheral interrupt handler names,                      */
/* please refer to the startup file (startup_stm32f1xx.s).                    */
/******************************************************************************/

/**
  * @brief This function handles DMA1 channel5 global interrupt.
  */
void DMA1_Channel5_IRQHandler(void)
{
  /* USER CODE BEGIN DMA1_Channel5_IRQn 0 */

  /* USER CODE END DMA1_Channel5_IRQn 0 */
  HAL_DMA_IRQHandler(&hdma_usart1_rx);
  /* USER CODE BEGIN DMA1_Channel5_IRQn 1 */

  /* USER CODE END DMA1_Channel5_IRQn 1 */
}

/**
  * @brief This function handles USART1 global interrupt.
  */
void USART1_IRQHandler(void)
{
  /* USER CODE BEGIN USART1_IRQn 0 */
  if(__HAL_UART_GET_FLAG(&huart1, UART_FLAG_IDLE) != RESET)    
  {
		DutyCycleData temp;
		HAL_UART_DMAStop(&huart1); // 停止DMA接收
    uint16_t esp_cnt = 256 - __HAL_DMA_GET_COUNTER(&hdma_usart1_rx); // 计算接收到的数据长度
		char *	ptr=strstr((char *)buffer, "max_pwm-");
		if (ptr) {
			My_Max_PWM = atof(ptr + strlen("max_pwm-"));
		}
		ptr=strstr((char *)buffer, "min_pwm-");
		if (ptr) {
			My_Min_PWM = atof(ptr + strlen("min_pwm-"));
		}
		temp.initialized_flag=INITIALIZED_FLAG;
		temp.max_duty_cycle=My_Max_PWM;
		temp.min_duty_cycle=My_Min_PWM;
		Flash_WriteDutyCycleData(&temp);
		memset(buffer, 0, 1024); // 清空接收缓冲区
    HAL_UART_Receive_DMA(&huart3, buffer, 1024); // 重新启动DMA接收
		__HAL_UART_CLEAR_IDLEFLAG(&huart3); // 清除空闲中断标志位
	}
  /* USER CODE END USART1_IRQn 0 */
  //HAL_UART_IRQHandler(&huart1);
  /* USER CODE BEGIN USART1_IRQn 1 */

  /* USER CODE END USART1_IRQn 1 */
}

/**
  * @brief This function handles USART3 global interrupt.
  */
uint8_t ReceiveData = 0;

void USART3_IRQHandler(void)
{
  /* USER CODE BEGIN USART3_IRQn 0 */
  
    // LIN断开帧中断
    if (__HAL_UART_GET_FLAG(&huart3, UART_FLAG_LBD)) {  // 检查 LIN 断路检测中断标志位 (LBD)
        LIN_RingBUF_ClearRxMsg(&LIN_RxDataBuff);  // 清空当前缓冲区
        
        __HAL_UART_CLEAR_FLAG(&huart3, UART_FLAG_LBD);  // 清除 LIN 断路检测中断标志位 (LBD)
				__HAL_UART_CLEAR_FLAG(&huart3, UART_FLAG_RXNE);  // 清除 LIN 断路检测中断标志位 (LBD)
        LIN_RxStateGet = SYNCH_GET;  // 设置接收状态为同步获取
        return;
    }
    
    // LIN接收中断
    if (__HAL_UART_GET_FLAG(&huart3, UART_FLAG_RXNE)) {  // 检查接收缓冲区非空标志位 (RXNE)
            // 等待接收缓冲区非空标志位 (RXNE) 被置位
			while(!(USART3->SR & USART_SR_RXNE))
				;
			ReceiveData=(uint8_t)(USART3->DR & 0xFF);
        
        if (!__HAL_UART_GET_FLAG(&huart3, UART_FLAG_FE)) {  // 检查帧错误标志位 (FE) 是否被设置
            if ((ReceiveData == 0x55) && (LIN_RxStateGet == BREAK_GET)) {  // 处理无同步间隔信号的LIN数据
                LIN_RingBUF_ClearRxMsg(&LIN_RxDataBuff);  // 清空当前缓冲区
                LIN_RxStateGet = SYNCH_GET;  // 设置接收状态为同步获取
                return;
            }
            LIN_MasterRxMsg(ReceiveData);  // 消息处理
        }
			__HAL_UART_CLEAR_FLAG(&huart3, UART_FLAG_RXNE);  // 清除 LIN 断路检测中断标志位 (LBD)
    }
  /* USER CODE END USART3_IRQn 0 */
  //HAL_UART_IRQHandler(&huart3);
  /* USER CODE BEGIN USART3_IRQn 1 */

  /* USER CODE END USART3_IRQn 1 */
}


/* USER CODE BEGIN 1 */

/* USER CODE END 1 */
