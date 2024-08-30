/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
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
#include "dma.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "lin_driver.h"
#include "linbuffer.h"
#include "duty_cycle_storage.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
LIN_MSG*  pWiperMsg;    //LIN数据 
extern    LIN_BUFFER  LIN_RxDataBuff;   //缓冲区
extern    uint8_t           data[8];
float My_Max_PWM,My_Min_PWM;
uint16_t PWM_LSB,PWM_BASE,my_duty;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
void LIN_FAN_Drive(LIN_MSG* pLINMsg);
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_TIM1_Init();
  MX_USART1_UART_Init();
  MX_USART3_UART_Init();
  /* USER CODE BEGIN 2 */
	InitDutyCycleData();
	DutyCycleData temp;
	Flash_ReadDutyCycleData(&temp);
	My_Max_PWM=temp.max_duty_cycle;
	My_Min_PWM=temp.min_duty_cycle;
	PWM_LSB=(My_Max_PWM-My_Min_PWM)*20;
	PWM_BASE=My_Min_PWM*20;
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
		LIN_RingBUF_Read(&LIN_RxDataBuff,&pWiperMsg);  //不断取出缓冲区数据
		LIN_FAN_Drive(pWiperMsg);
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
//从机接收驱动风扇
void LIN_FAN_Drive(LIN_MSG* pLINMsg)
{	
	if (pLINMsg->FrameID == 0x2D){
		switch (pLINMsg->Data[0] & 0x01){		
			case 0x01: 
			{
				HAL_GPIO_WritePin(CTL_EN_GPIO_Port,CTL_EN_Pin,GPIO_PIN_RESET);
				HAL_GPIO_WritePin(LED_GPIO_Port,LED_Pin,GPIO_PIN_SET);
				__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_1,(PWM_BASE+pLINMsg->Data[1]*PWM_LSB));
				HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_1);
				if((PWM_BASE+pLINMsg->Data[1]*PWM_LSB)>1200)
				{
					data[0]=(0x01 | 0x00 | 0x08) & 0x0f;
					data[1]=pLINMsg->Data[1];
					data[2]=135;
					data[3]=(pLINMsg->Data[1]*5) & 0x03;
					data[4]=(pLINMsg->Data[1]*5)>>2;
					data[5]=pLINMsg->Data[1]*5;
					data[6]=pLINMsg->Data[1]*5>>8;
					data[7]=0x00;
				}
				else
				{
					if(pLINMsg->Data[1]==0)
						data[0]=(0x00 | 0x01 | 0x00) & 0x0f;
					else
						data[0]=(0x00 | 0x01 | 0x08) & 0x0f;
					data[1]=pLINMsg->Data[1];
					data[2]=135;
					data[3]=(pLINMsg->Data[1]*5) & 0x03;
					data[4]=(pLINMsg->Data[1]*5)>>2;
					data[5]=pLINMsg->Data[1]*5;
					data[6]=pLINMsg->Data[1]*5>>8;
					data[7]=0x00;
				}
			}				
			break;
			case 0x00: 
			{
				HAL_GPIO_WritePin(CTL_EN_GPIO_Port,CTL_EN_Pin,GPIO_PIN_SET);
				HAL_GPIO_WritePin(LED_GPIO_Port,LED_Pin,GPIO_PIN_RESET);
				HAL_TIM_PWM_Stop(&htim1,TIM_CHANNEL_1);
				data[0]=(0x00 | 0x01 | 0x00) & 0x0f;
				data[1]=0x00;
				data[2]=135;
				data[3]=0x00;
			  data[4]=0x00;
				data[5]=0x00;
				data[6]=0x00;
				data[7]=0x00;
			}				
			break;				
		}		
	}
}
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
