/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
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
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <string.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
#define APP_ADDRESS 0x08010000
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

typedef void (*pFunction)(void);
void JumpToApplication(void);
void SendACK(void);
void Flash_Erase_App_Sectors(void);
void Flash_Write_Data(uint32_t address , uint8_t* data , uint16_t length);
void CustomFotaLoop(void);


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
  MX_USART6_UART_Init();
  /* USER CODE BEGIN 2 */
  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12, GPIO_PIN_SET);
  uint8_t syncByte = 0;
  HAL_StatusTypeDef res = HAL_UART_Receive(&huart6, &syncByte, 1, 2000);

  if(res == HAL_OK &&  syncByte == 0x7F)
  {
	  SendACK();
	  CustomFotaLoop();
  }
  else
  {
	  JumpToApplication();
	  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12, GPIO_PIN_RESET);
  }
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {

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

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 7;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */


void SendACK(void)
{
	uint8_t ack = 0x79;
	HAL_UART_Transmit(&huart6 , &ack , 1 , 100);
}


void JumpToApplication(void)
{
	uint32_t mspAddress = *(__IO uint32_t*)APP_ADDRESS;
	uint32_t jumpAddress = *(__IO uint32_t*) (APP_ADDRESS + 4);
	pFunction JumpToApp  = (pFunction) jumpAddress;


		HAL_GPIO_WritePin(GPIOD, GPIO_PIN_13, GPIO_PIN_RESET);
		HAL_Delay(100);
		HAL_RCC_DeInit();
		HAL_DeInit();
		SysTick->CTRL = 0;
		SysTick->LOAD = 0;
		SysTick->VAL = 0;


		for (int i = 0; i < 8; i++) {
		        NVIC->ICER[i] = 0xFFFFFFFF;
		        NVIC->ICPR[i] = 0xFFFFFFFF;
		}

		SCB->VTOR = APP_ADDRESS;
		__set_MSP(*(__IO uint32_t*) APP_ADDRESS);
		JumpToApp();

}


void Flash_Erase_App_Sectors(void)
{
	HAL_FLASH_Unlock();


	FLASH_EraseInitTypeDef EraseInitStruct;
	uint32_t SectorError = 0;


	EraseInitStruct.TypeErase 		= FLASH_TYPEERASE_SECTORS;
	EraseInitStruct.VoltageRange	= FLASH_VOLTAGE_RANGE_3;
	EraseInitStruct.Sector			= FLASH_SECTOR_4;
	EraseInitStruct.NbSectors		= 8;

	HAL_FLASHEx_Erase(&EraseInitStruct, &SectorError);

	HAL_FLASH_Lock();
}



void Flash_Write_Data(uint32_t address , uint8_t* data , uint16_t length)
{
	HAL_FLASH_Unlock();


	for(uint16_t i = 0 ; i < length ; i++)
	{
		HAL_FLASH_Program(FLASH_TYPEPROGRAM_BYTE, address + i , data[i]);

	}

	HAL_FLASH_Lock();
}


static uint8_t dataBuf[256];


void CustomFotaLoop(void)
{
	uint8_t cmd;
	uint32_t currentWriteAdress = APP_ADDRESS;


	while(1)
	{
		if(HAL_UART_Receive(&huart6, &cmd , 1 , HAL_MAX_DELAY) == HAL_OK)
		{
			if(cmd == 0xA1) //Delete
			{
				SendACK();
				Flash_Erase_App_Sectors();
				currentWriteAdress = APP_ADDRESS;
				SendACK();
			}




			else if (cmd == 0xA2)
			{
			    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_13, GPIO_PIN_SET);
			    SendACK();

			    uint8_t lenMinusOne;
			    if (HAL_UART_Receive(&huart6, &lenMinusOne, 1, 1000) == HAL_OK)
			    {
			        uint16_t length = (uint16_t)lenMinusOne + 1;

			        __HAL_UART_CLEAR_OREFLAG(&huart6);
			        huart6.ErrorCode = HAL_UART_ERROR_NONE;

			        SendACK();

			        HAL_GPIO_WritePin(GPIOC, GPIO_PIN_10, GPIO_PIN_SET);

			        if (HAL_UART_Receive_DMA(&huart6, dataBuf, length) == HAL_OK)
			        {
			            uint32_t startTime = HAL_GetTick();
			            while (huart6.RxState != HAL_UART_STATE_READY)
			            {
			                if (HAL_GetTick() - startTime > 2000) break;
			            }

			            if (huart6.RxState == HAL_UART_STATE_READY)
			            {
			                HAL_GPIO_WritePin(GPIOC, GPIO_PIN_10, GPIO_PIN_RESET);

			                Flash_Write_Data(currentWriteAdress, dataBuf, length);
			                currentWriteAdress += length;

			                HAL_Delay(10);
			                SendACK();
			                HAL_GPIO_WritePin(GPIOD, GPIO_PIN_13, GPIO_PIN_RESET);
			            }
			            else
			            {
			                HAL_UART_DMAStop(&huart6);
			                HAL_GPIO_WritePin(GPIOD, GPIO_PIN_13, GPIO_PIN_SET);
			                HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12 | GPIO_PIN_15, GPIO_PIN_RESET);
			            }
			        }
			        else
			        {
			            huart6.RxState = HAL_UART_STATE_READY;
			        }
			    }
			}

			else if (cmd == 0xA3)
			{
				SendACK(); //Exit
				JumpToApplication();
			}

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
#ifdef USE_FULL_ASSERT
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
