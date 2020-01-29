/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "spi.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "w25qxx.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

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
uint8_t RxCounter=0,RxBuffer1[50]={0},RxYemp1=0,F_Usart=0;
float get;
char getchar; 
uint32_t getu32;
int getint;
const uint8_t TEXT_Buffer[]={"Handsome Chen\r\n"};
uint8_t TEXT_Buffer1[2]={12,13};
#define SIZE sizeof(TEXT_Buffer)
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
	uint32_t FLASH_SIZE; 
	uint8_t datatemp[25];
	//float test=3.545454;
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
  MX_USART1_UART_Init();
  MX_SPI1_Init();
  /* USER CODE BEGIN 2 */
	W25QXX_Init();				    //W25QXX初始化
	FLASH_SIZE=16*1024*1024;	//FLASH 大小为16M字节
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
		while(W25QXX_ReadID()!=W25Q128)								//检测不到W25Q128
			{
				HAL_UART_Transmit(&huart1, "Read Data Failt/r/n", 19, 50);
				HAL_Delay(30);
			}			
//		W25QXX_AllData_Initial();
//		W25QXX_Put_u8Data(12);
//		W25QXX_Put_u8Data(123);
//		W25QXX_Put_u16Data(1234);
//		W25QXX_Put_u16Data(12345);
//		W25QXX_Put_u32Data(123456);
//		W25QXX_Put_u32Data(1234567);
//		W25QXX_Put_intData(12345678);
//		W25QXX_Put_intData(12345678);
//		W25QXX_Put_floatData(1.234567);
//		W25QXX_Put_floatData(12.34567);
//		W25QXX_Put_doubleData(12.3456);
//		W25QXX_Put_doubleData(123.456);
//		W25QXX_Put_charData(110);
//		W25QXX_Put_charData(110);
		W25QXX_Data_Fixchar(2,0);
		getchar = W25QXX_Data_charRead(2);
		while(1)
			{
				HAL_UART_Transmit(&huart1, TEXT_Buffer1, 4, 50);
				HAL_Delay(900);
			}
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
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 168;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB busses clocks 
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
// GPIO中断处理函数
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
	{
		if(GPIO_Pin==GPIO_PIN_2) // 2号中断线 按键共地检测下降沿
			{
				HAL_Delay(5);
				if(!HAL_GPIO_ReadPin(GPIOE, GPIO_PIN_2))
				HAL_GPIO_WritePin(GPIOF, GPIO_PIN_9, GPIO_PIN_RESET);
			}
		if(GPIO_Pin==GPIO_PIN_3) // 3号中断线 按键共地检测下降沿
			{
				HAL_Delay(5);
				if(!HAL_GPIO_ReadPin(GPIOE, GPIO_PIN_3))
				HAL_GPIO_WritePin(GPIOF, GPIO_PIN_9, GPIO_PIN_SET);
			}
//		if(GPIO_Pin==GPIO_PIN_4) // 4号中断线 按键共地检测下降沿
//			{
//				HAL_Delay(10);
//				if(!HAL_GPIO_ReadPin(GPIOE, GPIO_PIN_4))
//					{
//						HAL_GPIO_TogglePin(GPIOF, GPIO_PIN_9);
//						while(!HAL_GPIO_ReadPin(GPIOE, GPIO_PIN_4));
//					}
//				}
			// 按键中断设置函数
	}
// 串口中断
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
	{
		if(huart->Instance == USART1)
			{
				HAL_UART_Transmit(&huart1, RxBuffer1, 4, 50);
				HAL_UART_Receive_IT(&huart1,RxBuffer1,4);
				HAL_GPIO_TogglePin(GPIOF, GPIO_PIN_9);
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
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
