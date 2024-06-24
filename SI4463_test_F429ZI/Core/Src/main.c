/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include "si4463_huang.h"
#include "ax25_huang.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#ifdef __GNUC__
  #define PUTCHAR_PROTOTYPE int __io_putchar(int ch)
#else
  #define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)
#endif /* __GNUC__ */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
SPI_HandleTypeDef hspi1;

TIM_HandleTypeDef htim1;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
PUTCHAR_PROTOTYPE
{
	HAL_UART_Transmit(&huart2, (uint8_t *)&ch, 1, 0xFFFF);
	return ch;
}

si4463_t si4463;
bool irqFlag = false;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_SPI1_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_TIM1_Init(void);
/* USER CODE BEGIN PFP */
uint8_t si4463_SPI_Write(uint8_t* pTxData, uint8_t dataLen, uint32_t timeout);
uint8_t si4463_SPI_Read(uint8_t* pRxData, uint8_t dataLen, uint32_t timeout);
uint8_t si4463_SPI_WriteRead(uint8_t* pTxData, uint8_t* pRxData, uint8_t dataLen, uint32_t timeout);
uint8_t si4463_SPI_CheckState(void);
void si4463_DelayUs(uint32_t delay);
void si4463_setNSEL(bool val);
void si4463_setSDN(bool val);
bool si4463_getIRQ(void);
bool si4463_getGPIO1(void);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	if(GPIO_Pin == B1_Pin)
	{
		printf("Button interrupt !\r\n");
		si4463_getInterrupts(&si4463);
		printf("Current RSSI: %d\r\n", si4463_getCurrentRSSI(&si4463));
		irqFlag = true;
	}
	if(GPIO_Pin == IRQ_Pin)
	{
		irqFlag = true;
	}
}
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
  MX_SPI1_Init();
  MX_USART2_UART_Init();
  MX_TIM1_Init();
  /* USER CODE BEGIN 2 */

  HAL_TIM_Base_Start(&htim1); // us delay timer

	si4463.SPI_Write = si4463_SPI_Write;
	si4463.SPI_Read = si4463_SPI_Read;
	si4463.SPI_WriteRead = si4463_SPI_WriteRead;
	si4463.SPI_CheckState = si4463_SPI_CheckState;
	si4463.spi_state_ready = HAL_SPI_STATE_READY;
	si4463.DelayUs = si4463_DelayUs;
	si4463.NSEL = si4463_setNSEL;
	si4463.SDN = si4463_setSDN;
	si4463.gpios.IRQ = si4463_getIRQ;
	si4463.gpios.GPIO1 = si4463_getGPIO1;
	si4463.gpios.gpio_low = GPIO_PIN_RESET;
	si4463.gpios.gpio_high = GPIO_PIN_SET;

	//HAL_NVIC_DisableIRQ(EXTI4_IRQn);
	/* Initialize si4463 */
	int res = si4463_init(&si4463);
	if(res == SI4463_OK)
		printf("Si4463 init .. ok !\r\n");
	else
		printf("Si4463 init .. fail ! error code: %d\r\n", res);

	printf("Get Tx power %d \r\n", si4463_getTxPower(&si4463));

	res = si4463_setTxPower(&si4463, 10);
	if(res == SI4463_OK)
	{
		printf("Set Tx power .. ok !\r\n");
		printf("Get Tx power %d \r\n", si4463_getTxPower(&si4463));
	}
	else
		printf("Set Tx power .. fail ! error code: %d\r\n", res);


	res = si4463_setFrequency(&si4463, 434500000);
	if(res == SI4463_OK)
		printf("Set frequency .. ok !\r\n");
	else
		printf("Set frequency .. fail ! error code: %d\r\n", res);

	res = si4463_setTxModulation(&si4463, MOD_2FSK);

	res = si4463_setTxDataRate(&si4463, DR_1200);

	uint8_t control = RADIOLIB_AX25_CONTROL_U_UNNUMBERED_INFORMATION | RADIOLIB_AX25_CONTROL_POLL_FINAL_DISABLED | RADIOLIB_AX25_CONTROL_UNNUMBERED_FRAME;
	ax25frame_t* ax25frame = createAX25Frame("STARL", 0, "NCKU", 0, control, RADIOLIB_AX25_PID_NO_LAYER_3, (uint8_t*)"What is 'X' SATORO ?", strlen("What is 'X' SATORO ?"), 8);
	uint16_t txLen = 0;
	uint8_t* txData = AX25Frame_HDLC_Generator(ax25frame, &txLen);

//	ax25_g3ruh_scrambler_init(0x21000UL);
//	ax25_g3ruh_scrambler(txData, txData, txLen);
//	res = si4463_initRx(&si4463, 0, STATE_RX, STATE_RX, STATE_RX);
//	if(res == SI4463_OK)
//		printf("Start Rx .. ok !\r\n");
//	else
//		printf("Start Rx .. fail ! error code: %d\r\n", res);

	//HAL_NVIC_EnableIRQ(EXTI4_IRQn);
	uint8_t rxDataLen = 0;
	uint8_t rxData[100] = {0};
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {


	  res = si4463_transmit(&si4463, txData, txLen, STATE_NO_CHANGE);
	  if(res == SI4463_OK)
	  {
		  printf("Si4463 Transmit .. ok !\r\n");
		  printf("packet: ");
		  for(int i = 0; i < txLen; i++)
			  printf("0x%02x ", txData[i]);
		  printf("\r\n");
	  }

	  else
		  printf("Si4463 Transmit .. fail ! error code: %d\r\n", res);

	  HAL_Delay(3000);
//	if(si4463.gpios.IRQ() == si4463.gpios.gpio_low)
//	{
//		printf("RSSI: %d\r\n", si4463_getLatchRSSI(&si4463));
//		si4463_getInterrupts(&si4463);
//		res = si4463_getRxFifoInfo(&si4463);
//		if(res > 0)
//		{
//			rxDataLen = res;
//			res = si4463_receive(&si4463, rxData, rxDataLen);
//			if(res == SI4463_OK)
//			{
//				printf("Receive .. ok !\r\n");
//				printf("Message: ");
//				for(int i = 0;i < rxDataLen; i++)
//				{
//					printf("0x%02x ", rxData[i]);
//				}
//				printf("\r\n");
//				memset(rxData, '\0', rxDataLen);
//
//			}
//			else
//				printf("Receive .. fail ! error code: %d\r\n", res);
//		}
//		else
//		{
//			printf("Get Rx FiFO info .. fail ! error code: %d\r\n", res);
//		}
//	}
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
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 180;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 7;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Activate the Over-Drive mode
  */
  if (HAL_PWREx_EnableOverDrive() != HAL_OK)
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

/**
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_64;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

}

/**
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 180-1;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 65535;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOG_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(NSEL_GPIO_Port, NSEL_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(SDN_GPIO_Port, SDN_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOG, LD3_Pin|LD4_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : IRQ_Pin */
  GPIO_InitStruct.Pin = IRQ_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(IRQ_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : NSEL_Pin */
  GPIO_InitStruct.Pin = NSEL_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(NSEL_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : SDN_Pin */
  GPIO_InitStruct.Pin = SDN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(SDN_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : GPIO1_Pin */
  GPIO_InitStruct.Pin = GPIO1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIO1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : LD3_Pin LD4_Pin */
  GPIO_InitStruct.Pin = LD3_Pin|LD4_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOG, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI0_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(EXTI0_IRQn);

  HAL_NVIC_SetPriority(EXTI4_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(EXTI4_IRQn);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
uint8_t si4463_SPI_Write(uint8_t* pTxData, uint8_t dataLen, uint32_t timeout)
{
	return HAL_SPI_Transmit(&hspi1, pTxData, dataLen, timeout);
}

uint8_t si4463_SPI_Read(uint8_t* pRxData, uint8_t dataLen, uint32_t timeout)
{
	return HAL_SPI_Receive(&hspi1, pRxData, dataLen, timeout);
}

uint8_t si4463_SPI_WriteRead(uint8_t* pTxData, uint8_t* pRxData, uint8_t dataLen, uint32_t timeout)
{
	return HAL_SPI_TransmitReceive(&hspi1, pTxData, pRxData, dataLen, timeout);
}

uint8_t si4463_SPI_CheckState(void)
{
	return HAL_SPI_GetState(&hspi1);
}

void si4463_DelayUs(uint32_t delay)
{
	__HAL_TIM_SET_COUNTER(&htim1, 0);
	while((__HAL_TIM_GET_COUNTER(&htim1)) < delay);
}

void si4463_setNSEL(bool val)
{
	if(val == true)
		HAL_GPIO_WritePin(NSEL_GPIO_Port, NSEL_Pin, GPIO_PIN_SET);
	else
		HAL_GPIO_WritePin(NSEL_GPIO_Port, NSEL_Pin, GPIO_PIN_RESET);
}

void si4463_setSDN(bool val)
{
	if(val == true)
		HAL_GPIO_WritePin(SDN_GPIO_Port, SDN_Pin, GPIO_PIN_SET);
	else
		HAL_GPIO_WritePin(SDN_GPIO_Port, SDN_Pin, GPIO_PIN_RESET);
}

bool si4463_getIRQ(void)
{
	return HAL_GPIO_ReadPin(IRQ_GPIO_Port, IRQ_Pin);
}

bool si4463_getGPIO1(void)
{
	return HAL_GPIO_ReadPin(GPIO1_GPIO_Port, GPIO1_Pin);
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
