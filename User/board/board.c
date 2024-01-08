/*
 * board.c
 *
 *  Created on: Dec 26, 2023
 *      Author: vypa0
 */

#include "board.h"
#include "stm32f1xx_hal.h"

UART_hw rs485_com;


void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_ADC1_Init(void);

 void uart_rs485_hw_init(void){
	rs485_com.uart_module.Instance = RS485_PORT_COM;
	rs485_com.uart_module.Init.BaudRate = UART_BAUDRATE_SLAVE;
	rs485_com.uart_module.Init.WordLength = UART_WORDLENGTH_8B;
	rs485_com.uart_module.Init.StopBits = UART_STOPBITS_1;
	rs485_com.uart_module.Init.Parity = UART_PARITY_NONE;
	rs485_com.uart_module.Init.Mode = UART_MODE_TX_RX;
	rs485_com.uart_module.Init.HwFlowCtl = UART_HWCONTROL_NONE;
	rs485_com.uart_module.Init.OverSampling = UART_OVERSAMPLING_16;
	if (HAL_UART_Init(&rs485_com.uart_module) != HAL_OK)
	{
		Error_Handler();
	}
	HAL_UART_Receive_IT(&rs485_com.uart_module, (uint8_t*)&rs485_com.rx_data, 1);

}

 void HAL_UART_MspInit(UART_HandleTypeDef* uartHandle){

 	GPIO_InitTypeDef GPIO_InitStruct = {0};

 	if(uartHandle->Instance == RS485_PORT_COM)
 	{
 		/* USART1 clock enable */
 		__HAL_RCC_USART1_CLK_ENABLE();
 		__HAL_RCC_GPIOA_CLK_ENABLE();

 		/**USART1 GPIO Configuration
 		PA9     ------> USART1_TX
 		PA10     ------> USART1_RX
 		*/
 		GPIO_InitStruct.Pin = GPIO_PIN_9;
 		GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
 		GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
 		HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

 		GPIO_InitStruct.Pin = GPIO_PIN_10;
 		GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
 		GPIO_InitStruct.Pull = GPIO_NOPULL;
 		HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

 		/* USART1 interrupt Init */
 		HAL_NVIC_SetPriority(USART1_IRQn, 1, 0);
 		HAL_NVIC_EnableIRQ(USART1_IRQn);
 	}

 }

void board_init(void){
	  HAL_Init();
	  SystemClock_Config();
	  MX_GPIO_Init();
	  MX_DMA_Init();
	  MX_ADC1_Init();
	  uart_rs485_hw_init();
	  HAL_ADC_Start_DMA(&hadc1, (uint32_t*)sensor_value, 3);

}


void FloatToStringX_XXX(double* value, uint8_t* buffer){
	buffer[1] = '.' ;
	uint32_t number = (*value) * 1000.001;
	buffer[4] = (char)(number % 10 + 48);
	number = number / 10 ;
	buffer[3] = (char)(number % 10 + 48);
	number = number / 10 ;
	buffer[2] = (char)(number % 10 + 48);
	number = number / 10 ;
	buffer[0] = (char)(number % 10 + 48);
	return;
}


void FloatToStringXX_XX(double* value, uint8_t* buffer){
	buffer[2] = '.' ;
	uint32_t number = (*value) * 100.001;
	buffer[4] = (char)(number % 10 + 48);
	number = number / 10 ;
	buffer[3] = (char)(number % 10 + 48);
	number = number / 10 ;
	buffer[1] = (char)(number % 10 + 48);
	number = number / 10 ;
	buffer[0] = (char)(number % 10 + 48);
	return;

}


void FloatToStringXXX_X(double* value, uint8_t* buffer){
	buffer[3] = '.' ;
	uint32_t number = (*value) * 10.001;
	buffer[4] = (char)(number % 10 + 48);
	number = number / 10 ;
	buffer[2] = (char)(number % 10 + 48);
	number = number / 10 ;
	buffer[1] = (char)(number % 10 + 48);
	number = number / 10 ;
	buffer[0] = (char)(number % 10 + 48);
	return;
}

void FloatToStringXXXX(double* value, uint8_t* buffer){
	buffer[4] = ' ';
	uint32_t number = (*value) * 1.0001;
	buffer[3] = (char)(number % 10 + 48);
	number = number / 10 ;
	buffer[2] = (char)(number % 10 + 48);
	number = number / 10 ;
	buffer[1] = (char)(number % 10 + 48);
	number = number / 10 ;
	buffer[0] = (char)(number % 10 + 48);
	return;

}

void FloatToStringXXXXX(double* value, uint8_t* buffer){
	uint32_t number = (*value) * 1.000001;
	buffer[4] = (char)(number % 10 + 48);
	number = number / 10 ;
	buffer[3] = (char)(number % 10 + 48);
	number = number / 10 ;
	buffer[2] = (char)(number % 10 + 48);
	number = number / 10 ;
	buffer[1] = (char)(number % 10 + 48);
	number = number / 10 ;
	buffer[0] = (char)(number % 10 + 48);
	return;

}



void lkv_lcd_float_to_string(double* value, uint8_t* buffer){

	if(*value < 10){

		return FloatToStringX_XXX(value, buffer);

	}else if(*value < 100){

		return FloatToStringXX_XX(value, buffer);

	}else if(*value < 1000){

		return FloatToStringXXX_X(value, buffer);

	}else if(*value < 10000){

		return FloatToStringXXXX(value, buffer);

	}else {

		return FloatToStringXXXXX(value, buffer);
	}
}


void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

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
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC;
  PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV6;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Common config
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ScanConvMode = ADC_SCAN_ENABLE;
  hadc1.Init.ContinuousConvMode = ENABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 3;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_0;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_239CYCLES_5;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_1;
  sConfig.Rank = ADC_REGULAR_RANK_2;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_2;
  sConfig.Rank = ADC_REGULAR_RANK_3;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel1_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

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
