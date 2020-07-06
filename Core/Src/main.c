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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

#include "TComDef.h"

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
ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;

DAC_HandleTypeDef hdac;
DMA_HandleTypeDef hdma_dac_ch1;

TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim6;

UART_HandleTypeDef huart3;
DMA_HandleTypeDef hdma_usart3_rx;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM6_Init(void);
static void MX_ADC1_Init(void);
static void MX_DAC_Init(void);
static void MX_USART3_UART_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
/*!
 *	Обработчик прерываний по таймеру
 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	if (htim -> Instance == TIM2) managerState () ;			// С частотой 8 кГц обрабатываем состояния контроллера
	if (htim -> Instance == TIM3) managerTransfer () ;		// По срабатыванию этого таймера выполняется сжатие и передача буфера
	if (htim -> Instance == TIM6) {							// Если сработал этот таймер, то значит передача звука со второго контроллера прекращена
		setState (stateWait) ;
		stVoiceDecodeBufPos = 0 ;
		stVoiceDecodeBufNum = 0 ;
	}
}
//-------------------------------------------------------------------------------------------
/*!
 *	Обработка прерываний по нажатию кнопок
 */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	EXTI -> PR = EXTI_PR_PR0 ;		// Сбрасываем прерывание. Нужно что бы при нажатии на кнопку прерывание не вызывалось два раза.

	switch (stState) {
	  case stateWait :				// Реакция на кнопки возможна только в режиме ожидания и передачи голоса
	  case stateReady :
	  case stateADC :
	  case stateFirstTim3 :
	  case stateSpeexCompress :
		if (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_0) == GPIO_PIN_RESET) {
			setState (stateFirstTim3) ;						// Переводим в режим оцифровки с микрофона, пропуская первое срабатывание таймера TIM3
			stVoiceEncodeBufPos = 0 ;
			stVoiceEncodeBufNum = 0 ;
			HAL_TIM_Base_Start_IT(&htim3);					// запуск таймера контроля заполнения буфера
		}
		  else {
			setState (stateWait) ;
			HAL_TIM_Base_Stop_IT(&htim3) ;
		  }
	  break ;

	  default :
	  break ;
	}
}
//-------------------------------------------------------------------------------------------
/*!
 *	Обработка прерываний по DMA от аналогового микрофона
 *		!!! При недостаточной производительности контроллера, увеличивается кол-во ошибок. Зачем это нужно я не знаю.
 */
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc)
{
	switch (stState) {				// Если сработало прерывание, то значит нужно сменить позицию в буфере
	  case stateReady :
		if (++stVoiceEncodeBufPos == defVoiceInputBufSize) {
		    stVoiceEncodeBufPos = 0 ;
		    stVoiceEncodeBufTransfer = stVoiceEncodeBufNum ;
		    if (++stVoiceEncodeBufNum == defNumBuf) stVoiceEncodeBufNum = 0 ;
	    }
	  break ;

	  case stateSpeexCompress :	// Если сжатие кодеком еще не закончилось а буфер уже заполнен, то просто теряем оцифровку звука пока оно не закончится
		if (++stVoiceEncodeBufPos == defVoiceInputBufSize) {
			stVoiceEncodeBufPos = defVoiceInputBufSize - 1 ;
			stVoiceEncodeErr++ ;
		}
	  break ;

	  default :
	  break ;
	}
}
//-------------------------------------------------------------------------------------------
/*!
 *	Обработка прерываний по DMA от воспроизведения звука
 */
void HAL_DAC_ConvCpltCallbackCh1(DAC_HandleTypeDef *hdac)
{
	if (++stVoiceDecodeBufPos == defVoiceInputBufSize) {
		stVoiceDecodeBufPos = 0 ;
		if (++stVoiceDecodeBufPlay == defNumBuf)stVoiceDecodeBufPlay = 0 ;
		if (stVoiceDecodeBufPlay == stVoiceDecodeBufNum) setState(stateVoiceWait) ; // Если новых данных не получено, то переходим в режим ожидания получения данных
	}
}
//-------------------------------------------------------------------------------------------
/*!
 * Обработка прерывания DMA после приёма данных
 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	switch (stState) {
	  case stateVoicePlay:
	  case stateVoiceWait :
		htim6.Instance -> CNT = 0 ;									// По новой начинаем отсчёт ожидания прекращения получения звука
		setState (stateVoiceReceive) ;
	  break;

	  case stateWait:
		HAL_TIM_Base_Start_IT (&htim6) ;							// Запускаем таймер для контроля перекращения получения звука.
		setState (stateVoiceReceive) ;
	  break;

	  default:
	  break;
	}

}
//-------------------------------------------------------------------------------------------
/*!
 * В зависимости от состояния управляем светодиодами
 */
void setState (defState inState)
{
	stState = inState ;
	switch (stState) {
	  case stateWait :
		HAL_GPIO_WritePin (GPIOA, defColorLight1, GPIO_PIN_RESET) ;	// Выключаем светодиоды
		HAL_GPIO_WritePin (GPIOA, defColorLight2, GPIO_PIN_RESET) ;
		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_12, GPIO_PIN_RESET) ;		// Включаем контроллер на приём
	  break ;

	  case stateReady :
	  case stateSpeexCompress :
		HAL_GPIO_WritePin (GPIOA, defColorLight1, GPIO_PIN_SET) ;
		HAL_GPIO_WritePin (GPIOA, defColorLight2, GPIO_PIN_RESET) ;
		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_12, GPIO_PIN_SET) ;		// Включаем контроллер на передачу
	  break ;

	  case stateVoiceReceive :
		HAL_GPIO_WritePin (GPIOA, defColorLight1, GPIO_PIN_RESET) ;
		HAL_GPIO_WritePin (GPIOA, defColorLight2, GPIO_PIN_SET) ;
	  break ;

	  default :
	  break ;
	}
}
//-------------------------------------------------------------------------------------------
/*!
 *	Менеджер обработки состояний
 */
void managerState  ()
{
stTemp++ ;

	switch (stState) {
	  case stateStart :
		if (stStartPeriod-- == 0) setState(stateWait) ;
							// Сюда я хотел присать какие-то действия при запуске.
	  break ;

	  case stateReady :
	  case stateSpeexCompress :
        HAL_ADC_Start_DMA (&hadc1, (uint32_t*) &stVoiceEncodeBuf [stVoiceEncodeBufNum][stVoiceEncodeBufPos], 1);
	  break ;

	  case stateVoiceReceive : {	// Получен очередной блок.
		speex_bits_read_from(&stSpeexDecodeStream, (char *) stSpeexDecodeBuf, defVoiceEncodeBufSize) ;
		speex_decode_int(stSpeexDecodeHandle, &stSpeexDecodeStream, (spx_int16_t *) stVoiceDecodeBuf [stVoiceDecodeBufNum]) ;
		if (++stVoiceDecodeBufNum == defNumBuf) stVoiceDecodeBufNum = 0 ;
		startUART_DMA ;
		setState (stateVoicePlay) ;
	  }
	  break ;

	  case stateVoicePlay :	{		// Воспроизводим через динамик декодированные данные
		  uint32_t voice = stVoiceDecodeBuf [stVoiceDecodeBufPlay][stVoiceDecodeBufPos] + 700 ;
		  HAL_DAC_Start_DMA(&hdac, DAC_CHANNEL_1, (uint32_t *) &voice, 1, DAC_ALIGN_12B_R) ;
	  }
	  break ;

	  default :
	  break ;
	}

}
//----------------------------------------------------------------------------------
/*!
 *	Менеджер сжатия кодеком и передачи данных
 */
void managerTransfer ()
{
	switch (stState) {
	  case stateFirstTim3 :		// Пропускаем первое срабатывание таймера TIM3
		setState (stateReady) ;
	  break ;

	  case stateReady :
		setState (stateSpeexCompress) ;
		speex_bits_reset(&stSpeexEncodeStream) ;	// Обрабатываем кодеком входной буфер
		speex_encode_int(stSpeexEncodeHandle, (spx_int16_t*)stVoiceEncodeBuf [stVoiceEncodeBufTransfer], &stSpeexEncodeStream);
		speex_bits_write(&stSpeexEncodeStream, (char *) stSpeexEncodeBuf, defVoiceEncodeBufSize);

		HAL_UART_Transmit(&huart3, (uint8_t *)stSpeexEncodeBuf, defVoiceEncodeBufSize, defTimeoutTransmit) ;	// Сжатые данные передаём на второй контроллер
		if (stState == stateSpeexCompress) setState (stateReady) ;	// Проверяем, что за время обработки буфера кодеком состояние не изменилось
	  break ;
	}
}
//----------------------------------------------------------------------------------
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
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_TIM6_Init();
  MX_ADC1_Init();
  MX_DAC_Init();
  MX_USART3_UART_Init();
  /* USER CODE BEGIN 2 */

  speex_bits_init(&stSpeexEncodeStream);
  speex_bits_init(&stSpeexDecodeStream);
  stSpeexDecodeHandle = speex_decoder_init(&speex_nb_mode);		// Инициализация для работы кодека speex на передачу
  stSpeexEncodeHandle = speex_encoder_init(&speex_nb_mode);
  //	speex_encoder_ctl(stSpeexEncodeHandle, SPEEX_SET_VBR, &stSpeexVBR);	// По большому счёту эти настройки не нужны, т.к. всё прописано по умолчанию.
  //	speex_encoder_ctl(stSpeexEncodeHandle, SPEEX_SET_QUALITY,&stSpeexQuality);
  //	speex_encoder_ctl(stSpeexEncodeHandle, SPEEX_SET_COMPLEXITY, &stSpeexComplexity);
  speex_decoder_ctl(stSpeexDecodeHandle, SPEEX_SET_ENH, (void *)&stSpeexEnh) ;


  HAL_TIM_Base_Start_IT(&htim2) ;						// Запускаем основной таймер на 8 кГц

  HAL_ADCEx_Calibration_Start(&hadc1);
  startUART_DMA ;

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
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the CPU, AHB and APB busses clocks 
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
  /** Initializes the CPU, AHB and APB busses clocks 
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
  hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Regular Channel 
  */
  sConfig.Channel = ADC_CHANNEL_8;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_1CYCLE_5;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief DAC Initialization Function
  * @param None
  * @retval None
  */
static void MX_DAC_Init(void)
{

  /* USER CODE BEGIN DAC_Init 0 */

  /* USER CODE END DAC_Init 0 */

  DAC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN DAC_Init 1 */

  /* USER CODE END DAC_Init 1 */
  /** DAC Initialization 
  */
  hdac.Instance = DAC;
  if (HAL_DAC_Init(&hdac) != HAL_OK)
  {
    Error_Handler();
  }
  /** DAC channel OUT1 config 
  */
  sConfig.DAC_Trigger = DAC_TRIGGER_NONE;
  sConfig.DAC_OutputBuffer = DAC_OUTPUTBUFFER_ENABLE;
  if (HAL_DAC_ConfigChannel(&hdac, &sConfig, DAC_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN DAC_Init 2 */

  /* USER CODE END DAC_Init 2 */

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 0;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 8999;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_SlaveConfigTypeDef sSlaveConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 0;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 159;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sSlaveConfig.SlaveMode = TIM_SLAVEMODE_EXTERNAL1;
  sSlaveConfig.InputTrigger = TIM_TS_ITR1;
  if (HAL_TIM_SlaveConfigSynchro(&htim3, &sSlaveConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */

}

/**
  * @brief TIM6 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM6_Init(void)
{

  /* USER CODE BEGIN TIM6_Init 0 */

  /* USER CODE END TIM6_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM6_Init 1 */

  /* USER CODE END TIM6_Init 1 */
  htim6.Instance = TIM6;
  htim6.Init.Prescaler = 9999;
  htim6.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim6.Init.Period = 7199;
  htim6.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim6) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_OnePulse_Init(&htim6, TIM_OPMODE_SINGLE) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim6, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM6_Init 2 */

  /* USER CODE END TIM6_Init 2 */

}

/**
  * @brief USART3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART3_UART_Init(void)
{

  /* USER CODE BEGIN USART3_Init 0 */

  /* USER CODE END USART3_Init 0 */

  /* USER CODE BEGIN USART3_Init 1 */

  /* USER CODE END USART3_Init 1 */
  huart3.Instance = USART3;
  huart3.Init.BaudRate = 115200;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART3_Init 2 */

  /* USER CODE END USART3_Init 2 */

}

/** 
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void) 
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();
  __HAL_RCC_DMA2_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel1_IRQn);
  /* DMA1_Channel3_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel3_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel3_IRQn);
  /* DMA2_Channel3_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Channel3_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Channel3_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_2, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_12, GPIO_PIN_RESET);

  /*Configure GPIO pin : PA0 */
  GPIO_InitStruct.Pin = GPIO_PIN_0;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PA1 PA2 */
  GPIO_InitStruct.Pin = GPIO_PIN_1|GPIO_PIN_2;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : PC12 */
  GPIO_InitStruct.Pin = GPIO_PIN_12;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI0_IRQn);

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
