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
  * This software component is licensed by ST under Ultimate Liberty license
  * SLA0044, the "License"; You may not use this file except in compliance with
  * the License. You may obtain a copy of the License at:
  *                             www.st.com/SLA0044
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
/* USER CODE END Includes */
#include "sensor.hpp"
#include "one_wire_sensor.hpp"
#include "timer.hpp"
#include "device_manager.hpp"
#include "pin_data.hpp"

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
UART_HandleTypeDef huart3;
TaskHandle_t defaultTaskHandle;
/* USER CODE BEGIN PV */
TaskHandle_t readoutTaskHandle;
SemaphoreHandle_t binarySem;

DeviceManager deviceManager;
PinData sensor1Data = {GPIOA, GPIO_PIN_4};
OneWireSensor tempSensor1(&sensor1Data);

PinData sensor2Data = {GPIOA, GPIO_PIN_9};
OneWireSensor tempSensor2(&sensor2Data);

uint16_t delayTime = 3000;
uint8_t data[5];
char uartData[50];
bool readDone;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
void MX_USART3_UART_Init(void);
void StartDefaultTask(void const * argument);
/* USER CODE BEGIN PFP */
void ReadoutTask(void const * argument);
static void MX_TIM7_Init(void);
static void MX_TIM6_Init(void);
static void MX_TIM4_Init(void);
void delayMicroseconds(uint32_t us);
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
  MX_USART3_UART_Init();
  MX_BlueNRG_MS_Init();
  /* USER CODE BEGIN 2 */
  MX_TIM7_Init();
  MX_TIM6_Init();
  MX_TIM4_Init();
  /* USER CODE END 2 */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  binarySem = xSemaphoreCreateBinary();
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* definition and creation of defaultTask */
  osThreadDef(defaultTask, StartDefaultTask, osPriorityNormal, 0, 128);
  defaultTaskHandle = osThreadCreate(osThread(defaultTask), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  osThreadDef(readoutTask, ReadoutTask, osPriorityNormal, 0, 256);
  readoutTaskHandle = osThreadCreate(osThread(readoutTask), NULL);
  /* USER CODE END RTOS_THREADS */

  /* Start scheduler */
  osKernelStart();

  /* We should never get here as control is now taken by the scheduler */

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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 10;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV7;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART2|RCC_PERIPHCLK_USART3;
  PeriphClkInit.Usart2ClockSelection = RCC_USART2CLKSOURCE_PCLK1;
  PeriphClkInit.Usart3ClockSelection = RCC_USART3CLKSOURCE_PCLK1;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure the main internal regulator output voltage
  */
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief USART3 Initialization Function
  * @param None
  * @retval None
  */
void MX_USART3_UART_Init(void)
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
  huart3.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart3.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART3_Init 2 */

  /* USER CODE END USART3_Init 2 */

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
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1|LD2_Pin|GPIO_PIN_8, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : PA0 */
  GPIO_InitStruct.Pin = GPIO_PIN_0;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PA1 LD2_Pin PA8 */
  GPIO_InitStruct.Pin = GPIO_PIN_1|LD2_Pin|GPIO_PIN_8;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI0_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(EXTI0_IRQn);

  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

}

/* USER CODE BEGIN 4 */
/**
  * @brief TIM7 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM7_Init(void)
{
  /* USER CODE BEGIN TIM7_Init 0 */
  /* USER CODE END TIM7_Init 0 */
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  /* USER CODE BEGIN TIM7_Init 1 */
  /* USER CODE END TIM7_Init 1 */
  htim7.Instance = TIM7;
  htim7.Init.Prescaler = 7999;
  htim7.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim7.Init.Period = 9;
  htim7.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim7) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim7, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM7_Init 2 */
  /* USER CODE END TIM7_Init 2 */
}

static void MX_TIM6_Init(void) //TODO: check&fix
{
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  htim6.Instance = TIM6;
  htim6.Init.Prescaler = 7999;
  htim6.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim6.Init.Period = 9;
  htim6.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim6) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim6, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
}

static void MX_TIM4_Init(void) //TODO: check&fix
{
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 7999;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 9;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
}
/* USER CODE END 4 */

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void const * argument)
{
	/* USER CODE BEGIN 5 */
	/* Infinite loop */
	for (;;) {
		//wyslij sygnal do taska drivera zeby pryzgotowal dane (ew. parametry = zmienne globalne?)
		xTaskNotify(readoutTaskHandle, 0x01, eSetBits); //a domyslnie kolejka (queue) requestow

		TickType_t maxBlockTime = pdMS_TO_TICKS(300UL);
		xSemaphoreTake(binarySem, maxBlockTime);
		//zeby tu dojsc, musial byc oddany semafor
		HAL_UART_Transmit(&huart3, (uint8_t *)"notified\r\n", 10, 10);
		//MX_BlueNRG_MS_Process(data, 5); //po wykonaniu odczytu
		osDelay(delayTime);
	}
	/* USER CODE END 5 */
}

/* USER CODE BEGIN 6 */
void ReadoutTask(void const * argument){
	uint32_t notifValue;
	/* Infinite loop */
	for (;;) {
		xTaskNotifyWait(pdFALSE, 0xFF, &notifValue, portMAX_DELAY);
		if ((notifValue & 0x01) != 0x00){
			tempSensor1.startNewReadout();
			//tempSensor2.startNewReadout();
		}
	}
}

void OneWireDriver::firstStateHandler(void){
//std::function<void()> OneWireDriver::firstStateHandler{
	//to co ma zrobic w tym stanie
	HAL_UART_Transmit(&huart3, (uint8_t *)"First state!\r\n", 14, 10);
	this->changePinMode(ONE_WIRE_OUTPUT);
	this->writePin(0);
	//ustaw kolejny stan
	this->stateHandler = static_cast<StateHandler>(&OneWireDriver::secondStateHandler);
	//przestaw i uruchom timer
	this->timer->wakeMeUpAfterMicroseconds(800);
}

void OneWireDriver::secondStateHandler(void){
	//to co ma zrobic w tym stanie
	HAL_UART_Transmit(&huart3, (uint8_t *)"Second state!\r\n", 15, 10);
	this->writePin(1);
	this->changePinMode(ONE_WIRE_INPUT);
	//ustaw kolejny stan
	this->stateHandler = static_cast<StateHandler>(&OneWireDriver::thirdStateHandler);
	//przestaw i uruchom timer
	this->timer->wakeMeUpAfterMicroseconds(10);
}

void OneWireDriver::thirdStateHandler(void){
	//to co ma zrobic w tym stanie
	while(this->readPin()); //kolejne state'y
	while(!this->readPin());//..
	while(this->readPin());
	uint32_t rawBits = 0UL;
	uint8_t checksumBits = 0;
	//glowna czesc - odczyt danych i sumy kontrolnej
	for (int8_t i = 31; i >= 0; i--){	//Read 32 bits of temp.&humidity data
		/*
		 * Bit data "0" signal: the level is LOW for 50ms and HIGH for 26-28ms;
		 * Bit data "1" signal: the level is LOW for 50ms and HIGH for 70ms;
		 * MAX FREQUENCY ON STM32L476RG = 80MHz
		 * SO IT TAKES 12,5 ns FOR ONE INSTRUCTION TO EXECUTE
		 * A DELAY OF 1 SECOND (x TICKS): 80 MILLION NOP INSTRUCTIONS TO EXECUTE
		 */
		while (!this->readPin());
		delayMicroseconds(50);
		if (this->readPin()) {
			rawBits |= (1UL << i);
		}
		while (this->readPin());
	}
	for (int8_t i = 7; i >= 0; i--){		//Read 8 bits of checksum data
		while (!this->readPin());
		delayMicroseconds(50);
		if (this->readPin()) {
			checksumBits |= (1UL << i);
		}
		while (this->readPin());
	}
	uint8_t data[5];
	data[0] = (rawBits >> 24) & 0xFF;//Copy raw data bits to an array of bytes
	data[1] = (rawBits >> 16) & 0xFF;
	data[2] = (rawBits >> 8) & 0xFF;
	data[3] = (rawBits >> 0) & 0xFF;
	data[4] = (checksumBits) & 0xFF;
	uint16_t humid = (data[0] << 8) | data[1];
	uint16_t temp = (data[2] << 8) | data[3];
	uint16_t humidDecimal = humid % 10;
	uint16_t tempDecimal = temp % 10;
	temp = temp / (uint16_t) 10;
	humid = humid / (uint16_t) 10;
	MX_BlueNRG_MS_Process(data, sizeof(data)); //zamiast w glownym tasku po oddaniu semafora
	char uartData[50];
	sprintf(uartData, "\r\nTemperatura\t %hu.%huC\r\nWilgotnosc\t %hu.%hu%%\r\n",
			temp, tempDecimal, humid, humidDecimal);
	HAL_UART_Transmit(&huart3, (uint8_t *)uartData, /*sizeof(uartData)*/ 42, 10);


	//TODO: wypisz dane odebrane od mastera
	HAL_UART_Transmit(&huart3, rcvBLE, sizeof(rcvBLE), 10);


	//powiadom glowny task ze juz zakonczyla sie cala robota
	BaseType_t xHigherPriorityTaskWoken = pdFALSE;
	xSemaphoreGiveFromISR(binarySem, &xHigherPriorityTaskWoken);
	portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

void delayMicroseconds(uint32_t us){
	//Average, experimental time for 1 rotation of the 'for' loop with nops: ~140ns
	//for an 80MHz processor@max speed; that gives ~7.143 loop rotations for 1 ms
	//Use this fact and the processor frequency to adjust the loop counter value for any processor speed
	uint32_t clockFreq = HAL_RCC_GetHCLKFreq();	//Current processor frequency
	float clockFreqRel = clockFreq/(float)80000000.0;//Current processor freq. relative to base of 80MHz
	uint32_t loopCounter = (us > 0 ? (uint32_t)(us*clockFreqRel*7.143) : (uint32_t)(clockFreqRel*7.143));
	//uint32_t loopCounter = (us > 0 ? (uint32_t)(us*7.143) : 7); //A minimum delay of 1 us - 80MHz only
	for(uint32_t tmp = 0; tmp < loopCounter; tmp++) {asm volatile("nop");}
	//previously there was tmp < 800 giving 3200 processor cycles, each lasting 12.5 ns = 40 us delay
	//UINT_MAX	Maximum value for a variable of type unsigned int	4,294,967,295 (0xffffffff)
}

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM3 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  if (htim->Instance == TIM3) {
	HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 0 */
  else {
	  timers[deviceManager.getTimerIndex(htim)]->executeCallback(); //a w nim ExecuteState urzadzenia
  }

  /* USER CODE END Callback 0 */

  /* USER CODE BEGIN Callback 1 */

  /* USER CODE END Callback 1 */
}

/* USER CODE END 6 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
	HAL_UART_Transmit(&huart3, (uint8_t *)"Error Handler!\r\n", 20, 10);
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
void assert_failed(char *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
