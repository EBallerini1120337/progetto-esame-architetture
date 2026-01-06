/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2025 STMicroelectronics.
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
#include <stdbool.h>
#include "stm32f4xx_hal.h"
#include <string.h>
#include <stdio.h>

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

//structure for the records communicated via UART
typedef struct {
    RTC_TimeTypeDef start_time;
    RTC_DateTypeDef start_date;
    RTC_TimeTypeDef end_time;
    RTC_DateTypeDef end_date;
    bool valid;
} ActivityLogEntry;

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

#define TOKEN_DURATION 10 //number of TIM2 cycles a token lasts
#define MAX_TOKENS 5 //number of tokens (indicates the beginning of the activity)
#define LOG_DURATION 30 //number of TIM3 cycles before the log is sent via UART
#define LOG_SIZE 50 //number of entries in the log
#define TX_QUEUE_SIZE 50 //number of messages that can be queued for DMA communication
#define TX_BUF_SIZE 256 //length of a message

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;

RTC_HandleTypeDef hrtc;

TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;

UART_HandleTypeDef huart2;
DMA_HandleTypeDef hdma_usart2_tx;

/* USER CODE BEGIN PV */
bool tokens[MAX_TOKENS] = {false}; //token queue
uint16_t token_time[MAX_TOKENS] = {0}; //vector that keeps track of each token's lifespan
uint8_t count = 0; //number of active tokens
bool activity = {false}; //control if the tokens vector is full and the activity is taking place

ActivityLogEntry activity_log[LOG_SIZE]; //activity log
int log_index = 0; //index of the next log entry
uint16_t log_timer = LOG_DURATION; //time since last log communication

bool adc_state = false; //current state of the input (on/off)
bool last_adc_state = false; //previous state of the input (on/off)
uint32_t adc_value = 0; //analog value of the input

//variables for DMA communications via UART
char tx_queue[TX_QUEUE_SIZE][TX_BUF_SIZE]; //message queue
volatile int tx_head = 0; //next message
volatile int tx_tail = 0; //current message
volatile bool tx_busy = false; //control to check if transmission is available
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_TIM2_Init(void);
static void MX_RTC_Init(void);
static void MX_ADC1_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_TIM3_Init(void);
/* USER CODE BEGIN PFP */

//function to send one message through UART using DMA
void SendLine_DMA(const char *msg)
{
    int next = (tx_head + 1) % TX_QUEUE_SIZE; //goes circularly through the queue
    //if the queue is full the message is discarded
    if (next == tx_tail)
    {
        return;
    }

    //message is inserted in the queue
    snprintf(tx_queue[tx_head], TX_BUF_SIZE, "%s", msg);
    tx_head = next;

    //if transmission is available send next message using DMA, otherwise wait for interrupt
    if (!tx_busy)
    {
        tx_busy = true;

        HAL_UART_Transmit_DMA(&huart2,
                              (uint8_t*)tx_queue[tx_tail],
                              strlen(tx_queue[tx_tail]));
    }

    return;
}

//function for token generation
void enqueue_token(void)
{
	//if the queue has room insert a token and update count
	if (count < MAX_TOKENS)
	{
		tokens[count] = true;
		count++;

		//if the queue is now full and activity isn't taking place start activity and create new log entry
		if (count == MAX_TOKENS && activity == false)
		{
			activity = true;

			HAL_RTC_GetTime(&hrtc, &activity_log[log_index].start_time, RTC_FORMAT_BIN);
			HAL_RTC_GetDate(&hrtc, &activity_log[log_index].start_date, RTC_FORMAT_BIN);

		}
	}

	//regardless of queue state every token lifespan is resetted to reflect activity duration
	for (int i = 0; i < MAX_TOKENS; i++)
		{
			if(tokens[i] == true)
				token_time[i] = TOKEN_DURATION;
		}
}

//function for ADC reading
uint32_t Read_ADC()
{
    HAL_ADC_Start(&hadc1);
    HAL_ADC_PollForConversion(&hadc1, HAL_MAX_DELAY);
    uint32_t value = HAL_ADC_GetValue(&hadc1);
    HAL_ADC_Stop(&hadc1);
    return value;
}

//the Real Time Clock in the STM32F401 board doesn't keep track of time if the board isn't powered
//so it needs to be initialized at the start of every execution
void SyncRTCFromPC(void)
{
	//request for time synchronization
	char msg[] = "time_sync \n";

	//since the communication is brief and the execution can't start before the RTC is initialized
	//blocking communication is used instead of non-blocking communication using DMA
	if (HAL_UART_Transmit(&huart2, (uint8_t*)msg, strlen(msg), 1000) != HAL_OK)
	{
	    Error_Handler();
	}

	//the received date and time is first saved in a buffer
	char buf[32];
	uint8_t idx = 0;
	char c;
	while(1)
	{
		HAL_UART_Receive(&huart2,(uint8_t*)&c,1,HAL_MAX_DELAY);
		if(c == '\n') break;
		buf[idx++] = c;
		if(idx >= sizeof(buf)-1) break;
	}
	buf[idx] = '\0';

	//after communication is complete the buffer is used to initialize the RTC
	int y,m,d,h,min,s;
    if(sscanf(buf,"%d %d %d %d %d %d",&y,&m,&d,&h,&min,&s) == 6)
    {
        RTC_TimeTypeDef sTime = {0};
        RTC_DateTypeDef sDate = {0};

        sTime.Hours = h;
        sTime.Minutes = min;
        sTime.Seconds = s;
        sTime.DayLightSaving = RTC_DAYLIGHTSAVING_NONE;
        sTime.StoreOperation = RTC_STOREOPERATION_RESET;
        HAL_RTC_SetTime(&hrtc, &sTime, RTC_FORMAT_BIN);

        sDate.Year = y % 100;
        sDate.Month = m;
        sDate.Date = d;
        sDate.WeekDay = RTC_WEEKDAY_TUESDAY;
        HAL_RTC_SetDate(&hrtc, &sDate, RTC_FORMAT_BIN);
    }
}

//function for log communication
void SendFullActivityLog(void)
{
	//request for log communication
	char msg[] = "log_start \n";

	//since the handshake protocol is brief blocking communication is used
	HAL_UART_Transmit(&huart2,(uint8_t*)msg,strlen(msg),HAL_MAX_DELAY);

    int idx = 0;
    char syncbuf[32] = {0};

    while(idx < sizeof(syncbuf) - 1)
    {
    	uint8_t c;
    	HAL_UART_Receive(&huart2, &c, 1, HAL_MAX_DELAY);
    	syncbuf[idx++] = c;
    	if (c == '\n') break;
    }

    syncbuf[idx] = '\0';

    //if an ACK isn't received the communication is aborted
    if (strncmp(syncbuf, "ready", 5) != 0)
    {
    	char msg[] = "log_abort \n";
    	HAL_UART_Transmit(&huart2,(uint8_t*)msg,strlen(msg),HAL_MAX_DELAY);

    	return;
    }

    char buf[128];

    //each log entry is formatted for for the txt doc format
    for(int i = 0; i < LOG_SIZE; i++)
    {
        if(activity_log[i].valid)
        {
            RTC_DateTypeDef sd = activity_log[i].start_date;
            RTC_TimeTypeDef st = activity_log[i].start_time;
            RTC_DateTypeDef ed = activity_log[i].end_date;
            RTC_TimeTypeDef et = activity_log[i].end_time;

            snprintf(buf, sizeof(buf),
                "%d) start: %02d-%02d-%04d %02d:%02d:%02d    end: %02d-%02d-%04d %02d:%02d:%02d\n",
				i + 1,
                sd.Date, sd.Month, 2000 + sd.Year,
                st.Hours, st.Minutes, st.Seconds,
                ed.Date, ed.Month, 2000 + ed.Year,
                et.Hours, et.Minutes, et.Seconds
            );

            //all the log entries are added to the message queue and sent using DMA
            SendLine_DMA(buf);
        }
    }

    //end of communication
    SendLine_DMA("log_end\n");
}

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
  MX_TIM2_Init();
  MX_RTC_Init();
  MX_ADC1_Init();
  MX_USART2_UART_Init();
  MX_TIM3_Init();
  /* USER CODE BEGIN 2 */

  //starts the timers
  HAL_TIM_Base_Start_IT(&htim2);
  HAL_TIM_Base_Start_IT(&htim3);

  //Real Time Clock initialization
  SyncRTCFromPC();

    /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	//the board keeps doing ADC readings and converting them into on/off values
	adc_value = Read_ADC();
	adc_state = (adc_value > 2000);

	//if the input goes from 0 to 1 a new token is generated
	if (!last_adc_state && adc_state)
			{
		enqueue_token();
			}

	last_adc_state = adc_state;
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
  RCC_PeriphCLKInitTypeDef PeriphClkInitStruct = {0};

  /** Configure the main internal regulator output voltage 
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE2);
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_LSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.LSIState = RCC_LSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_RTC;
  PeriphClkInitStruct.RTCClockSelection = RCC_RTCCLKSOURCE_LSI;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
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
  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion) 
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV2;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time. 
  */
  sConfig.Channel = ADC_CHANNEL_0;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief RTC Initialization Function
  * @param None
  * @retval None
  */
static void MX_RTC_Init(void)
{

  /* USER CODE BEGIN RTC_Init 0 */

  /* USER CODE END RTC_Init 0 */

  RTC_TimeTypeDef sTime = {0};
  RTC_DateTypeDef sDate = {0};

  /* USER CODE BEGIN RTC_Init 1 */

  /* USER CODE END RTC_Init 1 */
  /** Initialize RTC Only 
  */
  hrtc.Instance = RTC;
  hrtc.Init.HourFormat = RTC_HOURFORMAT_24;
  hrtc.Init.AsynchPrediv = 127;
  hrtc.Init.SynchPrediv = 255;
  hrtc.Init.OutPut = RTC_OUTPUT_DISABLE;
  hrtc.Init.OutPutPolarity = RTC_OUTPUT_POLARITY_HIGH;
  hrtc.Init.OutPutType = RTC_OUTPUT_TYPE_OPENDRAIN;
  if (HAL_RTC_Init(&hrtc) != HAL_OK)
  {
    Error_Handler();
  }

  /* USER CODE BEGIN Check_RTC_BKUP */
    
  /* USER CODE END Check_RTC_BKUP */

  /** Initialize RTC and set the Time and Date 
  */
  sTime.Hours = 0x0;
  sTime.Minutes = 0x0;
  sTime.Seconds = 0x0;
  sTime.DayLightSaving = RTC_DAYLIGHTSAVING_NONE;
  sTime.StoreOperation = RTC_STOREOPERATION_RESET;
  if (HAL_RTC_SetTime(&hrtc, &sTime, RTC_FORMAT_BCD) != HAL_OK)
  {
    Error_Handler();
  }
  sDate.WeekDay = RTC_WEEKDAY_MONDAY;
  sDate.Month = RTC_MONTH_JANUARY;
  sDate.Date = 0x1;
  sDate.Year = 0x0;

  if (HAL_RTC_SetDate(&hrtc, &sDate, RTC_FORMAT_BCD) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN RTC_Init 2 */

  /* USER CODE END RTC_Init 2 */

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
  htim2.Init.Prescaler = 1599;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 999;
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
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
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

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 1599;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 9999;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */

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

  //for either an IDE bug or some configuration error the IDE wasn't automatically generating
  //the code to enable UART interrupts, so I added it manually
  //similarly in the stm32f4xx_it.c file I had to add the USART2_IRQHandler function manually
  HAL_NVIC_SetPriority(USART2_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(USART2_IRQn);
  /* USER CODE END USART2_Init 2 */

}

/** 
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void) 
{
  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Stream6_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream6_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream6_IRQn);

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
  __HAL_RCC_GPIOA_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_RESET);

  /*Configure GPIO pin : PA5 */
  GPIO_InitStruct.Pin = GPIO_PIN_5;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */
//function that gets called whenever a timer generates an interrupt
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	//if timer 2 generated the interrupt the board decrease the last tokens lifespan
	if (htim->Instance == TIM2)
	{

		int idx = count - 1;
		if (idx < 0) return;
		if (token_time[idx] > 0)
		{
			token_time[idx]--;

			//if the lifespan reaches 0 the token is removed
			if (token_time[idx] == 0)
			{
				tokens[idx] = false;
				if (count > 0) count--;

				//if the last token is removed and activity is taking place the activity ends
				//and time and date are stored in the log
				if (count == 0 && activity == true)
				{
					activity = false;

					HAL_RTC_GetTime(&hrtc, &activity_log[log_index].end_time, RTC_FORMAT_BIN);
					HAL_RTC_GetDate(&hrtc, &activity_log[log_index].end_date, RTC_FORMAT_BIN);

					activity_log[log_index].valid = true;
					log_index = (log_index + 1) % LOG_SIZE;

				}
			}
		}
	}

	//if timer 3 generated the interrupt the time until next communication is ticked down
	//then if it reaches 0 the log is sent via UART
	if(htim->Instance == TIM3)
	    {
	        log_timer--;

	        if(log_timer == 0)
	        {
	            log_timer = LOG_DURATION;
	            SendFullActivityLog();
	        }
	    }
}

//function that gets called whenever the UART generates an interrupt
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART2)
    {
        //goes to the next message
    	tx_tail = (tx_tail + 1) % TX_QUEUE_SIZE;

    	//if there is a next message it is sent using DMA
        if (tx_tail != tx_head)
        {
            HAL_UART_Transmit_DMA(&huart2,
                                  (uint8_t*)tx_queue[tx_tail],
                                  strlen(tx_queue[tx_tail]));
        }
        //if not transmission is now available
        else
        {
            tx_busy = false;
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
