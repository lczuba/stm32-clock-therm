/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
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
#include "nokia5110_LCD.h"
#include "math.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
float Tc = 0;
double temp_alarm_level = 26;
char degree[] = {127, '\0'};
int state = 0;	//0 = temperature; 1 = clock; 2 = credits;
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc2;

RTC_HandleTypeDef hrtc;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
RTC_TimeTypeDef sTime;
RTC_DateTypeDef sDate;

uint16_t ADC_RES = 0;

uint8_t UART2_rxBuffer[1] = {0};
uint8_t UART2_rxBufferAll[24] = {0};
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_ADC2_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_RTC_Init(void);
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
  MX_ADC2_Init();
  MX_USART2_UART_Init();
  MX_RTC_Init();
  /* USER CODE BEGIN 2 */
  HAL_UART_Receive_IT (&huart2, UART2_rxBuffer, 1);
  HAL_ADC_Start_IT(&hadc2);		//Start ADC2

  //*Start Init LCD
  LCD_setRST(GPIOB, GPIO_PIN_4);
  LCD_setCE(GPIOB, GPIO_PIN_5);
  LCD_setDC(GPIOB, GPIO_PIN_6);
  LCD_setDIN(GPIOB, GPIO_PIN_8);
  LCD_setCLK(GPIOB, GPIO_PIN_9);
  LCD_init();
  //*End Init LCD

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  updateDisplay();
	  HAL_Delay(1000);

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

  /** Configure LSE Drive Capability
  */
  HAL_PWR_EnableBkUpAccess();
  __HAL_RCC_LSEDRIVE_CONFIG(RCC_LSEDRIVE_LOW);
  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_LSE;
  RCC_OscInitStruct.LSEState = RCC_LSE_ON;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL16;
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
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_RTC;
  PeriphClkInit.RTCClockSelection = RCC_RTCCLKSOURCE_LSE;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC2_Init(void)
{

  /* USER CODE BEGIN ADC2_Init 0 */

  /* USER CODE END ADC2_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC2_Init 1 */

  /* USER CODE END ADC2_Init 1 */
  /** Common config
  */
  hadc2.Instance = ADC2;
  hadc2.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
  hadc2.Init.Resolution = ADC_RESOLUTION_12B;
  hadc2.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc2.Init.ContinuousConvMode = ENABLE;
  hadc2.Init.DiscontinuousConvMode = DISABLE;
  hadc2.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc2.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc2.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc2.Init.NbrOfConversion = 1;
  hadc2.Init.DMAContinuousRequests = DISABLE;
  hadc2.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc2.Init.LowPowerAutoWait = DISABLE;
  hadc2.Init.Overrun = ADC_OVR_DATA_OVERWRITTEN;
  if (HAL_ADC_Init(&hadc2) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_6;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SingleDiff = ADC_SINGLE_ENDED;
  sConfig.SamplingTime = ADC_SAMPLETIME_601CYCLES_5;
  sConfig.OffsetNumber = ADC_OFFSET_NONE;
  sConfig.Offset = 0;
  if (HAL_ADC_ConfigChannel(&hadc2, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC2_Init 2 */

  /* USER CODE END ADC2_Init 2 */

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
  sTime.Hours = 23;
  sTime.Minutes = 59;
  sTime.Seconds = 59;
  sTime.DayLightSaving = RTC_DAYLIGHTSAVING_NONE;
  sTime.StoreOperation = RTC_STOREOPERATION_RESET;
  if (HAL_RTC_SetTime(&hrtc, &sTime, RTC_FORMAT_BIN) != HAL_OK)
  {
    Error_Handler();
  }
  sDate.WeekDay = RTC_WEEKDAY_MONDAY;
  sDate.Month = RTC_MONTH_DECEMBER;
  sDate.Date = 31;
  sDate.Year = 20;

  if (HAL_RTC_SetDate(&hrtc, &sDate, RTC_FORMAT_BIN) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN RTC_Init 2 */

  /* USER CODE END RTC_Init 2 */

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
  huart2.Init.BaudRate = 115200 ;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
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

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_8
                          |GPIO_PIN_9, GPIO_PIN_RESET);

  /*Configure GPIO pin : PC13 */
  GPIO_InitStruct.Pin = GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : PC0 */
  GPIO_InitStruct.Pin = GPIO_PIN_0;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : LD2_Pin */
  GPIO_InitStruct.Pin = LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LD2_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : PB4 PB5 PB6 PB8
                           PB9 */
  GPIO_InitStruct.Pin = GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_8
                          |GPIO_PIN_9;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

}

/* USER CODE BEGIN 4 */

void calculateTemp() {
	float ADC_Max = 4095.0;
	float Vs = 3.3;
	float R1 = 1000.0;		//Voltage divider resistance value
	float Beta = 3950.0;	//Beta value
	float To = 298.15;		//Temperature in Kelvin for 25 degree Celsius
	float Ro = 1000.0;		//Resistance of Thermistor at 25 degree Celsius
	float Vout, Rt = 0;

	Vout = ADC_RES * Vs / ADC_Max;
	Rt = R1 * Vout / (Vs - Vout);
	Tc = 1/(1/To + log(Rt/Ro)/Beta) - 273.15;  // Temperature in Celsius
}

void updateDisplayTemp() {
	LCD_clrScr();
	LCD_print("Temperature: ", 6, 0);

	calculateTemp();
	char temp[8];
	sprintf(temp, "%.1f", Tc);
	strcat(temp, degree);
	strcat(temp, "C");
	LCD_print(temp, 25, 1);

	LCD_print("Alarm: ", 0, 3);
	sprintf(temp, "%.1f", temp_alarm_level);
	strcat(temp, degree);
	strcat(temp, "C");
	LCD_print(temp, 40, 3);

	if(Tc >= temp_alarm_level) {
		LCD_print("!!!TOO HOT!!!", 2, 5);
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_SET);
	}
	else {
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_RESET);
	}
}

void updateDisplayClock() {
	LCD_clrScr();
	HAL_RTC_GetTime(&hrtc, &sTime, RTC_FORMAT_BIN);
	HAL_RTC_GetDate(&hrtc, &sDate, RTC_FORMAT_BIN);

	char Message[64];
	sprintf((char*)Message, "%02d:%02d:%02d", sTime.Hours, sTime.Minutes, sTime.Seconds);

	LCD_print(Message, 16, 1);
	sprintf((char*)Message, "%02d.%02d.20%02d", sDate.Date, sDate.Month, sDate.Year);
	LCD_print(Message, 12, 3);
}

void updateDisplayCredits() {
	LCD_clrScr();
	LCD_print("Authors: ", 0, 1);
	LCD_print("Czuba Lukasz", 0, 2);
	LCD_print("Wojciech Jacoszek", 0, 3);
	LCD_print("Jan Krupinski", 0, 4);
}

void updateDisplay() {
	if(state == 0) updateDisplayTemp();
	else if(state == 1) updateDisplayClock();
	else if(state == 2) updateDisplayCredits();
}

void changeDisplayState() {
	if(state == 2) state = 0;
	else state++;

	updateDisplay();
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	changeDisplayState();
}

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc2)
{
    // Read & Update The ADC Result
	ADC_RES = HAL_ADC_GetValue(hadc2);
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	if( UART2_rxBuffer[0] == '/') {
		for(int i = 0; i < 24; i++) UART2_rxBufferAll[i] = 0;
		UART2_rxBufferAll[0] = UART2_rxBuffer[0];
		HAL_UART_Transmit(&huart2, "\n", 1, 100);
	}
	else if(UART2_rxBuffer[0] == 13) { // ENTER

		uint16_t command_size = sprintf(UART2_rxBufferAll, UART2_rxBufferAll);
		char command[command_size + 1];

		int i = 0;
		while( (i < command_size) && (UART2_rxBufferAll[i] != 32) ) {
			command[i] = UART2_rxBufferAll[i];
			i++;
		}
		command[i] = '\0';

		char argument[command_size];
		if(UART2_rxBufferAll[i] == 32) {	//if space exist
			i++; // pass space
			uint16_t argument_size = command_size - i;
			int j = 0;
			while( i < command_size) {
				argument[j] = UART2_rxBufferAll[i];
				i++;
				j++;
			}
			argument[argument_size] = '\0';
		}

		char command_help[] = "/help";
		char command_setAlarm[] = "/setAlarm";
		char command_getTemp[] = "/getTemp";
		char command_getTime[] = "/getTime";
		char command_setTime[] = "/setTime";
		char command_setDate[] = "/setDate";

		int menu_help = strcmp(command_help, command);
		int menu_setAlarm = strcmp(command_setAlarm, command);
		int menu_getTemp = strcmp(command_getTemp, command);
		int menu_getTime = strcmp(command_getTime, command);
		int menu_setTime = strcmp(command_setTime, command);
		int menu_setDate = strcmp(command_setDate, command);

		if(menu_help == 0) {
			char message[] = "\n/getTemp - wyswietla aktualna tempreature\n"
					"/setAlarm NN.N - ustawia granice temperatury (np. /setAlarm 30.5)\n"
					"/getTime - wyswietla aktualny czas\n"
					"/setDate DD:MM:YY - ustawia date\n"
					"/setTime HH:MM:SS - ustawia godzine";
			HAL_UART_Transmit(&huart2, message, sizeof(message), 100);
		}
		else if(menu_setAlarm == 0) {
			temp_alarm_level = ((int)argument[0] - 48) * 10;
			temp_alarm_level += (int)argument[1] - 48;
			temp_alarm_level += ((double)argument[3] - 48) / 10.0;

			char message[] = "\nUstawiono alarm temperatury";
			HAL_UART_Transmit(&huart2, message, sizeof(message), 100);
		}
		else if(menu_getTemp == 0) {
			calculateTemp();
			char temp[8];
			sprintf(temp, "%.1f", Tc);
			strcat(temp, " C");

			char message[32];
			sprintf(message, "\nAktualna temperatura: %s", temp);
			uint16_t size = sprintf(message, message);
			HAL_UART_Transmit(&huart2, message, size, 100);
		}
		else if(menu_getTime == 0) {
			HAL_RTC_GetTime(&hrtc, &sTime, RTC_FORMAT_BIN);
			HAL_RTC_GetDate(&hrtc, &sDate, RTC_FORMAT_BIN);

			char Message[124];
			int MessageLen = sprintf((char*)Message, "\nDate: %02d.%02d.20%02d Time: %02d:%02d:%02d\n\r", sDate.Date, sDate.Month, sDate.Year, sTime.Hours, sTime.Minutes, sTime.Seconds);
			HAL_UART_Transmit(&huart2, Message, MessageLen, 100);

		}
		else if(menu_setTime == 0) {
			int time_h = ((int)argument[0] - 48) * 10 + (int)argument[1] - 48;
			sTime.Hours = time_h;

			int time_m = ((int)argument[3] - 48) * 10 + (int)argument[4] - 48;
			sTime.Minutes = time_m;

			int time_s = ((int)argument[6] - 48) * 10 + (int)argument[7] - 48;
			sTime.Seconds = time_s;

			if (HAL_RTC_SetTime(&hrtc, &sTime, RTC_FORMAT_BIN) != HAL_OK) {
				char message[] = "\nError";
				HAL_UART_Transmit(&huart2, message, sizeof(message), 100);
				Error_Handler();
			}
			else {
				char message[] = "\nSuccess";
				HAL_UART_Transmit(&huart2, message, sizeof(message), 100);
			}
		}
		else if(menu_setDate == 0) {
					int date_d = ((int)argument[0] - 48) * 10 + (int)argument[1] - 48;
					sDate.Date = date_d;

					int date_m = ((int)argument[3] - 48) * 10 + (int)argument[4] - 48;
					sDate.Month = (uint8_t)date_m;

					int date_y = ((int)argument[6] - 48) * 10 + (int)argument[7] - 48;
					sDate.Year = date_y;

					if (HAL_RTC_SetDate(&hrtc, &sDate, RTC_FORMAT_BIN) != HAL_OK) {
						char message[] = "\nError";
						HAL_UART_Transmit(&huart2, message, sizeof(message), 100);
						Error_Handler();
					}
					else {
						char message[] = "\nSuccess";
						HAL_UART_Transmit(&huart2, message, sizeof(message), 100);
					}

				}

	}
	else if(UART2_rxBufferAll[0] == '/') {
		for(int i = 0; i < 24; i++) {
			if(UART2_rxBufferAll[i] == 0) {
				UART2_rxBufferAll[i] = UART2_rxBuffer[0];
				break;
			}
		}
	}

	HAL_UART_Transmit(&huart2, UART2_rxBuffer, 1, 100);
    HAL_UART_Receive_IT(&huart2, UART2_rxBuffer, 1);
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
