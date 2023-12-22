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
#include "lcd.h"
#include "DHT.h"
#include "stm32f4xx_hal_tim.h"
#include "stm32f4xx_hal_conf.h"
#include "stdbool.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define BUTTON_INC_PIN GPIO_PIN_7
#define BUTTON_DEC_PIN GPIO_PIN_9
#define BUTTON_SET_PIN GPIO_PIN_8
#define BUTTON_RESET_PIN GPIO_PIN_10
#define BUZZER_PIN GPIO_PIN_5
#define DHT11_PIN GPIO_PIN_0
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;

TIM_HandleTypeDef htim10;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
Lcd_HandleTypeDef lcd;
DHT_DataTypedef DHT_Data;
volatile uint32_t timer_seconds = 0;
volatile uint32_t reminders_count = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_TIM10_Init(void);
static void MX_ADC1_Init(void);
/* USER CODE BEGIN PFP */
void displayDataOnLCD(float temperature, float humidity, uint32_t timer_seconds);
void startReminderTimer(uint32_t seconds);
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
	float temperature;
	float humidity;
	int timer_seconds = 10;
	float voltage;
	uint16_t adcValue;
	bool buzzer = false;
	bool warning = false;
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
  MX_USART2_UART_Init();
  MX_TIM10_Init();
  MX_ADC1_Init();
  /* USER CODE BEGIN 2 */
  // Lcd_PortType ports[] = { D4_GPIO_Port, D5_GPIO_Port, D6_GPIO_Port, D7_GPIO_Port };
    Lcd_PortType ports[] = { GPIOA, GPIOA, GPIOB, GPIOB };
    // Lcd_PinType pins[] = {D4_Pin, D5_Pin, D6_Pin, D7_Pin};
    Lcd_PinType pins[] = {GPIO_PIN_6, GPIO_PIN_5, GPIO_PIN_9, GPIO_PIN_8};
    // Lcd_create(ports, pins, RS_GPIO_Port, RS_Pin, EN_GPIO_Port, EN_Pin, LCD_4_BIT_MODE);
    lcd = Lcd_create(ports, pins, GPIOA, GPIO_PIN_7, GPIOB, GPIO_PIN_6, LCD_4_BIT_MODE);
    Lcd_clear(&lcd);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */
	  if (!buzzer){
		  HAL_ADC_Start(&hadc1); // Starting ADC Conversion
		  HAL_ADC_PollForConversion(&hadc1, HAL_MAX_DELAY); // Wait for completion of conversion
		  adcValue = HAL_ADC_GetValue(&hadc1); // Read conversion
		  HAL_ADC_Stop(&hadc1); // Stop conversion

			   // Convert ADC value to temperature (3.3 V)
		  voltage = (adcValue / 4095.0) * 3.3;
		  temperature = ((voltage - 0.5) * 100) - 1; // TMP36 linear voltage-temperature conversion
		  humidity = (temperature * 2.381) - 5;

		  if (!warning){
			 displayDataOnLCD(temperature, humidity, timer_seconds);
		  }

		  if (temperature > 25){
			 HAL_GPIO_WritePin(GPIOB, BUZZER_PIN, GPIO_PIN_SET);
			 warning = true;
			 Lcd_clear(&lcd);
			 Lcd_cursor(&lcd, 0, 0);
			 Lcd_string(&lcd, "UNSAFE TEMP/HUM!");
			 Lcd_cursor(&lcd, 1, 0);
			 Lcd_string(&lcd, "MOVE BOX!");
			 HAL_Delay(100);


		  }
		  else if(temperature <= 25){
			 HAL_GPIO_WritePin(GPIOB, BUZZER_PIN, GPIO_PIN_RESET);
			 warning = false;
		  }
	  }


 	  if (HAL_GPIO_ReadPin(GPIOA, BUTTON_SET_PIN) == GPIO_PIN_SET) {
 		  HAL_GPIO_WritePin(GPIOB, BUZZER_PIN, GPIO_PIN_RESET);
 		  buzzer = false;
 		  for(uint32_t time = timer_seconds; time >= 0; time--){
			  HAL_ADC_Start(&hadc1); // Starting ADC Conversion
			  HAL_ADC_PollForConversion(&hadc1, HAL_MAX_DELAY); // Wait for completion of conversion
			  adcValue = HAL_ADC_GetValue(&hadc1); // Read conversion
			  HAL_ADC_Stop(&hadc1); // Stop conversion

				   // Convert ADC value to temperature (3.3 V)
			  voltage = (adcValue / 4095.0) * 3.3;
			  temperature = ((voltage - 0.5) * 100) - 1; // TMP36 linear voltage-temperature conversion
			  humidity = (temperature * 2.381) - 5;
			  HAL_Delay(1000);
			  displayDataOnLCD(temperature, humidity, time);
				if (time == 0){
					break;
				}

 			}

 		  	reminders_count++;

		  	Lcd_clear(&lcd);
		  	Lcd_cursor(&lcd, 0, 0);
		  	Lcd_string(&lcd, "Take Your Meds!");

 		  	if (reminders_count == 4){
 		  		Lcd_clear(&lcd);
 		  		Lcd_cursor(&lcd, 0, 0);
 		  		Lcd_string(&lcd, "Refill Meds!");
 		  		reminders_count = 0;
 		  	}

			HAL_GPIO_WritePin(GPIOB, BUZZER_PIN, GPIO_PIN_SET);
			buzzer = true;
 	  }

 	  if (HAL_GPIO_ReadPin(GPIOB, BUTTON_RESET_PIN) == GPIO_PIN_SET) {
 		  // Reset everything
 		  timer_seconds = 0;
 		  reminders_count = 0;
 		  HAL_GPIO_WritePin(GPIOB, BUZZER_PIN, GPIO_PIN_RESET);
 		  buzzer = false;
 		  displayDataOnLCD(temperature, humidity, timer_seconds);
 	  }
 	  if (HAL_GPIO_ReadPin(GPIOC, BUTTON_INC_PIN) == GPIO_PIN_SET) {
 		  // Increase timer
 		  timer_seconds += 5; // You can adjust the increment as needed
 		  displayDataOnLCD(temperature, humidity, timer_seconds);
 	  }

 	  if (HAL_GPIO_ReadPin(GPIOA, BUTTON_DEC_PIN) == GPIO_PIN_SET) {
 		  // Decrease timer
 		  timer_seconds -= 5; // You can adjust the decrement as needed
 		  if (timer_seconds < 0) {
 			  timer_seconds = 0;
 		  }
 		  displayDataOnLCD(temperature, humidity, timer_seconds);
 	  }
 	  HAL_Delay(100);

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
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE2);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 16;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
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
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
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
  * @brief TIM10 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM10_Init(void)
{

  /* USER CODE BEGIN TIM10_Init 0 */

  /* USER CODE END TIM10_Init 0 */

  /* USER CODE BEGIN TIM10_Init 1 */

  /* USER CODE END TIM10_Init 1 */
  htim10.Instance = TIM10;
  htim10.Init.Prescaler = 83;
  htim10.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim10.Init.Period = 65535;
  htim10.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim10.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim10) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM10_Init 2 */

  /* USER CODE END TIM10_Init 2 */

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
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_7, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_8|GPIO_PIN_9, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : PA5 PA6 PA7 */
  GPIO_InitStruct.Pin = GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_7;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PB10 PB4 */
  GPIO_InitStruct.Pin = GPIO_PIN_10|GPIO_PIN_4;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : PC7 */
  GPIO_InitStruct.Pin = GPIO_PIN_7;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : PA8 PA9 */
  GPIO_InitStruct.Pin = GPIO_PIN_8|GPIO_PIN_9;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PB5 PB6 PB8 PB9 */
  GPIO_InitStruct.Pin = GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_8|GPIO_PIN_9;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
void displayDataOnLCD(float temperature, float humidity, uint32_t timer_seconds) {
    // Implement LCD display functionality here
    // Display temperature, humidity, and timer on the LCD
	// Assuming your LCD library provides these functions
	Lcd_clear(&lcd); // Clear the LCD display
	Lcd_cursor(&lcd, 0, 0); // Set the cursor to the first line

	// Display temperature and humidity
	Lcd_string(&lcd, "Temp: ");
	Lcd_string(&lcd, "Hum: ");
	Lcd_string(&lcd, "Time: ");
	Lcd_cursor(&lcd, 1, 0);
	Lcd_int(&lcd, temperature); // Assuming LCD_PrintFloat prints a floating-point number with 2 decimal places
	Lcd_string(&lcd, "    ");
	Lcd_int(&lcd, humidity);

	// Display timer value
	Lcd_string(&lcd, "    ");
	Lcd_int(&lcd, timer_seconds); // Assuming LCD_PrintInt prints an integer value
	Lcd_string(&lcd, " s");
	HAL_Delay(300);
}

void startReminderTimer(uint32_t seconds) {

	for(uint32_t time = seconds; time >= 0; time--){
		HAL_Delay(1000);
		displayDataOnLCD(0, 0, time);
		if (time == 0){
			break;
		}
	}

}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
    if (htim->Instance == TIM10) {
        // Timer 10 has elapsed, set the reminder flag or perform necessary actions
        timer_seconds = 0; // Set timer_seconds to 0 to trigger the reminder in the main loop
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
