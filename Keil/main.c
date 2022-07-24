/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2022 STMicroelectronics.
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
// includes for using strings
#include <stdio.h>
#include <string.h>

// Frequencies defines for using in goertzel algorithmes
//coefficient = 2 * (cos(2 * PI * (Freq / 8000.0))) 
// First Harmony:
//Row:
#define 	coefficient_Freq_697     1.7300996575860355		// Row Number: 1
#define 	coefficient_Freq_770     1.6722788385415637		// Row Number: 2
#define 	coefficient_Freq_852     1.6012981177149554		// Row Number: 3
#define 	coefficient_Freq_941     1.5173419552846932		// Row Number: 4
//Column:
#define 	coefficient_Freq_1209    1.2249619542608279		// Col Number: 1
#define 	coefficient_Freq_1336    1.0681880511122386		// Col Number: 2
#define 	coefficient_Freq_1477    0.8827313049309178		// Col Number: 3
#define 	coefficient_Freq_1633    0.6660201856861759		// Col Number: 4
// Second Harmony:
//Row:
#define 	coefficient_Freq_1394    0.993244825179317		// Row Number: 1
#define 	coefficient_Freq_1540    0.7965165138339212   // Row Number: 2
#define 	coefficient_Freq_1704    0.5641556617974589   // Row Number: 3
#define 	coefficient_Freq_1882    0.30232660926717536  // Row Number: 4
//Column:
#define 	coefficient_Freq_2418    -0.4994682106134935  // Col Number: 1
#define 	coefficient_Freq_2672    -0.8589742874610377  // Col Number: 2
#define 	coefficient_Freq_2954    -1.220785443294959   // Col Number: 3
#define 	coefficient_Freq_3266    -1.5564171122585517  // Col Number: 4

// Number of converted analog input number and it's array length
#define 	N		114 

// 7-segment:
#define		a 	0
#define		b 	1
#define		c 	2
#define		d 	3
#define		e 	4
#define		f 	5
#define		g 	6

// Masks:
#define 	SET1(x) 	(1UL << (x))
#define 	SET0(x) 	(~SET1(x))

// End Line String:
#define 	endl 	"\n\r"
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

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
// Samples array
static volatile double samples[N];

static volatile uint32_t seven_segment[4][4] = {
	{0x6, 0x5B, 0x4F, 0x77},
	{0x66, 0x6D, 0x7D, 0x7C},
	{0x07, 0x7F, 0x6F, 0x58},
	{0x76, 0x3F, 0x5C, 0x5E}
};
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_ADC1_Init(void);
static void MX_USART2_UART_Init(void);
/* USER CODE BEGIN PFP */
void PrintString(char* s);
void PrintNumber(uint32_t input);
uint32_t Read_ADC_OSC(void);
void GetAudioVoltage(void);
int GortzelAlgorithem(double coefficient);
void FindTone(void);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void PrintString(char* s){
	HAL_UART_Transmit(&huart2,(uint8_t*) s , strlen(s), HAL_MAX_DELAY);
}

void PrintNumber(uint32_t number){
	char numberString[10]; 
	sprintf(numberString, "%u", number);
	HAL_UART_Transmit(&huart2,(uint8_t*) numberString , strlen(numberString), HAL_MAX_DELAY);
}

uint32_t Read_ADC_OSC(){
	uint32_t raw;
	HAL_ADC_Start(&hadc1);
	HAL_ADC_PollForConversion(&hadc1, HAL_MAX_DELAY);
	raw = HAL_ADC_GetValue(&hadc1);
	HAL_ADC_Stop(&hadc1);
	return raw;
}

void GetAudioVoltage(){
	for(int i = 0; i < N; i++)
		samples[i] = ((((double) Read_ADC_OSC()) - 1401.0) * 5.0) / 317.0;
}

int GortzelAlgorithem(double coefficient){
  double  v1 = 0.0, v2 = 0.0;
	// for every samples we have to calculate the following formula: vk(n) = (2*cos(2*PI*f0/fs)) * vk(n-1) - vk(n-2) + x(n)
	// (2*cos(2*PI*f0/fs))  is    coefficient
	// vk(n-1)              is    v1
	// vk(n-2)              is    v2
	// x(n)                 is    samples[i]
  for(int i = 0; i < N; i++){
    double result = (coefficient * v1) - v2 + samples[i];
    v2 = v1;
    v1 = result;
  }
	// Returing the Gortzel result with the following formula: vk(N)2 + vk(N-1)2 - (2*cos(2*PI*f0/fs)) * vk(N) * vk(N-1))
	// vk(N)                is     v1
	// vk(N-1)              is     v2
	// 2*cos(2*PI*f0/fs)    is     coefficient
  return (int) ((v1 * v1) + (v2 * v2) - (coefficient * v1 * v2));
}

void FindTone(void){
	int row1 = -1, col1 = -1, row2 = -1, col2 = -1;
	int row_first_harmony[4], column_first_harmony[4], row_second_harmony[4], column_second_harmony[4];
	int max_row = 0x40, max_col = 0x50;
	
	row_first_harmony[0] = GortzelAlgorithem(coefficient_Freq_697);
	row_first_harmony[1] = GortzelAlgorithem(coefficient_Freq_770);
	row_first_harmony[2] = GortzelAlgorithem(coefficient_Freq_852);
	row_first_harmony[3] = GortzelAlgorithem(coefficient_Freq_941);
	
	column_first_harmony[0] = GortzelAlgorithem(coefficient_Freq_1209);
	column_first_harmony[1] = GortzelAlgorithem(coefficient_Freq_1336);
	column_first_harmony[2] = GortzelAlgorithem(coefficient_Freq_1477);
	column_first_harmony[3] = GortzelAlgorithem(coefficient_Freq_1633);
	
	for(int i = 0; i < 4; i++){
		if (max_row < row_first_harmony[i]){
			max_row = row_first_harmony[i];
			row1 = i;
		}
	}
	for(int i = 0; i < 4; i++){
		if (max_col < column_first_harmony[i]){
			max_col = column_first_harmony[i];
			col1 = i;
		}
	}
	
	if (row1 == -1 || col1 == -1){
		PrintString("Under The Min."); PrintString(endl);
		return;
	}
	
	max_row = 0x40;
	max_col = 0x50;
	
	row_second_harmony[0] = GortzelAlgorithem(coefficient_Freq_1394);
	row_second_harmony[1] = GortzelAlgorithem(coefficient_Freq_1540);
	row_second_harmony[2] = GortzelAlgorithem(coefficient_Freq_1704);
	row_second_harmony[3] = GortzelAlgorithem(coefficient_Freq_1882);
	
	column_second_harmony[0] = GortzelAlgorithem(coefficient_Freq_2418);
	column_second_harmony[1] = GortzelAlgorithem(coefficient_Freq_2672);
	column_second_harmony[2] = GortzelAlgorithem(coefficient_Freq_2954);
	column_second_harmony[3] = GortzelAlgorithem(coefficient_Freq_3266);
	
	
	for(int i = 0; i < 4; i++){
		if (max_row < row_second_harmony[i]){
			max_row = row_second_harmony[i];
			row2 = i;
		}
	}
	
	for(int i = 0; i < 4; i++){
		if (max_col < column_second_harmony[i]){
			max_col = column_second_harmony[i];
			col2 = i;
		}
	}
	
	if (row1 == row2 && col1 == col2){
		PrintString("Second Harmony."); PrintString(endl);
	}else {
		GPIOB -> ODR = seven_segment[row1][col1];
		PrintString("Row: "); PrintNumber(row1 + 1); PrintString(endl);
		PrintString("Col: "); PrintNumber(col1 + 1); PrintString(endl);
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
  MX_ADC1_Init();
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	PrintString(" ");
	PrintString(endl);
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
		GetAudioVoltage();
		FindTone();
		HAL_Delay(220);
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
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV4;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
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
  hadc1.Init.ScanConvMode = ENABLE;
  hadc1.Init.ContinuousConvMode = ENABLE;
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
  sConfig.SamplingTime = ADC_SAMPLETIME_480CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

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
  huart2.Init.Mode = UART_MODE_TX;
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

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3
                          |GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_6, GPIO_PIN_RESET);

  /*Configure GPIO pins : PB0 PB1 PB2 PB3
                           PB4 PB5 PB6 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3
                          |GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_6;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

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
