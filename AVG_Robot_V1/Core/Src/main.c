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
#include "string.h"
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

TIM_HandleTypeDef htim2;

/* USER CODE BEGIN PV */
volatile uint16_t sensorValuesDMA[10];
const int sensorValuesDMA_ChannelCount = sizeof(sensorValuesDMA)/sizeof(sensorValuesDMA[0]);

uint16_t max_sensor_val[10];
uint16_t min_sensor_val[10];

const uint8_t sensorNumber = 10;
uint16_t sensorCalibratedValues[10];

int _lastValue;
uint16_t lineValue;

uint8_t baseSpeed = 150;
int P, I, D;

float Kp = 0.05;
float Ki = 0.00001;
float Kd = 0.8;

int motorCheckSpeedA;
int motorCheckSpeedB;

int lastError = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_ADC1_Init(void);
static void MX_TIM2_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

uint8_t commandBuffer[64];


uint8_t irSensorTxBuffer[130];
uint8_t lcd_text_buffer[50];
uint16_t lcd_buffer_length;

int lengthIrSensorTxBuffer;




void sensorCalibration(volatile uint16_t *sensor_values,
				  	   uint16_t *sensor_max_values,
					   uint16_t *sensor_min_values,
					   uint8_t   sensor_numbers,
					   uint16_t  cycle){
	int i;
	uint16_t max_temp_sensor_val[10];
	uint16_t min_temp_sensor_val[10];

	for(int a = 0; a < sensor_numbers; a++){
		max_temp_sensor_val[a] = 0;
		min_temp_sensor_val[a] = 4096;

	}

	int j;
	for(j = 0; j< cycle ;j++){
		for(i = 0; i< sensor_numbers; i++){
			if(j==0 || max_temp_sensor_val[i] < sensor_values[i]){
				max_temp_sensor_val[i] = sensor_values[i];
			}

			if(j==0 || min_temp_sensor_val[i] > sensor_values[i]){
				min_temp_sensor_val[i] = sensor_values[i];
			}
		}
	}

	for(i = 0; i < sensor_numbers; i++){
		if(min_temp_sensor_val[i] > sensor_max_values[i]){
			sensor_max_values[i] = min_temp_sensor_val[i];
		}
		if(max_temp_sensor_val[i] < sensor_min_values[i]){
			sensor_min_values[i] = max_temp_sensor_val[i];
		}
	}

}

int map(int x, int in_min, int in_max, int out_min, int out_max)
{
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

void readCalibrated(volatile uint16_t *sensor_values,
					uint16_t *sensor_calibrated_values,
					uint16_t *sensor_max_values,
					uint16_t *sensor_min_values,
					uint8_t   sensor_numbers){
	int i;

	for(i = 0; i < sensor_numbers; i++){
		uint16_t calmin, calmax;
		uint16_t denominator;

		calmax = sensor_max_values[i];
		calmin = sensor_min_values[i];

		denominator = calmax - calmin;

		signed int x = 0;
		if(denominator != 0){
			x = (((signed long)sensor_values[i]) - calmin) * 1000/denominator;

		}

		if(x < 0){
			x = 0;
		}
		else if(x>1000){
			x = 1000;
		}


		sensor_calibrated_values[i] = x;
	}

}

int read_ir_line(volatile uint16_t *sensor_values,
				 uint16_t *sensor_calibrated_values,
				 uint16_t *sensor_max_values,
				 uint16_t *sensor_min_values,
				 uint8_t   sensor_numbers,
				 uint8_t   white_line){
	uint8_t i, on_line = 0;
	unsigned long avg;
	unsigned int sum;

	readCalibrated(sensor_values,
			  	   sensor_calibrated_values,
				   sensor_max_values,
				   sensor_min_values,
				   sensor_numbers);

	avg = 0;
	sum = 0;

	for(i = 0; i < sensor_numbers; i++){
		int value = sensor_calibrated_values[i];
		if(white_line){
			value = 1000-value;
		}

		if(value > 400){
			on_line = 1;
		}

		if(value > 50){
			avg += (long)(value) * (i * 1000);
			sum += value;
		}
	}

	if(!on_line){
		if(_lastValue < (sensor_numbers - 1) * 1000/2){
			return 0;
		}
		else{
			return (sensor_numbers - 1)*1000;
		}
	}

	_lastValue = avg/sum;

	return _lastValue;


}

void PID_control(volatile uint16_t *sensor_values,
						  uint16_t *sensor_calibrated_values,
						  uint16_t *sensor_max_values,
						  uint16_t *sensor_min_values,
						  uint8_t   sensor_numbers,
						  uint8_t   white_line,
						  int mSpeedA,
						  int mSpeedB){

	lineValue = read_ir_line(sensor_values, sensor_calibrated_values, sensor_max_values, sensor_min_values, sensor_numbers, white_line);
	int error = 4500 - lineValue;

	P = error;
	I = error + I;
	D = error - lastError;
	lastError = error;

	int motorSpeedChange = P*Kp + I*Ki + D*Kd;

	int motorSpeedA = baseSpeed + motorSpeedChange;
	int motorSpeedB = baseSpeed - motorSpeedChange;
	mSpeedA = motorSpeedA;
	mSpeedB = motorSpeedB;

	motor_forward(motorSpeedA, motorSpeedB);

}

void motor_forward(int enableA, int enableB){

	if(enableA < 0){
		enableA = 0;
	}
	if(enableA > 255){
		enableA = 255;
	}
	if(enableB < 0){
		enableB = 0;
	}
	if(enableB > 255){
		enableB = 255;
	}

	//LEFT
	HAL_GPIO_WritePin(IN1_GPIO_Port, IN1_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(IN2_GPIO_Port, IN2_Pin, GPIO_PIN_SET);

	//RIGHT
	HAL_GPIO_WritePin(IN3_GPIO_Port, IN3_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(IN4_GPIO_Port, IN4_Pin, GPIO_PIN_SET);

	//Right
	__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, enableA);

	//Left
	__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, enableB);
}

void motor_sharp_left(uint8_t speed){
	//LEFT
	HAL_GPIO_WritePin(IN1_GPIO_Port, IN1_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(IN2_GPIO_Port, IN2_Pin, GPIO_PIN_RESET);

	//RIGHT
	HAL_GPIO_WritePin(IN3_GPIO_Port, IN3_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(IN4_GPIO_Port, IN4_Pin, GPIO_PIN_SET);

	//Right
	__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, speed);

	//Left
	__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, speed);
}

void motor_sharp_right(uint8_t speed){
	//LEFT
	HAL_GPIO_WritePin(IN1_GPIO_Port, IN1_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(IN2_GPIO_Port, IN2_Pin, GPIO_PIN_SET);

	//RIGHT
	HAL_GPIO_WritePin(IN3_GPIO_Port, IN3_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(IN4_GPIO_Port, IN4_Pin, GPIO_PIN_RESET);

	//Right
	__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, speed);

	//Left
	__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, speed);
}

void calibrationSequence(){

	 HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);
	 for(int a = 0; a<100; a++){
		  sensorCalibration(sensorValuesDMA,
							max_sensor_val,
							min_sensor_val,
							sensorNumber, 10);

		  if(a >= 0 && a < 25){
			  motor_sharp_right(120);
		  }
		  if(a >= 25 && a < 50){
			  motor_sharp_left(120);
		  }
		  if(a >= 50 && a < 75){
			  motor_sharp_right(120);
		  }
		  if(a >= 75 && a < 100){
			  motor_sharp_left(120);
		  }
		  HAL_Delay(50);
	 }
	 //readCalibrated(sensorValuesDMA, sensorCalibratedValues, max_sensor_val, min_sensor_val, sensorNumber);
	 while(sensorValuesDMA[4] > 2500){
		  motor_sharp_right(100);
	 }
	 HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_SET);
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
  MX_DMA_Init();
  MX_ADC1_Init();
  MX_TIM2_Init();
  /* USER CODE BEGIN 2 */
  for(int a = 0; a<10;a++){
  		max_sensor_val[a] = 0;
  		min_sensor_val[a] = 4096;

  	}
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2);

  HAL_ADC_Start_DMA(&hadc1, (uint16_t*) sensorValuesDMA, sensorValuesDMA_ChannelCount);


  calibrationSequence();
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
  RCC_OscInitStruct.PLL.PLLM = 12;
  RCC_OscInitStruct.PLL.PLLN = 96;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
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

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK)
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
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV8;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = ENABLE;
  hadc1.Init.ContinuousConvMode = ENABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 10;
  hadc1.Init.DMAContinuousRequests = ENABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_9;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_8;
  sConfig.Rank = 2;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_7;
  sConfig.Rank = 3;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_6;
  sConfig.Rank = 4;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_5;
  sConfig.Rank = 5;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_4;
  sConfig.Rank = 6;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_3;
  sConfig.Rank = 7;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_2;
  sConfig.Rank = 8;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_1;
  sConfig.Rank = 9;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_0;
  sConfig.Rank = 10;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

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
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 0;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 255;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */
  HAL_TIM_MspPostInit(&htim2);

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA2_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA2_Stream0_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream0_IRQn);

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
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, IN3_Pin|IN4_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, IN1_Pin|IN2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : PC13 */
  GPIO_InitStruct.Pin = GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : IN3_Pin IN4_Pin */
  GPIO_InitStruct.Pin = IN3_Pin|IN4_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : IN1_Pin IN2_Pin */
  GPIO_InitStruct.Pin = IN1_Pin|IN2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

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
