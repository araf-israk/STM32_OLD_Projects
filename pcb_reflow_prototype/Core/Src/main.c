/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
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
#include <math.h>
//#include <stdio.h>

#include "ssd1306.h"
#include "fonts.h"

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

I2C_HandleTypeDef hi2c1;

TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;

/* USER CODE BEGIN PV */


uint32_t ADC_Raw[1];

float Ntc_Tmp;
float Ntc_Tmp_Raw;
uint32_t Ntc_R;

float target_tmp = 200;

/* R1 resistance */
#define NTC_UP_R 100000.0f

#define FILTER_SMA_ORDER 15

/* constants of Steinhart-Hart equation */
#define A 0.0018937f
#define B -0.00003306639f
#define C 0.0000012931f


uint8_t Sch_100ms = 255;


uint8_t test_buffer[20];

uint16_t Filter_Buffer[FILTER_SMA_ORDER] = {0,};

volatile float P, I, D;

float Kp = 5;
float Ki = 0.001;
float Kd = 100;

volatile float pid_last_error;
volatile float pid_error;
volatile float pid_duty_cycle_change;


/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_TIM2_Init(void);
static void MX_I2C1_Init(void);
static void MX_ADC1_Init(void);
static void MX_TIM3_Init(void);
/* USER CODE BEGIN PFP */
#define SCREEN_HEIGHT 64
#define SCREEN_WIDTH 128
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

//typedef struct {
//
//	/* Controller gains */
//	float Kp;
//	float Ki;
//	float Kd;
//
//	/* Derivative low-pass filter time constant */
//	float tau;
//
//	/* Output limits */
//	float limMin;
//	float limMax;
//
//	/* Integrator limits */
//	float limMinInt;
//	float limMaxInt;
//
//	/* Sample time (in seconds) */
//	float T;
//
//	/* Controller "memory" */
//	float integrator;
//	float prevError;			/* Required for integrator */
//	float differentiator;
//	float prevMeasurement;		/* Required for differentiator */
//
//	/* Controller output */
//	float out;
//
//} PIDController;

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
 if(htim->Instance == TIM2)
	{
		/* set every 100ms */
		Sch_100ms = 255;
	}


}


void HAL_TIM_TriggerCallback(TIM_HandleTypeDef *htim){
	 if(htim->Instance == TIM3)
		{
		 HAL_GPIO_WritePin(GPIOA, GPIO_PIN_6, GPIO_PIN_SET);
		}
}


uint32_t Filter_SMA(uint32_t For_Filtered)
{
	/* Load new value */
	Filter_Buffer[FILTER_SMA_ORDER - 1] = For_Filtered;
	/* For output value */
	uint32_t Output = 0;
	/* Sum */
	for(uint8_t i = 0; i < FILTER_SMA_ORDER; i++)
	{
		Output += Filter_Buffer[i];
	}
	/* Divide */
	Output /= FILTER_SMA_ORDER;
	/* Left Shift */
	for(uint8_t i = 0; i < FILTER_SMA_ORDER; i++)
				Filter_Buffer[i] = Filter_Buffer[i+1];
	/* Return filtered value */
	return (uint32_t) Output;
}

void pwm_duty_cycle(uint8_t dc){

	htim3.Instance->CCR1 = ((float)(dc/100.0f))*(htim3.Instance->ARR);
}

void PID_control(volatile float *current_tmp, float target_tmp){

	pid_error = target_tmp - *current_tmp;

	P = pid_error;
	I = pid_error + I;
	D = pid_error - pid_last_error;
	pid_last_error = pid_error;
	pid_duty_cycle_change = P*Kp + I*Ki + D*Kd;

	if(pid_duty_cycle_change > 100){
		pid_duty_cycle_change = 100;
	}
	if(pid_duty_cycle_change < 0){
		pid_duty_cycle_change = 0;
	}

	pwm_duty_cycle(pid_duty_cycle_change);


}

//void PIDController_Init(PIDController *pid) {
//
//	/* Clear controller variables */
//	pid->integrator = 0.0f;
//	pid->prevError  = 0.0f;
//
//	pid->differentiator  = 0.0f;
//	pid->prevMeasurement = 0.0f;
//
//	pid->out = 0.0f;
//
//}
//
//float PIDController_Update(PIDController *pid, float setpoint, float measurement) {
//
//	/*
//	* Error signal
//	*/
//    float error = setpoint - measurement;
//
//
//	/*
//	* Proportional
//	*/
//    float proportional = pid->Kp * error;
//
//
//	/*
//	* Integral
//	*/
//    pid->integrator = pid->integrator + 0.5f * pid->Ki * pid->T * (error + pid->prevError);
//
//	/* Anti-wind-up via integrator clamping */
//    if (pid->integrator > pid->limMaxInt) {
//
//        pid->integrator = pid->limMaxInt;
//
//    } else if (pid->integrator < pid->limMinInt) {
//
//        pid->integrator = pid->limMinInt;
//
//    }
//
//
//	/*
//	* Derivative (band-limited differentiator)
//	*/
//
//    pid->differentiator = -(2.0f * pid->Kd * (measurement - pid->prevMeasurement)	/* Note: derivative on measurement, therefore minus sign in front of equation! */
//                        + (2.0f * pid->tau - pid->T) * pid->differentiator)
//                        / (2.0f * pid->tau + pid->T);
//
//
//	/*
//	* Compute output and apply limits
//	*/
//    pid->out = proportional + pid->integrator + pid->differentiator;
//
//    if (pid->out > pid->limMax) {
//
//        pid->out = pid->limMax;
//
//    } else if (pid->out < pid->limMin) {
//
//        pid->out = pid->limMin;
//
//    }
//
//	/* Store error and measurement for later use */
//    pid->prevError       = error;
//    pid->prevMeasurement = measurement;
//
//	/* Return controller output */
//    return pid->out;
//
//}

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
  MX_I2C1_Init();
  MX_ADC1_Init();
  MX_TIM3_Init();
  /* USER CODE BEGIN 2 */

  SSD1306_Init();

  HAL_ADCEx_Calibration_Start(&hadc1);

  HAL_ADC_Start_DMA(&hadc1, ADC_Raw, 1);

  HAL_TIM_Base_Start_IT(&htim2);

  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
		if(Sch_100ms){
			/* get adc value */
			//HAL_ADC_Start_IT(&hadc1);

			uint16_t Ntc_Temp_ADC = Filter_SMA(ADC_Raw[0]);
			/* calc. ntc resistance */
			Ntc_R = ((NTC_UP_R)/((4095.0/Ntc_Temp_ADC) - 1));

			/* temp */
			float Ntc_Ln = log(Ntc_R);
			/* calc. temperature */
			Ntc_Tmp = (1.0/(A + B*Ntc_Ln + C*Ntc_Ln*Ntc_Ln*Ntc_Ln)) - 273.15;
			HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
			/* nullify */
			Sch_100ms = 0;
		}
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_6, GPIO_PIN_RESET);
		PID_control(&Ntc_Tmp, target_tmp);
		 SSD1306_GotoXY(4, 10);
		 sprintf(test_buffer, "T: %d C ", Ntc_Tmp);
		 SSD1306_Puts(test_buffer, &Font_11x18, 1);

		 SSD1306_GotoXY(4, 30);
		 sprintf(test_buffer, "DC: %d ", Ntc_R);
		 SSD1306_Puts(test_buffer, &Font_11x18, 1);
//
//		 SSD1306_UpdateScreen();

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

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV2;
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
  hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc1.Init.ContinuousConvMode = ENABLE;
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
  sConfig.Channel = ADC_CHANNEL_0;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_55CYCLES_5;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 400000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

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
  htim2.Init.Prescaler = 125-1;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 57600-1;
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
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 320-1;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 45000-1;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */
  HAL_TIM_MspPostInit(&htim3);

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
  GPIO_InitTypeDef GPIO_InitStruct = {0};
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_7, GPIO_PIN_RESET);

  /*Configure GPIO pin : PC13 */
  GPIO_InitStruct.Pin = GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : PA7 */
  GPIO_InitStruct.Pin = GPIO_PIN_7;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

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
