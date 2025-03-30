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

#include "ssd1306.h"
#include "fonts.h"
#include "test.h"
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
I2C_HandleTypeDef hi2c1;

TIM_HandleTypeDef htim1;

/* USER CODE BEGIN PV */
uint32_t previousMillis = 0;
uint32_t currentMillis = 0;
uint32_t counterOutside = 0; //For testing only
uint32_t counterInside = 0; //For testing only

uint8_t cursor = 1;
uint8_t okState = 0;
uint8_t okLastState = 1;

uint8_t upFlag = 0;
uint8_t downFlag = 0;

float Kp = 0.006;
float Ki = 0.06;
float Kd = 0.0006;
int readLine = 4500;

uint8_t kpState = 0;
uint8_t kiState = 0;
uint8_t kdState = 0;

uint8_t Kp_buffer[20];
uint8_t Ki_buffer[20];
uint8_t Kd_buffer[20];
uint8_t readLine_buffer[20];

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM1_Init(void);
static void MX_I2C1_Init(void);
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
  MX_TIM1_Init();
  MX_I2C1_Init();
  /* USER CODE BEGIN 2 */
  SSD1306_Init ();

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  if(cursor >3){cursor = 3;}
	  if(cursor <1){cursor = 1;}
	  SSD1306_Fill(0);
	  SSD1306_GotoXY (4, 16);
	  sprintf(Kp_buffer, "Kp: %0.3f", Kp);
	  if(okState == 1 && cursor == 1){
		  SSD1306_DrawFilledRectangle(1, 16-2, 80, 12, 1);
		  SSD1306_Puts (Kp_buffer, &Font_7x10, 0);
	  }
	  else if(okState == 0 && cursor == 1){
		  SSD1306_Puts (Kp_buffer, &Font_7x10, 1);
		  SSD1306_DrawRectangle(1, 16-2, 80, 12, 1);
	  }
	  else{
		  SSD1306_Puts (Kp_buffer, &Font_7x10, 1);
		  SSD1306_DrawRectangle(1, 16-2, 80, 12, 0);
	  }

	  SSD1306_GotoXY (4, 30);
	  sprintf(Ki_buffer, "Ki: %0.3f", Ki);
	  if(okState == 1 && cursor == 2){
		  SSD1306_DrawFilledRectangle(1, 30-2, 80, 12, 1);
		  SSD1306_Puts (Ki_buffer, &Font_7x10, 0);
	  }
	  else if(okState == 0 && cursor == 2){
		  SSD1306_Puts (Ki_buffer, &Font_7x10, 1);
		  SSD1306_DrawRectangle(1, 30-2, 80, 12, 1);
	  }
	  else{
		  SSD1306_Puts (Ki_buffer, &Font_7x10, 1);
		  SSD1306_DrawRectangle(1, 30-2, 80, 12, 0);
	  }

	  SSD1306_GotoXY (4, 44);
	  sprintf(Kd_buffer, "Kd: %0.3f", Kd);
	  if(okState == 1 && cursor == 3){
		  SSD1306_DrawFilledRectangle(1, 44-2, 80, 12, 1);
		  SSD1306_Puts (Kd_buffer, &Font_7x10, 0);
	  }
	  else if(okState == 0 && cursor == 3){

		  SSD1306_Puts (Kd_buffer, &Font_7x10, 1);
		  SSD1306_DrawRectangle(1, 44-2, 80, 12, 1);
	  }
	  else{

		  SSD1306_Puts (Kd_buffer, &Font_7x10, 1);
		  SSD1306_DrawRectangle(1, 44-2, 80, 12, 0);
	  }

	  //ReadLine
	  SSD1306_GotoXY (3, 1);
	  sprintf(readLine_buffer, "ReadLine: %d", readLine);
	  SSD1306_Puts (readLine_buffer, &Font_7x10, 1);


	  //right side deisgn
	  SSD1306_DrawTriangle(115, 1, 110, 14, 120, 14, 1);
	  if(upFlag == 1){
		  SSD1306_DrawFilledTriangle(115, 1, 110, 14, 120, 14, 1);
		  upFlag = 0;
	  }

	  SSD1306_DrawTriangle(115, 14+50, 110, 1+50, 120, 1+50, 1);
	  if(downFlag == 1){
		  SSD1306_DrawFilledTriangle(115, 14+50, 110, 1+50, 120, 1+50, 1);
		  downFlag = 0;
	  }

	  SSD1306_DrawCircle(115, 32, 5, 1);
	  if(okState == 1){
		  SSD1306_DrawFilledCircle(115, 32, 5, 1);
	  }



	  SSD1306_UpdateScreen();


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
  htim1.Init.Prescaler = 0;
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
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_7, GPIO_PIN_RESET);

  /*Configure GPIO pins : DOWN_Pin UP_Pin OK_Pin */
  GPIO_InitStruct.Pin = DOWN_Pin|UP_Pin|OK_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : PA4 PA5 PA6 PA7 */
  GPIO_InitStruct.Pin = GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_7;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
  currentMillis = HAL_GetTick();
  if (GPIO_Pin == DOWN_Pin && (currentMillis - previousMillis > 80))
  {
    HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_6);
    previousMillis = currentMillis;
    if(okState == 1 && cursor == 1){Kp -= 0.001;}
    else if(okState == 1 && cursor == 2){Ki -= 0.001;}
    else if(okState == 1 && cursor == 3){Kd -= 0.001;}
    else{cursor += 1;}
    downFlag = 1;


  }
  if (GPIO_Pin == UP_Pin && (currentMillis - previousMillis > 80))
  {
    HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_5);
    previousMillis = currentMillis;


    if(okState == 1 && cursor == 1){Kp += 0.001;}
    else if(okState == 1 && cursor == 2){Ki += 0.001;}
    else if(okState == 1 && cursor == 3){Kd += 0.001;}
    else{cursor -= 1;}
    upFlag = 1;


  }
  if (GPIO_Pin == OK_Pin && (currentMillis - previousMillis > 80))
  {
    HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_4);
    previousMillis = currentMillis;
    okLastState = okState;
    if(okLastState == 0){
    	okState = 1;

    }
    if(okLastState == 1){
    	okState = 0;

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
