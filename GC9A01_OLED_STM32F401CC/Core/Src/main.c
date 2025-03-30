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
#include "GC9A01.h"

#include "bitmap.h"
#include "fonts.h"


#include "stdio.h"

uint8_t hh,mm,ss;

#define PI 	3.14159265
#define xC	120
#define yC	120
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
RTC_HandleTypeDef hrtc;

SPI_HandleTypeDef hspi1;
DMA_HandleTypeDef hdma_spi1_rx;
DMA_HandleTypeDef hdma_spi1_tx;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_SPI1_Init(void);
static void MX_RTC_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
static void DrawArrow(int16_t angle, uint8_t lineLen, uint8_t thick, uint16_t color) {

	angle -= 90;
	float angleRad = (float) angle * PI / 180;
	int x = cos(angleRad) * lineLen + xC;
	int y = sin(angleRad) * lineLen + yC;

	GC9A01_DrawLineThick(xC, yC, x, y, color, thick);
}

void DrawClock(uint8_t hour, uint8_t min, uint8_t sec, uint8_t light, uint8_t secBubbles) {

	uint16_t bgColor, riskColor, digitColor, arrowColor, secArcColor;

	if (light) {
		bgColor = GC9A01_WHITE;
		riskColor = digitColor = arrowColor = GC9A01_BLACK;
		secArcColor = GC9A01_MAGENTA;
	} else {
		bgColor = GC9A01_BLACK;
		riskColor = digitColor = arrowColor = GC9A01_WHITE;
		secArcColor = GC9A01_GREEN;
	}

//	GC9A01_FillScreen(bgColor);   //NO DMA
	ClearScreen2(GC9A01_BLACK);   //DMA


	uint8_t radius1 = 119;
	for (uint16_t angle = 0; angle <= 360; angle += 6) {
		uint8_t riskSize;
		if (!(angle % 90))
			riskSize = 13;
		else if (!(angle % 30))
			riskSize = 10;
		else
			riskSize = 6;

		uint8_t radius2 = radius1 - riskSize;
		float angleRad = (float) angle * PI / 180;
		int x1 = cos(angleRad) * radius1 + xC;
		int y1 = sin(angleRad) * radius1 + yC;
		int x2 = cos(angleRad) * radius2 + xC;
		int y2 = sin(angleRad) * radius2 + yC;

		GC9A01_DrawLine(x1, y1, x2, y2, riskColor);
	}


	GC9A01_print( 165, 30, digitColor , RGB565(0, 10, 100) , 0, &Font_11x18, 1, "1" );
	GC9A01_print( 200, 63, digitColor , RGB565(0, 10, 100) , 0, &Font_11x18, 1, "2" );
	GC9A01_print( 207, 110, digitColor , RGB565(0, 10, 100) , 0, &Font_16x26, 1, "3" );
	GC9A01_print( 200, 160, digitColor , RGB565(0, 10, 100) , 0, &Font_11x18, 1, "4" );
	GC9A01_print( 165, 193, digitColor , RGB565(0, 10, 100) , 0, &Font_11x18, 1, "5" );
	GC9A01_print( 110, 200, digitColor , RGB565(0, 10, 100) , 0, &Font_16x26, 1, "6" );
	GC9A01_print( 65, 193, digitColor , RGB565(0, 10, 100) , 0, &Font_11x18, 1, "7" );
	GC9A01_print( 32, 160, digitColor , RGB565(0, 10, 100) , 0, &Font_11x18, 1, "8" );
	GC9A01_print( 19, 110, digitColor , RGB565(0, 10, 100) , 0, &Font_16x26, 1, "9" );
	GC9A01_print( 32, 63, digitColor , RGB565(0, 10, 100) , 0, &Font_11x18, 1, "10" );
	GC9A01_print( 65, 30, digitColor , RGB565(0, 10, 100) , 0, &Font_11x18, 1, "11" );
	GC9A01_print( 106, 20, digitColor , RGB565(0, 10, 100) , 0, &Font_16x26, 1, "12" );

	DrawArrow(min * 6 + sec / 10, 80, 2, arrowColor);

	DrawArrow(hour * 30 + min / 2, 50, 4, arrowColor);

	if (!sec)
		sec = 60;
	if (secBubbles) {
		int16_t startAngle = -90;
		int16_t endAngle = sec * 6 - 90;

		for (int16_t angle = startAngle; angle <= endAngle; angle += 6) {
			float angleRad = (float) angle * PI / 180;
			int x = cos(angleRad) * 119 + xC;
			int y = sin(angleRad) * 119 + yC;

			if (angle == endAngle)
				GC9A01_DrawCircleFilled(x, y, 4, secArcColor);
			else
				GC9A01_DrawCircleFilled(x, y, 2, secArcColor);
		}
	} else
		GC9A01_DrawArc(xC, yC, 119, 0, sec * 6, secArcColor, 2);


	//HAL_Delay(50);
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
  MX_SPI1_Init();
  MX_RTC_Init();
  /* USER CODE BEGIN 2 */
  GC9A01_Init();
//  	GC9A01_FillScreen( RGB565(255, 0, 0) );
//  	HAL_Delay (1000);
//  	GC9A01_FillScreen( RGB565(0, 255, 0) );
//  	HAL_Delay (1000);
//  	GC9A01_FillScreen( RGB565(0, 0, 255) );
//  	HAL_Delay (1000);
//  	GC9A01_FillScreen( RGB565(57, 179, 185) );
//  	HAL_Delay (1000);
//  	GC9A01_DrawPixel( 12, 12, RGB565( 255, 255, 255 ) );
//  	HAL_Delay (1000);
//    GC9A01_DrawChar( 120, 20, RGB565( 228, 233, 50 ) , RGB565( 0, 10, 10 ) , 0, &Font_16x26, 1, 'F' );
//  	HAL_Delay (1000);
//  	GC9A01_Clear();
//  	GC9A01_DrawRectangleFilled(105, 105, 30, 30, RGB565(255, 255, 255)) ;
//  	HAL_Delay (1000);
//  	for( uint8_t i = 0; i< GC9A01_Width; i+=3){
//  		GC9A01_DrawRectangleFilled(i, i, GC9A01_Width-i, GC9A01_Height-i, RGB565(i/2, 255-i, 0+i)) ;
//  	}
//
//  	for( uint8_t i = 0; i< GC9A01_Width/2; i+=3){
//  		GC9A01_DrawRectangle(i, i, GC9A01_Width-i, GC9A01_Height-i, RGB565(255, 0, 0)) ;
//  	}
//  	HAL_Delay (1000);
//  	GC9A01_Clear();
//    GC9A01_DrawImage( 80, 80, 85, 53, img0	);
//  	HAL_Delay (1000);
//
//  	GC9A01_print( 1, 1, RGB565(255, 255, 255) , RGB565(0, 10, 100) , 1, &Font_16x26, 1, "STM32 TFT" );
//
//  	GC9A01_print( 1, 20, RGB565(255, 0, 0) , RGB565(0, 10, 100) , 1, &Font_11x18, 1, "Кла�?�?ный ди�?плей !!!" );
//
//    GC9A01_print( 1, 40, RGB565(0, 255, 0) , RGB565(0, 10, 100) , 1, &Font_7x9, 2, "GC9A01 : 240x240" );
//
//  	HAL_Delay (1000);
//
//  	GC9A01_Clear();
//
//  	GC9A01_DrawBitmap( 50, 50, logo, 128, 27, RGB565(255, 0, 255) );
//
//  	HAL_Delay (1000);
//
//  	GC9A01_Clear();
//
//  	GC9A01_DrawCircle(120-1, 120-1, 10, RGB565(255, 0, 255));
//
//  	HAL_Delay (1000);
//
//  	GC9A01_DrawArc(100, 100, 50, 90, 180, RGB565(255, 255, 0), 5);
//
//  	GC9A01_DrawCircleFilled(50, 290, 20, RGB565(255, 40, 255)) ;
//
//  	GC9A01_DrawLine(1, 319, 239, 319, RGB565(255, 255, 0));
//
//  	GC9A01_DrawLineThick(10, 120, 200, 140, RGB565(255, 255, 0), 5);
//
//  	GC9A01_DrawRectangleFilled(90, 265, 140, 310, RGB565(0, 255, 0)) ;
//
//  	GC9A01_DrawRectangle(160, 265, 220, 309, RGB565(255, 255, 255)) ;
//
//  	GC9A01_DrawTriangle(60, 40, 150, 100, 200, 200, RGB565(100, 255, 150) );
//
//  	GC9A01_DrawFilledTriangle(20, 40, 150, 100, 200, 200, RGB565(100, 255, 150) );
//
//  	GC9A01_DrawFillRoundRect(10, 10, 50, 50, 10, RGB565(100, 255, 150));
//
//  	GC9A01_DrawRoundRect(10, 10, 50, 50, 10, RGB565(100, 255, 150));
//
//  	GC9A01_DrawFillCircleHelper(30, 30, 20 , 1, 0, RGB565(100, 255, 150));
//
//  	GC9A01_DrawCircleHelper(30, 30, 20 , 1, RGB565(100, 255, 150));
//
//  	HAL_Delay (1000);
//
//  	GC9A01_SleepModeEnter();
//  	HAL_Delay (1000);
//
//  	GC9A01_SleepModeExit();
//  	HAL_Delay (1000);
//
//  	GC9A01_DisplayPower( 0 );
//  	HAL_Delay (1000);
//
//  	GC9A01_DisplayPower( 1 );
//  	HAL_Delay (1000);
//
//  	GC9A01_InversionMode(0);
//  	HAL_Delay (1000);
//
//  	GC9A01_InversionMode(1);
//  	HAL_Delay (1000);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {

		RTC_DateTypeDef gDate;
		RTC_TimeTypeDef gTime;

		  HAL_RTC_GetTime(&hrtc, &gTime, RTC_FORMAT_BIN);
		  /* Get the RTC current Date */
		  HAL_RTC_GetDate(&hrtc, &gDate, RTC_FORMAT_BIN);

		    ss=gTime.Seconds;
		    mm=gTime.Minutes;
		    hh=gTime.Hours;

//		for( uint8_t i = 0; i < 60; i++ ){
			DrawClock(hh, mm, ss, 0, 0);
			HAL_Delay(100);
//		}
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
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE2);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_LSI|RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.LSIState = RCC_LSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 25;
  RCC_OscInitStruct.PLL.PLLN = 168;
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

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
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
  sTime.Hours = 0x12;
  sTime.Minutes = 0x15;
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
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

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
  /* DMA2_Stream3_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream3_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream3_IRQn);

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
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, LCD_CS_Pin|LCD_RST_Pin|LCD_DC_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : LCD_CS_Pin LCD_RST_Pin LCD_DC_Pin */
  GPIO_InitStruct.Pin = LCD_CS_Pin|LCD_RST_Pin|LCD_DC_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
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
