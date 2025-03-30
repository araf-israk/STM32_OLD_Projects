/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f1xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define RFID_IRQ_Pin GPIO_PIN_12
#define RFID_IRQ_GPIO_Port GPIOB
#define RFID_SS_Pin GPIO_PIN_8
#define RFID_SS_GPIO_Port GPIOD
#define RFID_Reset_Pin GPIO_PIN_9
#define RFID_Reset_GPIO_Port GPIOD
#define RFID_BSS_Pin GPIO_PIN_10
#define RFID_BSS_GPIO_Port GPIOD
#define IN1_Pin GPIO_PIN_10
#define IN1_GPIO_Port GPIOC
#define IN2_Pin GPIO_PIN_11
#define IN2_GPIO_Port GPIOC
#define UART_STATUS_Pin GPIO_PIN_3
#define UART_STATUS_GPIO_Port GPIOD
#define INTER_STATUS_Pin GPIO_PIN_4
#define INTER_STATUS_GPIO_Port GPIOD
#define IN3_Pin GPIO_PIN_4
#define IN3_GPIO_Port GPIOB
#define IN4_Pin GPIO_PIN_5
#define IN4_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
