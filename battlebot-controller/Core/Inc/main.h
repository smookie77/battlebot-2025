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

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define LED_Pin GPIO_PIN_13
#define LED_GPIO_Port GPIOC
#define JOY_X_Pin GPIO_PIN_0
#define JOY_X_GPIO_Port GPIOA
#define JOY_Y_Pin GPIO_PIN_1
#define JOY_Y_GPIO_Port GPIOA
#define JOY_BTN_Pin GPIO_PIN_2
#define JOY_BTN_GPIO_Port GPIOA
#define RF24_CSN_Pin GPIO_PIN_0
#define RF24_CSN_GPIO_Port GPIOB
#define RF24_CE_Pin GPIO_PIN_1
#define RF24_CE_GPIO_Port GPIOB
#define BTN_DWN_Pin GPIO_PIN_12
#define BTN_DWN_GPIO_Port GPIOB
#define BTN_L_Pin GPIO_PIN_13
#define BTN_L_GPIO_Port GPIOB
#define BTN_R_Pin GPIO_PIN_14
#define BTN_R_GPIO_Port GPIOB
#define BTN_UP_Pin GPIO_PIN_15
#define BTN_UP_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
