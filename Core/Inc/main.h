/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32g0xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <string.h>
#include "sx1278.h"
#include "ssd1306draw.h"
#include "ssd_lora_config_menu.h"
#include "gtu7_gps.h"
#include "common.h"
/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */
typedef enum
{
  IDLE = 0x00U,
  CONFIG = 0x01U,
  TRANSMITTING = 0x02U,
  REQUEST4POSITION = 0x03U,
  RECEIVING_POSITION = 0x04U
} Program_Status;

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

#define DEBOUNCING_TIME                 250
#define TIME_BETWEEN_TRANSMISSIONS      2000
#define TIME_BETWEEN_MSGS               300
// #define __MP_DEBUG__

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define BTN2_Pin GPIO_PIN_7
#define BTN2_GPIO_Port GPIOB
#define BTN2_EXTI_IRQn EXTI4_15_IRQn
#define DIO0_Pin GPIO_PIN_0
#define DIO0_GPIO_Port GPIOA
#define DIO0_EXTI_IRQn EXTI0_1_IRQn
#define BTN1_Pin GPIO_PIN_4
#define BTN1_GPIO_Port GPIOA
#define BTN1_EXTI_IRQn EXTI4_15_IRQn
#define RST_Pin GPIO_PIN_5
#define RST_GPIO_Port GPIOA
#define NSS_Pin GPIO_PIN_8
#define NSS_GPIO_Port GPIOA

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
