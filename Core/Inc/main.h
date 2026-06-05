/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
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
#include "stm32f0xx_hal.h"

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
#define MOTION_INT_Pin GPIO_PIN_0
#define MOTION_INT_GPIO_Port GPIOA
#define MOTION_INT_EXTI_IRQn EXTI0_1_IRQn
#define TDS_MCU_Pin GPIO_PIN_2
#define TDS_MCU_GPIO_Port GPIOA
#define NTC_TEMP_Pin GPIO_PIN_3
#define NTC_TEMP_GPIO_Port GPIOA
#define HX711_DOUT_Pin GPIO_PIN_4
#define HX711_DOUT_GPIO_Port GPIOA
#define HX711_SCK_Pin GPIO_PIN_5
#define HX711_SCK_GPIO_Port GPIOA
#define TDS_DRIVE_Pin GPIO_PIN_6
#define TDS_DRIVE_GPIO_Port GPIOA
#define BAT__Pin GPIO_PIN_7
#define BAT__GPIO_Port GPIOA
#define CHRG_STAT_Pin GPIO_PIN_0
#define CHRG_STAT_GPIO_Port GPIOB
#define STDBY_STAT_Pin GPIO_PIN_1
#define STDBY_STAT_GPIO_Port GPIOB
#define RGB_D_Pin GPIO_PIN_8
#define RGB_D_GPIO_Port GPIOA
#define RTC_INT_Pin GPIO_PIN_11
#define RTC_INT_GPIO_Port GPIOA
#define RTC_INT_EXTI_IRQn EXTI4_15_IRQn
#define BLE_STATE_Pin GPIO_PIN_15
#define BLE_STATE_GPIO_Port GPIOA
#define BLE_STATE_EXTI_IRQn EXTI4_15_IRQn
#define BUZZER_Pin GPIO_PIN_5
#define BUZZER_GPIO_Port GPIOB
#define BUTTON_Pin GPIO_PIN_3
#define BUTTON_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
