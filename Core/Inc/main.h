/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.h
 * @brief          : Header for main.c file.
 *                   This file contains the common defines of the application.
 ******************************************************************************
 * @attention
 *
 * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
 * All rights reserved.</center></h2>
 *
 * This software component is licensed by ST under Ultimate Liberty license
 * SLA0044, the "License"; You may not use this file except in compliance with
 * the License. You may obtain a copy of the License at:
 *                             www.st.com/SLA0044
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
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */
typedef struct {
  void *key;
  void *ntc;
  void *pwm;
  void *display;
  void *led;
} message_t;

typedef struct {
  uint8_t a;
  uint8_t b;
  uint8_t c;
} led_t;

typedef struct {
  uint8_t a;
  uint8_t b;
  uint8_t c;
  uint8_t d;
  uint8_t e;
  uint8_t f;
  uint8_t g;
  uint8_t h;
} display_t;

typedef struct {
  uint8_t a;
  uint8_t b;
  uint8_t c;
} key_t;

typedef float NTC_t;

typedef uint8_t pwm_t;

typedef enum{
  relax_mode,
  sleep_mode,
  nature_mode,
  common_mode,
} Mode_t;

typedef struct {
  key_t key;
  NTC_t ntc;
  pwm_t pwm;
  display_t display;
  led_t led;
  Mode_t mode;
} data_t;

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
