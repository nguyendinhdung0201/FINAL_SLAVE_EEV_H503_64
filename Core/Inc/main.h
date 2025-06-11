/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
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
#include "stm32h5xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */
extern volatile uint16_t step_position;
extern volatile float percent_step;
extern volatile uint8_t state_motor_step;
extern volatile uint32_t last_time_step;

extern volatile uint16_t adc_buffer[5];


extern void printLOGDATA(const char *format, ...);
void I2C1_Reinit(void);
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
#define hoive_INP0_Pin GPIO_PIN_0
#define hoive_INP0_GPIO_Port GPIOA
#define dauday_INP1_Pin GPIO_PIN_1
#define dauday_INP1_GPIO_Port GPIOA
#define low_pressure_INP14_Pin GPIO_PIN_2
#define low_pressure_INP14_GPIO_Port GPIOA
#define high_pressure_INP15_Pin GPIO_PIN_3
#define high_pressure_INP15_GPIO_Port GPIOA
#define EWDG_Pin GPIO_PIN_4
#define EWDG_GPIO_Port GPIOA
#define RUN_Defrost_Pin GPIO_PIN_6
#define RUN_Defrost_GPIO_Port GPIOA
#define RUN_Pin GPIO_PIN_7
#define RUN_GPIO_Port GPIOA
#define STEPPER_1_Pin GPIO_PIN_12
#define STEPPER_1_GPIO_Port GPIOB
#define STEPPER_2_Pin GPIO_PIN_13
#define STEPPER_2_GPIO_Port GPIOB
#define STEPPER_3_Pin GPIO_PIN_14
#define STEPPER_3_GPIO_Port GPIOB
#define STEPPER_4_Pin GPIO_PIN_15
#define STEPPER_4_GPIO_Port GPIOB
#define RELAY_Pin GPIO_PIN_8
#define RELAY_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
