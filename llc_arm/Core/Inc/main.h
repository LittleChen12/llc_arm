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
#include "stm32h7xx_hal.h"

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
#define KEY2_Pin GPIO_PIN_13
#define KEY2_GPIO_Port GPIOC
#define KEY1_Pin GPIO_PIN_0
#define KEY1_GPIO_Port GPIOA
#define ADC0_Pin GPIO_PIN_0
#define ADC0_GPIO_Port GPIOB
#define ADC1_Pin GPIO_PIN_1
#define ADC1_GPIO_Port GPIOB
#define ADC2_Pin GPIO_PIN_2
#define ADC2_GPIO_Port GPIOB
#define LED_Pin GPIO_PIN_7
#define LED_GPIO_Port GPIOE
#define Relay_Switch_Pin GPIO_PIN_8
#define Relay_Switch_GPIO_Port GPIOE
#define Limit_Switch_1_Pin GPIO_PIN_9
#define Limit_Switch_1_GPIO_Port GPIOE
#define Limit_Switch_2_Pin GPIO_PIN_10
#define Limit_Switch_2_GPIO_Port GPIOE
#define Limit_Switch_3_Pin GPIO_PIN_11
#define Limit_Switch_3_GPIO_Port GPIOE
#define Limit_Switch_4_Pin GPIO_PIN_12
#define Limit_Switch_4_GPIO_Port GPIOE
#define Limit_Switch_5_Pin GPIO_PIN_13
#define Limit_Switch_5_GPIO_Port GPIOE
#define Limit_Switch_6_Pin GPIO_PIN_14
#define Limit_Switch_6_GPIO_Port GPIOE
#define ADC10_Pin GPIO_PIN_10
#define ADC10_GPIO_Port GPIOB
#define ADC11_Pin GPIO_PIN_14
#define ADC11_GPIO_Port GPIOB
#define ADC12_Pin GPIO_PIN_15
#define ADC12_GPIO_Port GPIOB
#define ADC13_Pin GPIO_PIN_8
#define ADC13_GPIO_Port GPIOD
#define ADC3_Pin GPIO_PIN_3
#define ADC3_GPIO_Port GPIOB
#define ADC4_Pin GPIO_PIN_4
#define ADC4_GPIO_Port GPIOB
#define ADC5_Pin GPIO_PIN_5
#define ADC5_GPIO_Port GPIOB
#define ADC6_Pin GPIO_PIN_6
#define ADC6_GPIO_Port GPIOB
#define ADC7_Pin GPIO_PIN_7
#define ADC7_GPIO_Port GPIOB
#define ADC8_Pin GPIO_PIN_8
#define ADC8_GPIO_Port GPIOB
#define ADC9_Pin GPIO_PIN_9
#define ADC9_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
