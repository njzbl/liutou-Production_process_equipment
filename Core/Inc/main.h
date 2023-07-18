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
#define S4_IN_Pin GPIO_PIN_13
#define S4_IN_GPIO_Port GPIOC
#define S5_IN_Pin GPIO_PIN_14
#define S5_IN_GPIO_Port GPIOC
#define S6_IN_Pin GPIO_PIN_15
#define S6_IN_GPIO_Port GPIOC
#define START3_OUT_Pin GPIO_PIN_0
#define START3_OUT_GPIO_Port GPIOF
#define START2_OUT_Pin GPIO_PIN_1
#define START2_OUT_GPIO_Port GPIOF
#define START1_OUT_Pin GPIO_PIN_0
#define START1_OUT_GPIO_Port GPIOA
#define CHK_OUT_Pin GPIO_PIN_1
#define CHK_OUT_GPIO_Port GPIOA
#define D7_Pin GPIO_PIN_2
#define D7_GPIO_Port GPIOB
#define D6_Pin GPIO_PIN_10
#define D6_GPIO_Port GPIOB
#define D5_Pin GPIO_PIN_11
#define D5_GPIO_Port GPIOB
#define D4_Pin GPIO_PIN_12
#define D4_GPIO_Port GPIOB
#define D3_Pin GPIO_PIN_13
#define D3_GPIO_Port GPIOB
#define D2_Pin GPIO_PIN_14
#define D2_GPIO_Port GPIOB
#define D1_Pin GPIO_PIN_15
#define D1_GPIO_Port GPIOB
#define D0_Pin GPIO_PIN_8
#define D0_GPIO_Port GPIOA
#define SYSTEM_OK_Pin GPIO_PIN_6
#define SYSTEM_OK_GPIO_Port GPIOC
#define WDI_Pin GPIO_PIN_7
#define WDI_GPIO_Port GPIOC
#define KEY_IN_Pin GPIO_PIN_15
#define KEY_IN_GPIO_Port GPIOA
#define CS0_Pin GPIO_PIN_0
#define CS0_GPIO_Port GPIOD
#define CS1_Pin GPIO_PIN_1
#define CS1_GPIO_Port GPIOD
#define CS2_Pin GPIO_PIN_2
#define CS2_GPIO_Port GPIOD
#define CS3_Pin GPIO_PIN_3
#define CS3_GPIO_Port GPIOD
#define CS4_Pin GPIO_PIN_3
#define CS4_GPIO_Port GPIOB
#define LE_Pin GPIO_PIN_4
#define LE_GPIO_Port GPIOB
#define S1_IN_Pin GPIO_PIN_5
#define S1_IN_GPIO_Port GPIOB
#define S2_IN_Pin GPIO_PIN_8
#define S2_IN_GPIO_Port GPIOB
#define S3_IN_Pin GPIO_PIN_9
#define S3_IN_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
