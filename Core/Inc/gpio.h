/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    gpio.h
  * @brief   This file contains all the function prototypes for
  *          the gpio.c file
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
#ifndef __GPIO_H__
#define __GPIO_H__

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

void MX_GPIO_Init(void);

/* USER CODE BEGIN Prototypes */

GPIO_PinState getS1Status(void);
GPIO_PinState getS2Status(void);
GPIO_PinState getS3Status(void);
GPIO_PinState getS4Status(void);
GPIO_PinState getS5Status(void);
GPIO_PinState getS6Status(void);
GPIO_PinState getKeyinStatus(void);
uint8_t getS1S6KeyStatus(void);
void setCheckOut(uint8_t sta);
void setStart1Out(uint8_t sta);
void setStart2Out(uint8_t sta);
void setStart3Out(uint8_t sta);
void disenableCS0toCS4(void);
void setDforLed(uint16_t CSPin, uint8_t pinSta);
void ToggleWDI(void);
void Toggle_SYSTEM_OK(void);
void set_SYSTEM_OK(void);
void reset_SYSTEM_OK(void);
/* USER CODE END Prototypes */

#ifdef __cplusplus
}
#endif
#endif /*__ GPIO_H__ */

