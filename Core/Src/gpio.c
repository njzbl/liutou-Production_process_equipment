/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    gpio.c
  * @brief   This file provides code for the configuration
  *          of all used GPIO pins.
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

/* Includes ------------------------------------------------------------------*/
#include "gpio.h"

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/*----------------------------------------------------------------------------*/
/* Configure GPIO                                                             */
/*----------------------------------------------------------------------------*/
/* USER CODE BEGIN 1 */

/* USER CODE END 1 */

/** Configure pins as
        * Analog
        * Input
        * Output
        * EVENT_OUT
        * EXTI
*/
void MX_GPIO_Init(void)
{

  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOF, START3_OUT_Pin|START2_OUT_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, START1_OUT_Pin|CHK_OUT_Pin|D0_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, D7_Pin|D6_Pin|D5_Pin|D4_Pin
                          |D3_Pin|D2_Pin|D1_Pin|CS4_Pin
                          |LE_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, SYSTEM_OK_Pin|WDI_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, CS0_Pin|CS1_Pin|CS2_Pin|CS3_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : PCPin PCPin PCPin */
  GPIO_InitStruct.Pin = S4_IN_Pin|S5_IN_Pin|S6_IN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : PFPin PFPin */
  GPIO_InitStruct.Pin = START3_OUT_Pin|START2_OUT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOF, &GPIO_InitStruct);

  /*Configure GPIO pins : PAPin PAPin PAPin */
  GPIO_InitStruct.Pin = START1_OUT_Pin|CHK_OUT_Pin|D0_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PBPin PBPin PBPin PBPin
                           PBPin PBPin PBPin PBPin
                           PBPin */
  GPIO_InitStruct.Pin = D7_Pin|D6_Pin|D5_Pin|D4_Pin
                          |D3_Pin|D2_Pin|D1_Pin|CS4_Pin
                          |LE_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : PCPin PCPin */
  GPIO_InitStruct.Pin = SYSTEM_OK_Pin|WDI_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : PtPin */
  GPIO_InitStruct.Pin = KEY_IN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(KEY_IN_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : PDPin PDPin PDPin PDPin */
  GPIO_InitStruct.Pin = CS0_Pin|CS1_Pin|CS2_Pin|CS3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pins : PBPin PBPin PBPin */
  GPIO_InitStruct.Pin = S1_IN_Pin|S2_IN_Pin|S3_IN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

}

/* USER CODE BEGIN 2 */
GPIO_PinState getS1Status(void)
{
    return  HAL_GPIO_ReadPin(GPIOB, S1_IN_Pin);
}

GPIO_PinState getS2Status(void)
{
    return  HAL_GPIO_ReadPin(GPIOB, S2_IN_Pin);
}

GPIO_PinState getS3Status(void)
{
    return  HAL_GPIO_ReadPin(GPIOB, S3_IN_Pin);
}

GPIO_PinState getS4Status(void)
{
    return  HAL_GPIO_ReadPin(GPIOC, S4_IN_Pin);
}

GPIO_PinState getS5Status(void)
{
    return  HAL_GPIO_ReadPin(GPIOC, S5_IN_Pin);
}

GPIO_PinState getS6Status(void)
{
    return  HAL_GPIO_ReadPin(GPIOC, S6_IN_Pin);
}

GPIO_PinState getKeyinStatus(void)
{
    return  HAL_GPIO_ReadPin(GPIOC, S6_IN_Pin);
}

uint8_t getS1S6KeyStatus(void)
{
    uint8_t re = 0;
    if(getS1Status() == GPIO_PIN_SET);
        re |= 0x01;
    if(getS2Status() == GPIO_PIN_SET);
        re |= 0x02;
    if(getS3Status() == GPIO_PIN_SET);
        re |= 0x04;
    if(getS4Status() == GPIO_PIN_SET);
        re |= 0x08;
    if(getS5Status() == GPIO_PIN_SET);
        re |= 0x10;
    if(getS6Status() == GPIO_PIN_SET);
        re |= 0x20;
    if(getKeyinStatus() == GPIO_PIN_SET);
        re |= 0x30;
    return re;
}

void setCheckOut(uint8_t sta)
{   
    GPIO_PinState pinSta;
    if(sta == 1)
        pinSta = GPIO_PIN_SET;
    else
        pinSta = GPIO_PIN_RESET;
    HAL_GPIO_WritePin(GPIOA, CHK_OUT_Pin, pinSta);
}

void setStart1Out(uint8_t sta)
{   
    GPIO_PinState pinSta;
    if(sta == 1)
        pinSta = GPIO_PIN_SET;
    else
        pinSta = GPIO_PIN_RESET;
    HAL_GPIO_WritePin(GPIOA, START1_OUT_Pin, pinSta);
}

void setStart2Out(uint8_t sta)
{   
    GPIO_PinState pinSta;
    if(sta == 1)
        pinSta = GPIO_PIN_SET;
    else
        pinSta = GPIO_PIN_RESET;
    HAL_GPIO_WritePin(GPIOF, START2_OUT_Pin, pinSta);
}

void setStart3Out(uint8_t sta)
{   
    GPIO_PinState pinSta;
    if(sta == 1)
        pinSta = GPIO_PIN_SET;
    else
        pinSta = GPIO_PIN_RESET;
    HAL_GPIO_WritePin(GPIOF, START3_OUT_Pin, pinSta);
}

void disenableCS0toCS4(void)
{
    HAL_GPIO_WritePin(GPIOD, CS0_Pin|CS1_Pin|CS2_Pin|CS3_Pin, GPIO_PIN_RESET);    
    HAL_GPIO_WritePin(GPIOB, CS4_Pin|LE_Pin, GPIO_PIN_RESET);
}

void setDforRed(uint16_t CSPin, uint8_t pinSta)
{
    uint16_t GPIOB_Pin2Set = 0;
    uint16_t GPIOB_Pin2Reset = 0;
    uint16_t GPIOA_Pin2Set = 0;
    uint16_t GPIOA_Pin2Reset = 0;

    if(pinSta & 0x80)
        GPIOB_Pin2Set |= D7_Pin;
    else
        GPIOB_Pin2Reset |= D7_Pin;
    if(pinSta & 0x40)
        GPIOB_Pin2Set |= D6_Pin;
    else
        GPIOB_Pin2Reset |= D6_Pin;
    if(pinSta & 0x20)
        GPIOB_Pin2Set |= D5_Pin;
    else
        GPIOB_Pin2Reset |= D5_Pin;
    if(pinSta & 0x10)
        GPIOB_Pin2Set |= D4_Pin;
    else
        GPIOB_Pin2Reset |= D4_Pin;
    if(pinSta & 0x08)
        GPIOB_Pin2Set |= D3_Pin;
    else
        GPIOB_Pin2Reset |= D3_Pin;
    if(pinSta & 0x04)
        GPIOB_Pin2Set |= D2_Pin;
    else
        GPIOB_Pin2Reset |= D2_Pin;
    if(pinSta & 0x02)
        GPIOB_Pin2Set |= D1_Pin;
    else
        GPIOB_Pin2Reset |= D1_Pin;
    if(pinSta & 0x01)
        GPIOA_Pin2Set |= D0_Pin;
    else
        GPIOA_Pin2Reset |= D0_Pin;
    if(GPIOB_Pin2Set != 0)
        HAL_GPIO_WritePin(GPIOB, GPIOB_Pin2Set, GPIO_PIN_SET);
    if(GPIOB_Pin2Reset != 0)
        HAL_GPIO_WritePin(GPIOB, GPIOB_Pin2Reset, GPIO_PIN_RESET);
    if(GPIOA_Pin2Set != 0)
        HAL_GPIO_WritePin(GPIOA, GPIOA_Pin2Set, GPIO_PIN_SET);
    if(GPIOA_Pin2Reset != 0)
        HAL_GPIO_WritePin(GPIOA, GPIOA_Pin2Reset, GPIO_PIN_RESET);
    if(CSPin == CS0_Pin || CSPin == CS1_Pin || CSPin == CS2_Pin || CSPin == CS3_Pin){
        HAL_GPIO_WritePin(GPIOD, CSPin, GPIO_PIN_SET);
        // HAL_Delay(10); 
        HAL_GPIO_WritePin(GPIOD, CSPin, GPIO_PIN_RESET);
    }
    else if(CSPin == CS4_Pin){
        HAL_GPIO_WritePin(GPIOB, CSPin, GPIO_PIN_SET);
        // HAL_Delay(10); 
        HAL_GPIO_WritePin(GPIOB, CSPin, GPIO_PIN_RESET);
    }
}

void setDforGreen(uint16_t CSPin, uint8_t pinSta)   //CS1
{
    uint16_t GPIOB_Pin2Set = 0;
    uint16_t GPIOB_Pin2Reset = 0;
    uint16_t GPIOA_Pin2Set = 0;
    uint16_t GPIOA_Pin2Reset = 0;

    if(pinSta & 0x80)
        GPIOB_Pin2Set |= D7_Pin;
    else
        GPIOB_Pin2Reset |= D7_Pin;
    if(pinSta & 0x40)
        GPIOB_Pin2Set |= D6_Pin;
    else
        GPIOB_Pin2Reset |= D6_Pin;
    if(pinSta & 0x20)
        GPIOB_Pin2Set |= D5_Pin;
    else
        GPIOB_Pin2Reset |= D5_Pin;
    if(pinSta & 0x10)
        GPIOB_Pin2Set |= D4_Pin;
    else
        GPIOB_Pin2Reset |= D4_Pin;
    if(pinSta & 0x08)
        GPIOB_Pin2Set |= D3_Pin;
    else
        GPIOB_Pin2Reset |= D3_Pin;
    if(pinSta & 0x04)
        GPIOB_Pin2Set |= D2_Pin;
    else
        GPIOB_Pin2Reset |= D2_Pin;
    if(pinSta & 0x02)
        GPIOB_Pin2Set |= D1_Pin;
    else
        GPIOB_Pin2Reset |= D1_Pin;
    if(pinSta & 0x01)
        GPIOA_Pin2Set |= D0_Pin;
    else
        GPIOA_Pin2Reset |= D0_Pin;
    if(GPIOB_Pin2Set != 0)
        HAL_GPIO_WritePin(GPIOB, GPIOB_Pin2Set, GPIO_PIN_SET);
    if(GPIOB_Pin2Reset != 0)
        HAL_GPIO_WritePin(GPIOB, GPIOB_Pin2Reset, GPIO_PIN_RESET);
    if(GPIOA_Pin2Set != 0)
        HAL_GPIO_WritePin(GPIOA, GPIOA_Pin2Set, GPIO_PIN_SET);
    if(GPIOA_Pin2Reset != 0)
        HAL_GPIO_WritePin(GPIOA, GPIOA_Pin2Reset, GPIO_PIN_RESET);    
    if(CSPin == CS0_Pin || CSPin == CS1_Pin || CSPin == CS2_Pin || CSPin == CS3_Pin){
        HAL_GPIO_WritePin(GPIOD, CSPin, GPIO_PIN_SET);
        // HAL_Delay(10); 
        HAL_GPIO_WritePin(GPIOD, CSPin, GPIO_PIN_RESET);
    }
    else if(CSPin == CS4_Pin){
        HAL_GPIO_WritePin(GPIOB, CSPin, GPIO_PIN_SET);
        // HAL_Delay(10); 
        HAL_GPIO_WritePin(GPIOB, CSPin, GPIO_PIN_RESET);
    }
}


/* USER CODE END 2 */
