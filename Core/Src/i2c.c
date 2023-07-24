/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    i2c.c
  * @brief   This file provides code for the configuration
  *          of the I2C instances.
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
#include "i2c.h"

/* USER CODE BEGIN 0 */
#define INA219_BUSVOLT_LSB_MW               4     // Bus Voltage LSB value = 4mV
#define INA219_SHUNTVOLT_LSB_UV             10    // Shunt Voltage LSB value = 10uV
#define INA219_CURRENT_LSB_UA               100
#define INA219_POWER_LSB_MW                 2
#define INA219_CALVALUE                     0x0D90

 
INA219_DATA ina219_data;
/* USER CODE END 0 */

I2C_HandleTypeDef hi2c1;

/* I2C1 init function */
void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.Timing = 0x10707DBC;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c1, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

void HAL_I2C_MspInit(I2C_HandleTypeDef* i2cHandle)
{

  GPIO_InitTypeDef GPIO_InitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};
  if(i2cHandle->Instance==I2C1)
  {
  /* USER CODE BEGIN I2C1_MspInit 0 */

  /* USER CODE END I2C1_MspInit 0 */

  /** Initializes the peripherals clocks
  */
    PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_I2C1;
    PeriphClkInit.I2c1ClockSelection = RCC_I2C1CLKSOURCE_PCLK1;
    if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
    {
      Error_Handler();
    }

    __HAL_RCC_GPIOB_CLK_ENABLE();
    /**I2C1 GPIO Configuration
    PB6     ------> I2C1_SCL
    PB7     ------> I2C1_SDA
    */
    GPIO_InitStruct.Pin = GPIO_PIN_6|GPIO_PIN_7;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_OD;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.Alternate = GPIO_AF6_I2C1;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    /* I2C1 clock enable */
    __HAL_RCC_I2C1_CLK_ENABLE();
  /* USER CODE BEGIN I2C1_MspInit 1 */

  /* USER CODE END I2C1_MspInit 1 */
  }
}

void HAL_I2C_MspDeInit(I2C_HandleTypeDef* i2cHandle)
{

  if(i2cHandle->Instance==I2C1)
  {
  /* USER CODE BEGIN I2C1_MspDeInit 0 */

  /* USER CODE END I2C1_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_I2C1_CLK_DISABLE();

    /**I2C1 GPIO Configuration
    PB6     ------> I2C1_SCL
    PB7     ------> I2C1_SDA
    */
    HAL_GPIO_DeInit(GPIOB, GPIO_PIN_6);

    HAL_GPIO_DeInit(GPIOB, GPIO_PIN_7);

  /* USER CODE BEGIN I2C1_MspDeInit 1 */

  /* USER CODE END I2C1_MspDeInit 1 */
  }
}

/* USER CODE BEGIN 1 */

s32 ina219_GetCurrent_uA(uint16_t DevAddress)
{
    s32 current;
    unsigned int regval;
    unsigned char val[2];
    
    // Sometimes a sharp load will reset the INA219, which will
    // reset the cal register, meaning CURRENT and POWER will
    // not be available ... avoid this by always setting a cal
    // value even if it's an unfortunate extra step
    regval = INA219_CALVALUE;
    val[0] = (unsigned char)(regval >> 8);
    val[1] = (unsigned char)(regval & 0xFF);
    HAL_I2C_Mem_Write(&hi2c1, DevAddress, INA219_REG_CALIBRATION,
                                      I2C_MEMADD_SIZE_16BIT, val, 2, 1000);
    HAL_Delay(1);   //1ms
    
    val[0] = 0;
    val[1] = 0;
    // Now we can safely read the CURRENT register!
    HAL_I2C_Mem_Read(&hi2c1, DevAddress, INA219_REG_CURRENT,
                                I2C_MEMADD_SIZE_16BIT, val, 2, 1000);
                                
    regval = ((unsigned int)(val[0]) << 8) + val[1];
    current = (s32)regval * INA219_CURRENT_LSB_UA;
  
    return (current);
}

//stm32为小端模式:低字节存储到低地址
// INA219 Set Calibration 16V/16A(Max) 0.02|?
void ina219_SetCalibration_16V_16A(void)
{
  u16 configValue;
  unsigned char val[2];
  uint16_t	DevAddress;
  
  // By default we use a pretty huge range for the input voltage,
  // which probably isn't the most appropriate choice for system
  // that don't use a lot of power.  But all of the calculations
  // are shown below if you want to change the settings.  You will
  // also need to change any relevant register settings, such as
  // setting the VBUS_MAX to 16V instead of 32V, etc.
  
  // VBUS_MAX     = 16V   (Assumes 16V, can also be set to 32V)
  // VSHUNT_MAX   = 0.32  (Assumes Gain 8, 320mV, can also be 0.16, 0.08, 0.04)
  // RSHUNT       = 0.02   (Resistor value in ohms)
  
  // 1. Determine max possible current
  // MaxPossible_I = VSHUNT_MAX / RSHUNT
  // MaxPossible_I = 16A
  
  // 2. Determine max expected current
  // MaxExpected_I = 16A
  
  // 3. Calculate possible range of LSBs (Min = 15-bit, Max = 12-bit)
  // MinimumLSB = MaxExpected_I/32767
  // MinimumLSB = 0.00048            (0.48mA per bit)
  // MaximumLSB = MaxExpected_I/4096
  // MaximumLSB = 0,00390            (3.9mA per bit)
  
  // 4. Choose an LSB between the min and max values
  //    (Preferrably a roundish number close to MinLSB)
  // CurrentLSB = 0.00050            (500uA per bit)
  
  // 5. Compute the calibration register
  // Cal = trunc (0.04096 / (Current_LSB * RSHUNT))
  // Cal = 4096 (0x1000)
  
  // ina219_calValue = 0x0D90;  //0x1000;     ZBL
  
  // 6. Calculate the power LSB
  // PowerLSB = 20 * CurrentLSB
  // PowerLSB = 0.01 (10mW per bit)
  
  // 7. Compute the maximum current and shunt voltage values before overflow
  //
  // Max_Current = Current_LSB * 32767
  // Max_Current = 16.3835A before overflow
  //
  // If Max_Current > Max_Possible_I then
  //    Max_Current_Before_Overflow = MaxPossible_I
  // Else
  //    Max_Current_Before_Overflow = Max_Current
  // End If
  //
  // Max_ShuntVoltage = Max_Current_Before_Overflow * RSHUNT
  // Max_ShuntVoltage = 0.32V
  //
  // If Max_ShuntVoltage >= VSHUNT_MAX
  //    Max_ShuntVoltage_Before_Overflow = VSHUNT_MAX
  // Else
  //    Max_ShuntVoltage_Before_Overflow = Max_ShuntVoltage
  // End If
  
  // 8. Compute the Maximum Power
  // MaximumPower = Max_Current_Before_Overflow * VBUS_MAX
  // MaximumPower = 1.6 * 16V
  // MaximumPower = 256W
  
  // Set multipliers to convert raw current/power values
  // ina219_current_LSB_uA = 100;     // Current LSB = 500uA per bit                            //ZBL
  // ina219_power_LSB_mW = 2;        // Power LSB = 10mW per bit = 20 * Current LSB             //ZBL
  
  // Set Calibration register to 'Cal' calculated above
  for(int ina219_chn = 0; ina219_chn < 4; ina219_chn++) {
        if(ina219_chn == 0)
				  DevAddress = INA219_I2C_ADDRESS_CONF_0;
				else if(ina219_chn == 1)
					DevAddress = INA219_I2C_ADDRESS_CONF_1;
				else if(ina219_chn == 2)
					DevAddress = INA219_I2C_ADDRESS_CONF_4;
        else
          DevAddress = INA219_I2C_ADDRESS_CONF_5;

        configValue = INA219_CALVALUE; 
        val[0] = (unsigned char)(configValue >> 8);
        val[1] = (unsigned char)(configValue & 0xFF);
        HAL_I2C_Mem_Write(&hi2c1, DevAddress, INA219_REG_CALIBRATION,
                                            I2C_MEMADD_SIZE_16BIT, val, 2, 1000);
        HAL_Delay(1);  
        // Set Config register to take into account the settings above
        configValue = ( INA219_CFG_BVOLT_RANGE_32V | INA219_CFG_SVOLT_RANGE_320MV | INA219_CFG_BADCRES_12BIT_16S_8MS | INA219_CFG_SADCRES_12BIT_16S_8MS | INA219_CFG_MODE_SANDBVOLT_CONTINUOUS );
        
        val[0] = (unsigned char)(configValue >> 8);
        val[1] = (unsigned char)(configValue & 0xFF);
        HAL_I2C_Mem_Write(&hi2c1, DevAddress, INA219_REG_CONFIG,
                                            I2C_MEMADD_SIZE_16BIT, val, 2, 1000);
  }
}

void ina219_configureRegisters(void)
{
  HAL_Delay(15);  
  ina219_SetCalibration_16V_16A();
}

/* USER CODE END 1 */
