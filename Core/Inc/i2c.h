/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    i2c.h
  * @brief   This file contains all the function prototypes for
  *          the i2c.c file
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
#ifndef __I2C_H__
#define __I2C_H__

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

extern I2C_HandleTypeDef hi2c1;

/* USER CODE BEGIN Private defines */

/*----------------------------------------------------------------------------*/
// I2C Address Options
#define INA219_I2C_ADDRESS_CONF_0               (u8)(0x40 << 1)     // A0 = GND, A1 = GND
#define INA219_I2C_ADDRESS_CONF_1               (u8)(0x41 << 1)     // A0 = VS+, A1 = GND
#define INA219_I2C_ADDRESS_CONF_2               (u8)(0x42 << 1)     // A0 = SDA, A1 = GND
#define INA219_I2C_ADDRESS_CONF_3               (u8)(0x43 << 1)     // A0 = SCL, A1 = GND
#define INA219_I2C_ADDRESS_CONF_4               (u8)(0x44 << 1)     // A0 = GND, A1 = VS+
#define INA219_I2C_ADDRESS_CONF_5               (u8)(0x45 << 1)     // A0 = VS+, A1 = VS+
#define INA219_I2C_ADDRESS_CONF_6               (u8)(0x46 << 1)     // A0 = SDA, A1 = VS+
#define INA219_I2C_ADDRESS_CONF_7               (u8)(0x47 << 1)     // A0 = SCL, A1 = VS+
#define INA219_I2C_ADDRESS_CONF_8               (u8)(0x48 << 1)     // A0 = GND, A1 = SDA
#define INA219_I2C_ADDRESS_CONF_9               (u8)(0x49 << 1)     // A0 = VS+, A1 = SDA
#define INA219_I2C_ADDRESS_CONF_A               (u8)(0x4A << 1)     // A0 = SDA, A1 = SDA
#define INA219_I2C_ADDRESS_CONF_B               (u8)(0x4B << 1)     // A0 = SCL, A1 = SDA
#define INA219_I2C_ADDRESS_CONF_C               (u8)(0x4C << 1)     // A0 = GND, A1 = SCL
#define INA219_I2C_ADDRESS_CONF_D               (u8)(0x4D << 1)     // A0 = VS+, A1 = SCL
#define INA219_I2C_ADDRESS_CONF_E               (u8)(0x4E << 1)     // A0 = SDA, A1 = SCL
#define INA219_I2C_ADDRESS_CONF_F               (u8)(0x4F << 1)     // A0 = SCL, A1 = SCL
//#define INA219_I2C_ADDRESS                      INA219_I2C_ADDRESS_CONF_1
 
 
/*----------------------------------------------------------------------------*/
// Register Addresses
#define INA219_REG_CONFIG                       (u8)(0x00)      // CONFIG REGISTER (R/W)
#define INA219_REG_SHUNTVOLTAGE                 (u8)(0x01)      // SHUNT VOLTAGE REGISTER (R)
#define INA219_REG_BUSVOLTAGE                   (u8)(0x02)      // BUS VOLTAGE REGISTER (R)
#define INA219_REG_POWER                        (u8)(0x03)      // POWER REGISTER (R)
#define INA219_REG_CURRENT                      (u8)(0x04)      // CURRENT REGISTER (R)
#define INA219_REG_CALIBRATION                  (u8)(0x05)      // CALIBRATION REGISTER (R/W)
 
 
/*----------------------------------------------------------------------------*/
// Macros for assigning config bits
#define INA219_CFGB_RESET(x)                    (u16)((x & 0x01) << 15)     // Reset Bit
#define INA219_CFGB_BUSV_RANGE(x)               (u16)((x & 0x01) << 13)     // Bus Voltage Range
#define INA219_CFGB_PGA_RANGE(x)                (u16)((x & 0x03) << 11)     // Shunt Voltage Range
#define INA219_CFGB_BADC_RES_AVG(x)             (u16)((x & 0x0F) << 7)      // Bus ADC Resolution/Averaging
#define INA219_CFGB_SADC_RES_AVG(x)             (u16)((x & 0x0F) << 3)      // Shunt ADC Resolution/Averaging
#define INA219_CFGB_MODE(x)                     (u16) (x & 0x07)            // Operating Mode
 
 
/*----------------------------------------------------------------------------*/
// Configuration Register
#define INA219_CFG_RESET                        INA219_CFGB_RESET(1)            // Reset Bit
 
#define INA219_CFG_BVOLT_RANGE_MASK             INA219_CFGB_BUSV_RANGE(1)       // Bus Voltage Range Mask
#define INA219_CFG_BVOLT_RANGE_16V              INA219_CFGB_BUSV_RANGE(0)       // 0-16V Range
#define INA219_CFG_BVOLT_RANGE_32V              INA219_CFGB_BUSV_RANGE(1)       // 0-32V Range (default)
 
#define INA219_CFG_SVOLT_RANGE_MASK             INA219_CFGB_PGA_RANGE(3)        // Shunt Voltage Range Mask
#define INA219_CFG_SVOLT_RANGE_40MV             INA219_CFGB_PGA_RANGE(0)        // Gain 1, 40mV Range
#define INA219_CFG_SVOLT_RANGE_80MV             INA219_CFGB_PGA_RANGE(1)        // Gain 2, 80mV Range
#define INA219_CFG_SVOLT_RANGE_160MV            INA219_CFGB_PGA_RANGE(2)        // Gain 4, 160mV Range
#define INA219_CFG_SVOLT_RANGE_320MV            INA219_CFGB_PGA_RANGE(3)        // Gain 8, 320mV Range (default)
 
#define INA219_CFG_BADCRES_MASK                 INA219_CFGB_BADC_RES_AVG(15)    // Bus ADC Resolution and Averaging Mask
#define INA219_CFG_BADCRES_9BIT_1S_84US         INA219_CFGB_BADC_RES_AVG(0)     // 1 x 9-bit Bus sample
#define INA219_CFG_BADCRES_10BIT_1S_148US       INA219_CFGB_BADC_RES_AVG(1)     // 1 x 10-bit Bus sample
#define INA219_CFG_BADCRES_11BIT_1S_276US       INA219_CFGB_BADC_RES_AVG(2)     // 1 x 11-bit Bus sample
#define INA219_CFG_BADCRES_12BIT_1S_532US       INA219_CFGB_BADC_RES_AVG(3)     // 1 x 12-bit Bus sample (default)
#define INA219_CFG_BADCRES_12BIT_2S_1MS         INA219_CFGB_BADC_RES_AVG(9)     // 2 x 12-bit Bus samples averaged together
#define INA219_CFG_BADCRES_12BIT_4S_2MS         INA219_CFGB_BADC_RES_AVG(10)    // 4 x 12-bit Bus samples averaged together
#define INA219_CFG_BADCRES_12BIT_8S_4MS         INA219_CFGB_BADC_RES_AVG(11)    // 8 x 12-bit Bus samples averaged together
#define INA219_CFG_BADCRES_12BIT_16S_8MS        INA219_CFGB_BADC_RES_AVG(12)    // 16 x 12-bit Bus samples averaged together
#define INA219_CFG_BADCRES_12BIT_32S_17MS       INA219_CFGB_BADC_RES_AVG(13)    // 32 x 12-bit Bus samples averaged together
#define INA219_CFG_BADCRES_12BIT_64S_34MS       INA219_CFGB_BADC_RES_AVG(14)    // 64 x 12-bit Bus samples averaged together
#define INA219_CFG_BADCRES_12BIT_128S_68MS      INA219_CFGB_BADC_RES_AVG(15)    // 128 x 12-bit Bus samples averaged together
 
#define INA219_CFG_SADCRES_MASK                 INA219_CFGB_SADC_RES_AVG(15)    // Shunt ADC Resolution and Averaging Mask
#define INA219_CFG_SADCRES_9BIT_1S_84US         INA219_CFGB_SADC_RES_AVG(0)     // 1 x 9-bit Shunt sample
#define INA219_CFG_SADCRES_10BIT_1S_148US       INA219_CFGB_SADC_RES_AVG(1)     // 1 x 10-bit Shunt sample
#define INA219_CFG_SADCRES_11BIT_1S_276US       INA219_CFGB_SADC_RES_AVG(2)     // 1 x 11-bit Shunt sample
#define INA219_CFG_SADCRES_12BIT_1S_532US       INA219_CFGB_SADC_RES_AVG(3)     // 1 x 12-bit Shunt sample (default)
#define INA219_CFG_SADCRES_12BIT_2S_1MS         INA219_CFGB_SADC_RES_AVG(9)     // 2 x 12-bit Shunt samples averaged together
#define INA219_CFG_SADCRES_12BIT_4S_2MS         INA219_CFGB_SADC_RES_AVG(10)    // 4 x 12-bit Shunt samples averaged together
#define INA219_CFG_SADCRES_12BIT_8S_4MS         INA219_CFGB_SADC_RES_AVG(11)    // 8 x 12-bit Shunt samples averaged together
#define INA219_CFG_SADCRES_12BIT_16S_8MS        INA219_CFGB_SADC_RES_AVG(12)    // 16 x 12-bit Shunt samples averaged together
#define INA219_CFG_SADCRES_12BIT_32S_17MS       INA219_CFGB_SADC_RES_AVG(13)    // 32 x 12-bit Shunt samples averaged together
#define INA219_CFG_SADCRES_12BIT_64S_34MS       INA219_CFGB_SADC_RES_AVG(14)    // 64 x 12-bit Shunt samples averaged together
#define INA219_CFG_SADCRES_12BIT_128S_68MS      INA219_CFGB_SADC_RES_AVG(15)    // 128 x 12-bit Shunt samples averaged together
 
#define INA219_CFG_MODE_MASK                    INA219_CFGB_MODE(7)             // Operating Mode Mask
#define INA219_CFG_MODE_POWERDOWN               INA219_CFGB_MODE(0)             // Power-Down
#define INA219_CFG_MODE_SVOLT_TRIGGERED         INA219_CFGB_MODE(1)             // Shunt Voltage, Triggered
#define INA219_CFG_MODE_BVOLT_TRIGGERED         INA219_CFGB_MODE(2)             // Bus Voltage, Triggered
#define INA219_CFG_MODE_SANDBVOLT_TRIGGERED     INA219_CFGB_MODE(3)             // Shunt and Bus, Triggered
#define INA219_CFG_MODE_ADCOFF                  INA219_CFGB_MODE(4)             // ADC Off (disabled)
#define INA219_CFG_MODE_SVOLT_CONTINUOUS        INA219_CFGB_MODE(5)             // Shunt Voltage, Continuous
#define INA219_CFG_MODE_BVOLT_CONTINUOUS        INA219_CFGB_MODE(6)             // Bus Voltage, Continuous
#define INA219_CFG_MODE_SANDBVOLT_CONTINUOUS    INA219_CFGB_MODE(7)             // Shunt and Bus, Continuous (default)
 
 
/*----------------------------------------------------------------------------*/
// Bus Voltage Register
#define INA219_BVOLT_CNVR                       (u16)(0x0002)       // Conversion Ready
#define INA219_BVOLT_OVF                        (u16)(0x0001)       // Math Overflow Flag
#define u8 	uint8_t
#define u16 uint16_t
#define u32 uint32_t
#define s16 short
#define s32 int
typedef struct
{
  signed short voltage_ina219;
  signed long shunt_ina219;
  signed long current_ina219;
  signed long power_ina219;
}INA219_DATA;
/* USER CODE END Private defines */

void MX_I2C1_Init(void);

/* USER CODE BEGIN Prototypes */

/* USER CODE END Prototypes */

#ifdef __cplusplus
}
#endif

#endif /* __I2C_H__ */

