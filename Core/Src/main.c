/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
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
#include "main.h"
#include "adc.h"
#include "dma.h"
#include "i2c.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

void sendBufUart1(uint8_t txLen);
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
#define VALID           1
#define INVALID         0
#define MACHINA         03
#define MOTOR1          VALID
#define MOTOR2          INVALID
#define MOTOR3          INVALID
#define MOTOR4          INVALID
#define FAN1            VALID
#define FAN2            INVALID
#define FAN3            INVALID

#define MOTOR_CURRENT_F_MIN     3000
#define MOTOR_CURRENT_B_MIN    -3000
#define FAN_CURRENT_MIN         2000

#define OK      1
#define ERR     0
#define ADC_CONVERTED_DATA_BUFFER_SIZE      3   //目前3路AC_FAN_ADC
#define VAR_CONVERTED_DATA_INIT_VALUE       0

#define UART1_RXBUF_MAX                     20
#define UART1_TXBUF_MAX                     60
#define UART2_RXBUF_MAX                     10
#define UART2_TXBUF_MAX                     60
#define UART2_TX_CMD_LEN                    34


#define OPENING			1
#define CLOSEING		0
#define OPRN_CLOSE		2
#define MACHINE_OK		0
#define MACHINE_NG		1

__IO   uint16_t mADCxConvertedData[ADC_CONVERTED_DATA_BUFFER_SIZE]; //
__IO   uint8_t mDmaTransferStatus = 2;//
__IO   uint8_t mTimerStatus = 2;//

__IO ITStatus mUart1Ready = RESET;
__IO ITStatus mUart2Ready = RESET;
__IO   uint8_t mRx1Num = 0;
__IO   uint8_t mRx2Num = 0;

signed long mMotorCurrent[4][11] = {0};
int mFanCurrent[3][41] = {0};
uint8_t mMotorSta[4] = {MACHINE_OK};
uint8_t mFanSta[3] = {MACHINE_OK};
uint8_t mS1toS6Sta1 = 0;
uint8_t mS1toS6Sta2 = 0;
uint8_t mS1toS6Sta = 0;
uint8_t mInputStatus = 0;
uint8_t mLampStatus[3];

uint16_t mVolAcFan[ADC_CONVERTED_DATA_BUFFER_SIZE]; // 交流风机实际的采样电压
uint8_t m573Status[5] = {0};

uint8_t mCheckStatus = 3;               //3：开机后，一直没有开启检测流程，等待KEY_IN和uart2 信号  2：开机后，已经等到KEY_IN，马上发送启动命令给PC  1：正在检测中，不响应KEY_IN信号   0：检测完成，等待KEY_IN和uart2 信号


uint8_t mRxBufUart1[UART1_RXBUF_MAX] = {0};
uint8_t mTxBufUart1[UART1_TXBUF_MAX] = {0};
uint8_t mRxBufUart2[UART2_RXBUF_MAX] = {0};
uint8_t mTxBufUart2[UART2_TXBUF_MAX] = {0};
uint8_t mCmdBufU1[UART1_RXBUF_MAX] = {0};
uint8_t mCmdBufU2[UART2_RXBUF_MAX] = {0};
uint8_t mCmdferCountUart1 = 0;
uint8_t mCmdferCountUart2 = 0;



uint16_t crc(uint8_t *s,uint16_t len)
{     uint16_t acc = 0,i;
      while (len--)
      {     acc = acc ^ (*s++ << 8);
            for (i = 0; i++ < 8;)
            {     if (acc & 0x8000)
                          acc = (acc << 1) ^ 0x8005;
                  else
                          acc <<= 1;
            }
       }
      return  acc; 
}

//命令长度采用定长命令 0x1b 0x10 CMD D0 D1 D2 D3 D4 CRC1 CRC2 10个字 
uint8_t CMD_Analysis(uint8_t* CmdBuf, uint8_t bufSize)
{
    uint16_t crc16Val;
    if(bufSize < 10)
        return ERR;
    if(CmdBuf[0] != 0x1b || CmdBuf[1] != 0x10) {
        HAL_UART_Receive_IT(&huart2, (uint8_t *)mRxBufUart2, UART2_RXBUF_MAX);
        return ERR;
    }
    crc16Val = crc(CmdBuf + 2, 8);
    if(crc16Val != 0) {
        HAL_UART_Receive_IT(&huart2, (uint8_t *)mRxBufUart2, UART2_RXBUF_MAX);
        return ERR;
    }
    return OK;
}
#define   CMD_START_PC                    0x00
#define   CMD_WRITE_CHK                   0x01
#define   CMD_WRITE_START1                0x02
#define   CMD_WRITE_START2                0x03
#define   CMD_WRITE_START3                0x04

#define   CMD_READ_S1_S6_KEY              0x11
#define   CMD_READ_DC_M1_CURRENT          0x12
#define   CMD_READ_DC_M2_CURRENT          0x13
#define   CMD_READ_DC_M3_CURRENT          0x14
#define   CMD_READ_DC_M4_CURRENT          0x15

#define   CMD_READ_AC_FAN1_CURRENT        0x21
#define   CMD_READ_AC_FAN2_CURRENT        0x22
#define   CMD_READ_AC_FAN3_CURRENT        0x23

#define   CMD_READ_ALL                    0x2f

#define   CMD_WRITE_D0_D7_CS0             0x31
#define   CMD_WRITE_D0_D7_CS1             0x32
#define   CMD_WRITE_D0_D7_CS2             0x33
#define   CMD_WRITE_D0_D7_CS3             0x34
#define   CMD_WRITE_D0_D7_CS4             0x35

void sendBufUart1(uint8_t txLen)
{
    uint16_t i = 0;
    for(i = 0; i < 3 ;i++) {
        if(HAL_UART_Transmit_IT(&huart1,(uint8_t*)mTxBufUart1,txLen) != HAL_OK) {
            HAL_Delay(10);
        }
        else
            break;
    }
    if(i >=3)
        Error_Handler();
}

void sendBufUart2(uint8_t txLen)
{
    uint16_t i = 0;
    for(i = 0; i < 3 ;i++) {
        if(HAL_UART_Transmit_IT(&huart2,(uint8_t*)mTxBufUart2,txLen) != HAL_OK) {
            HAL_Delay(10);
        }
        else
            break;
    }
    if(i >=3)
        Error_Handler();
}
//这个函数可以作废了
void StatusUpdata(uint8_t CMD,uint8_t* data,uint8_t size)
{
    uint16_t crcVal = 0;
    if(size > UART2_TX_CMD_LEN - 5)
        size = UART2_TX_CMD_LEN - 5;
    mTxBufUart2[0] = 0x1b;
    mTxBufUart2[1] = 0x10;
    mTxBufUart2[2] = CMD;
    for(int i = 0; i < UART2_TX_CMD_LEN - 5;i++)
        mTxBufUart2[i + 3] = 0x00;
    for(int i = 0; i < size;i++)
        mTxBufUart2[3 + i] = data[i];
    crcVal = crc(mTxBufUart2 + 2, size + 1);
    mTxBufUart2[UART2_TX_CMD_LEN - 2] = crcVal & 0xff;
    mTxBufUart2[UART2_TX_CMD_LEN - 1] = (crcVal >> 8) & 0xff;
    sendBufUart2(UART2_TX_CMD_LEN);
	// while(1){
    //     if(__HAL_UART_GET_FLAG(&huart2,UART_FLAG_TC) == SET)
    //         break;
    //     times++;
    //     if(times > 0xA000)
    //         break;
    // }
    // mUart2Ready = RESET;
    return;
}

void AllStatusUpdata(void)
{
    uint16_t crcVal = 0, i = 0;
    uint8_t *prt;
    mTxBufUart2[0] = 0x1b;
    mTxBufUart2[1] = 0x10;
    mTxBufUart2[2] = CMD_READ_ALL;
    for(i = 0; i < 4; i++) {
        prt = (uint8_t *)mMotorCurrent[i];
        mTxBufUart2[3 + i * 4] = prt[0] & 0xff;
        mTxBufUart2[4 + i * 4] = prt[1] & 0xff;
        mTxBufUart2[5 + i * 4] = prt[2] & 0xff;
        mTxBufUart2[6 + i * 4] = prt[3] & 0xff;
    }
 
    for(i = 0; i < 3; i++) {
        prt = (uint8_t *)mFanCurrent[i];
        mTxBufUart2[23 + i * 4] = prt[0] & 0xff;
        mTxBufUart2[24 + i * 4] = prt[1] & 0xff;
        mTxBufUart2[25 + i * 4] = prt[2] & 0xff;
        mTxBufUart2[26 + i * 4] = prt[3] & 0xff;
    }
    mTxBufUart2[27] = mCheckStatus;
    mTxBufUart2[28] = mS1toS6Sta;
    mTxBufUart2[29] = mMotorSta[0] | (mMotorSta[1] << 1) | (mMotorSta[2] << 2) | (mMotorSta[3] << 3)
                         | (mFanSta[0] << 4) | (mFanSta[1] << 5) | (mFanSta[2] << 6);
    mTxBufUart2[30] = 0;        //预留
    mTxBufUart2[31] = 0;        //预留

    crcVal = crc(mTxBufUart2 + 2, 30);
    mTxBufUart2[32] = crcVal & 0xff;
    mTxBufUart2[33] = (crcVal >> 8) & 0xff;
    sendBufUart2(34);
}

uint8_t CMD_Process(void)
{   
    uint8_t cmd1Len = 0,cmd2Len = 0;
    uint8_t re;
    uint8_t status;
    uint8_t cmdType;
    if(mUart1Ready == SET) {
        cmd1Len = cmd1Len;//Do something
    }
    if(mUart2Ready == SET) {
        cmd2Len = huart2.RxXferSize - huart2.RxXferCount;
        re = CMD_Analysis(huart2.pRxBuffPtr,cmd2Len);
        if(re != OK)
          return re;
        cmdType = huart2.pRxBuffPtr[3];
        switch (cmdType)
        {
            case CMD_START_PC:
                mCheckStatus = 1;
                break;
            case CMD_WRITE_CHK:
                setCheckOut(huart2.pRxBuffPtr[4]);
                break;
            case CMD_WRITE_START1:
                setStart1Out(huart2.pRxBuffPtr[4]);
                break;
            case CMD_WRITE_START2:
                setStart2Out(huart2.pRxBuffPtr[4]);
                break;
            case CMD_WRITE_START3:
                setStart3Out(huart2.pRxBuffPtr[4]);
                break;

            case CMD_READ_S1_S6_KEY:
                status = getS1S6KeyStatus();
                StatusUpdata(CMD_READ_S1_S6_KEY,&status,1);
                break;
            case CMD_READ_ALL:
                AllStatusUpdata();
                break;
            default:
                break;
        }
        HAL_UART_Receive_IT(&huart2, (uint8_t *)mRxBufUart2, UART2_RXBUF_MAX);
    }
    return re;
}

//ADC
 void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc)
{
  /* Prevent unused argument(s) compilation warning */
	 UNUSED(hadc);
   mDmaTransferStatus = 1;//本次转换完成
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
  /* Prevent unused argument(s) compilation warning */
  if(huart->Instance == USART1)
	    mUart1Ready = SET;
  else if(huart->Instance == USART2) {
        mUart2Ready = SET;
  }
	    
  /* NOTE : This function should not be modified, when the callback is needed,
            the HAL_UART_RxCpltCallback can be implemented in the user file.
   */
}

void set573DataforS1S6(uint8_t bit)
{
    if(((mS1toS6Sta1 & bit) == bit) && ((mS1toS6Sta2 & bit) == 0x00)) {
        m573Status[0] |= bit;
        m573Status[1] &= ~bit;
    }
    else {
        m573Status[1] |= bit;
        m573Status[0] &= ~bit;
    }
    mS1toS6Sta = m573Status[1];     //0:OK  1： NG
}

void set573DataforMotorFan(uint8_t sta,uint8_t bit)
{
    if(sta == MACHINE_OK) {
        m573Status[2] |= bit;
        m573Status[3] &= ~bit;
    }
    else {
        m573Status[2] &= ~bit;
        m573Status[3] |= bit;
    }
}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
  GPIO_PinState keyin;
  GPIO_PinState keyinPrevious;
  int keyinCount = 0,avrMCurrrent = 0;;
  uint8_t inaAddress = 0,motorCurrentCount = 0,fanCurrentCount = 0, i = 0,j = 0;
  s32 avrFanCurrent = 0;
  uint32_t totalCount = 0;
  uint8_t systemStaMachine = 0;
  uint16_t motorStep[4] = {0};
  uint16_t fanStep[3] = {0};
  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_ADC1_Init();
  MX_I2C1_Init();
  MX_TIM14_Init();
  MX_USART1_UART_Init();
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */
    
  for (i = 0; i < ADC_CONVERTED_DATA_BUFFER_SIZE; i++)
  {
      mADCxConvertedData[i] = VAR_CONVERTED_DATA_INIT_VALUE;  //
  }
  if (HAL_ADCEx_Calibration_Start(&hadc1) != HAL_OK)//ADC
  {
      /* Calibration Error */
        Error_Handler();
  }
  if (HAL_ADC_Start_DMA(&hadc1,
                      (uint32_t *)mADCxConvertedData,
                      ADC_CONVERTED_DATA_BUFFER_SIZE
                      ) != HAL_OK)
  {
      /* ADC conversion start error */
        Error_Handler();
  }
  if (HAL_TIM_Base_Start_IT(&htim14) != HAL_OK)//start TIM14
  {
        Error_Handler();
  }
  
  if(HAL_UART_Receive_IT(&huart2, (uint8_t *)mRxBufUart2, UART2_RXBUF_MAX) != HAL_OK)
  {
        Error_Handler();
  }
  ina219_configureRegisters();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

  while (1)
  {
        CMD_Process();
        if(mTimerStatus != 1) {   //500us Timer
            continue;
        }
        ToggleWDI();
        totalCount++;
        if(mDmaTransferStatus == 1) {
            mFanCurrent[0][fanCurrentCount] = mADCxConvertedData[0]*3300/4096;
            mFanCurrent[1][fanCurrentCount] = mADCxConvertedData[1]*3300/4096;
            mFanCurrent[2][fanCurrentCount] = mADCxConvertedData[2]*3300/4096;
            fanCurrentCount++;
            if(fanCurrentCount >= 40) {
                fanCurrentCount = 0;
                for(i = 0;i < 3;i++) {
                    avrFanCurrent = 0;
                    for(j = 0;j < 40; j++) {
                        avrFanCurrent += mFanCurrent[i][j];
                    }
                    mFanCurrent[i][40] = avrFanCurrent / 40;
                    if(totalCount > 200 && totalCount < 1000) {     //100ms ~ 500ms
                        if(mFanCurrent[i][40] > FAN_CURRENT_MIN) {
                            fanStep[i]++;
                        }
                    }
                }
            }
        }
        //
        if(inaAddress == 0){
                mMotorCurrent[0][motorCurrentCount] = ina219_GetCurrent_uA(INA219_I2C_ADDRESS_CONF_0);
        }
        else if(inaAddress == 1){
                mMotorCurrent[1][motorCurrentCount] = ina219_GetCurrent_uA(INA219_I2C_ADDRESS_CONF_1);
        }
        else if(inaAddress == 2){
                mMotorCurrent[2][motorCurrentCount] = ina219_GetCurrent_uA(INA219_I2C_ADDRESS_CONF_4);
        }
        else if(inaAddress == 3){
                mMotorCurrent[3][motorCurrentCount] = ina219_GetCurrent_uA(INA219_I2C_ADDRESS_CONF_5);
        }
        inaAddress++;
        if(inaAddress >= 4) {
                inaAddress = 0;
                motorCurrentCount++;
        }
        if(motorCurrentCount >= 10) {
            motorCurrentCount = 0;
            for(i = 0;i < 4;i++) {
                avrMCurrrent = 0;
                for(j = 0;j < 10; j++) {
                    avrMCurrrent += mMotorCurrent[i][j];
                }
                mMotorCurrent[i][10] = avrMCurrrent / 10;
                if(totalCount > 200 && totalCount < 1000) {     //100ms ~ 500ms
                    if(mMotorCurrent[i][10] > MOTOR_CURRENT_F_MIN || mMotorCurrent[i][10] < MOTOR_CURRENT_B_MIN) {
                        motorStep[i]++;
                    }
                }
            }            
        }

        if(mCheckStatus != 1){
                //>>>>>>>>     Read keyin status    >>>>>>>>>>>>>>>>>>>>>
                keyin = getKeyinStatus();
                if(keyinPrevious == GPIO_PIN_RESET && keyin == GPIO_PIN_RESET) {
                    keyinCount++;
                    if(keyinCount > 200)    //100ms 去抖
                    {   mCheckStatus = 1;     //开始检测
                        totalCount = 0;
                        keyinCount = 0;
                        systemStaMachine = 1;
                        mS1toS6Sta1 = 0xff;
                        mS1toS6Sta2 = 0x00;
                        mS1toS6Sta = 0x00;      //0： OK ，1 ：NG
                        mInputStatus = 0x00;
                        for(i = 0;i < 4; i++) {
                            mMotorSta[i] = 0;
                            motorStep[i] = 0;
                        }
                        for(i = 0;i < 3; i++) {
                            mFanSta[i] = 0;
                            fanStep[i] = 0;
                        }
                        for(i = 0;i < 4; i++) {
                            m573Status[i] = 0;
                        }
                    }
                }
                else {
                    keyinCount = 0;
                }
                keyinPrevious = keyin;
            //<<<<<<<<<   Read keyin status     <<<<<<<<<<<<<<<<<<<<
            //   if(mRx2Num!= 0 && __HAL_UART_GET_IT(&huart2,UART_IT_IDLE)!=0)
            //   { 
            //       HAL_UART_RxCpltCallback(&huart2);
            //       __HAL_UART_CLEAR_IT(&huart2, UART_CLEAR_IDLEF); //手动置0清空标志位
            //   }
        }
        switch (systemStaMachine)
        {
            case 1:
                setCheckOut(1);
                setStart3Out(1);         //接通直流电机
                systemStaMachine = 2;
                break;
            case 2:
                if(totalCount > 1000)    //500ms
                    systemStaMachine = 3;
                break;
            case 3:
                setCheckOut(0);
                setStart3Out(0);         //断开直流电机，只要断开一个直接点击，S1.S2就会报警。
                systemStaMachine = 4;
                break;
            case 4:
                if(totalCount > 12000) {   //6000ms
                    systemStaMachine = 5;
                    mCheckStatus = 0; //检测完成，可以进行下一次循环检测    
                }
                break;
            default:
                break;
        }
        if(totalCount > 200 && totalCount < 1000) {    //这个周期内S1 和 S2都应该是开路的。开路都是高电平
            mS1toS6Sta1 &= (getS1S6KeyStatus() & 0xff); // 检测到一次错误就算错误。
        }
        if(totalCount > 1000 && totalCount < 1100) {     //500ms ~ 550ms  :（100ms~500ms） 400ms / 0.5ms / 40 = 20 , 应该有20 次，但是留有余量，可能中间有串口中断等情况
            if(motorStep[0] < 10 && MOTOR1 == VALID)
                mMotorSta[0] = MACHINE_NG;
            if(motorStep[1] < 10  && MOTOR2 == VALID)
                mMotorSta[1] = MACHINE_NG;
            if(motorStep[2] < 10  && MOTOR3 == VALID)
                mMotorSta[2] = MACHINE_NG;
            if(motorStep[3] < 10  && MOTOR4 == VALID)
                mMotorSta[3] = MACHINE_NG;
            if(fanStep[0] < 10 && FAN1 == VALID)
                mFanSta[0] = MACHINE_NG;
            if(fanStep[1] < 10 && FAN2 == VALID)
                mFanSta[1] = MACHINE_NG;
            if(fanStep[2] < 10 && FAN3 == VALID)
                mFanSta[2] = MACHINE_NG;
        }
        if(totalCount > 1100 && totalCount < 11900) {
            if(motorStep[0] == 0 && motorStep[1] == 0 && motorStep[2] == 0 && motorStep[3] == 0
                && fanStep[0] == 0 && fanStep[1] == 0 && fanStep[2] == 0) {
                    mInputStatus |= 0x01;
                    mInputStatus &= 0xfd;
                }
            else {
                mInputStatus &= 0xfe;
                mInputStatus |= 0x02;
            }
        }
        if(totalCount > 11900) {// 6秒后, 实际是断开气感信号5.5秒后。
            mS1toS6Sta2 |= (getS1S6KeyStatus() & 0xff);  //应该每次都是闭合的。闭路都是低电平
        }
        if(totalCount > 12000) {
            set573DataforS1S6(0x01);
            set573DataforS1S6(0x02);
            set573DataforS1S6(0x04);
            set573DataforS1S6(0x08);
            set573DataforS1S6(0x10);
            set573DataforS1S6(0x20);
            set573DataforMotorFan(mFanSta[0],0x01);
            set573DataforMotorFan(mFanSta[1],0x02);
            set573DataforMotorFan(mFanSta[2],0x04);
            set573DataforMotorFan(mMotorSta[0],0x08);
            set573DataforMotorFan(mMotorSta[1],0x10);
            set573DataforMotorFan(mMotorSta[2],0x20);
            set573DataforMotorFan(mMotorSta[3],0x40);    
        }
        
        if(totalCount < 10 || totalCount > 12000){
            setDforGreen(0, m573Status[0]);
            setDforRed(1, m573Status[1]);
            setDforGreen(2, m573Status[2]);
            setDforRed(3, m573Status[3]);
            setDforRed(4, m573Status[4]);
        }
        mTimerStatus = 0;
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSIDiv = RCC_HSI_DIV1;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV1;
  RCC_OscInitStruct.PLL.PLLN = 8;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */



/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
    HAL_Delay(100);
    Toggle_SYSTEM_OK_();
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
