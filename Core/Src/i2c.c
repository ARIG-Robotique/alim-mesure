/**
  ******************************************************************************
  * @file    i2c.c
  * @brief   This file provides code for the configuration
  *          of the I2C instances.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under Ultimate Liberty license
  * SLA0044, the "License"; You may not use this file except in compliance with
  * the License. You may obtain a copy of the License at:
  *                             www.st.com/SLA0044
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "i2c.h"

/* USER CODE BEGIN 0 */
#include "stdio.h"
#include "itm_log.h"
/* USER CODE END 0 */

I2C_HandleTypeDef hi2c1;

/* I2C1 init function */
void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */
  // Lecture des flags pour l'adresse I2C
  int i2cSlaveAddress = I2C_SLAVE_ADDRESS_BASE << 1;
  // Uncomment to use pin to change I2C address
  //i2cSlaveAddress += HAL_GPIO_ReadPin(I2C_ADD_1_GPIO_Port, I2C_ADD_1_Pin) == GPIO_PIN_SET ? 1 : 0;
  //i2cSlaveAddress += HAL_GPIO_ReadPin(I2C_ADD_2_GPIO_Port, I2C_ADD_2_Pin) == GPIO_PIN_SET ? 2 : 0;
  //i2cSlaveAddress += HAL_GPIO_ReadPin(I2C_ADD_2_GPIO_Port, I2C_ADD_2_Pin) == GPIO_PIN_SET ? 4 : 0;

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 100000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = i2cSlaveAddress;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

void HAL_I2C_MspInit(I2C_HandleTypeDef* i2cHandle)
{

  GPIO_InitTypeDef GPIO_InitStruct = {0};
  if(i2cHandle->Instance==I2C1)
  {
  /* USER CODE BEGIN I2C1_MspInit 0 */

  /* USER CODE END I2C1_MspInit 0 */

    __HAL_RCC_GPIOB_CLK_ENABLE();
    /**I2C1 GPIO Configuration
    PB6     ------> I2C1_SCL
    PB7     ------> I2C1_SDA
    */
    GPIO_InitStruct.Pin = SCL_Pin|SDA_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_OD;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    /* I2C1 clock enable */
    __HAL_RCC_I2C1_CLK_ENABLE();

    /* I2C1 interrupt Init */
    HAL_NVIC_SetPriority(I2C1_EV_IRQn, 5, 0);
    HAL_NVIC_EnableIRQ(I2C1_EV_IRQn);
    HAL_NVIC_SetPriority(I2C1_ER_IRQn, 5, 0);
    HAL_NVIC_EnableIRQ(I2C1_ER_IRQn);
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
    HAL_GPIO_DeInit(SCL_GPIO_Port, SCL_Pin);

    HAL_GPIO_DeInit(SDA_GPIO_Port, SDA_Pin);

    /* I2C1 interrupt Deinit */
    HAL_NVIC_DisableIRQ(I2C1_EV_IRQn);
    HAL_NVIC_DisableIRQ(I2C1_ER_IRQn);
  /* USER CODE BEGIN I2C1_MspDeInit 1 */

  /* USER CODE END I2C1_MspDeInit 1 */
  }
}

/* USER CODE BEGIN 1 */
static uint8_t cmd; 	// index of current cmd
static uint8_t getCommand = 1;
void HAL_I2C_ListenCpltCallback(I2C_HandleTypeDef *hi2c)
{
  LOG_INFO("Listen Complete Callback");
  getCommand = true;
  HAL_I2C_EnableListen_IT(hi2c); // slave is ready again
}

void HAL_I2C_AddrCallback(I2C_HandleTypeDef *hi2c, uint8_t TransferDirection, uint16_t AddrMatchCode)
{
  char buf[100];
  sprintf(buf, "i2c: Address Callback (Dir : %s ; Get Cmd : %d)", TransferDirection == I2C_DIRECTION_RECEIVE ? "RX" : "TX", getCommand);
  LOG_INFO(buf);
  if(TransferDirection == I2C_DIRECTION_TRANSMIT) {
    if(getCommand) {
      HAL_I2C_Slave_Seq_Receive_IT(hi2c, &cmd, 1, I2C_NEXT_FRAME);
    } else {
      // Implement else when data is received
      LOG_WARN("i2c: Address Callback, no data needed in this firmware");
    }
  } else {
    if (cmd == I2C_CMD_VERSION) {
      sprintf(buf, "i2c: Address Callback send version %s", FIRMWARE_VERSION);
      LOG_INFO("i2c: Address Callback send version");
      HAL_I2C_Slave_Seq_Transmit_IT(hi2c, FIRMWARE_VERSION, sizeof(FIRMWARE_VERSION), I2C_NEXT_FRAME);
    } else if (cmd == I2C_CMD_GET_DATA) {
      LOG_INFO("i2c: Address Callback send ADC data");
      uint8_t txBuffer[9];
      uint16_t tension, current;

      // 0-1   : Alim 1 tension
      tension = alim1.tension * 100;
      txBuffer[0] = (tension >> 8) & 0xFF;
      txBuffer[1] = tension & 0xFF;
      // 2-3   : Alim 1 current
      current = alim1.current * 100;
      txBuffer[2] = (current >> 8) & 0xFF;
      txBuffer[3] = current & 0xFF;

      // 4-5  : Alim 2 tension
      tension = alim2.tension * 100;
      txBuffer[4] = (tension >> 8) & 0xFF;
      txBuffer[5] = tension & 0xFF;
      // 6-7 : Alim 2 current
      current = alim2.current * 100;
      txBuffer[6] = (current >> 8) & 0xFF;
      txBuffer[7] = current & 0xFF;
      // 8    : 0 0 0 0 0 0 Alim 2 fault Alim 1 fault
      txBuffer[8] = (alim2.fault << 1) + alim1.fault;

//      for (int i = 0 ; i < sizeof(txBuffer); i++) {
//        sprintf(buf, "i2c: idx %d -> 0x%02X", i, txBuffer[i]);
//        LOG_DEBUG(buf);
//      }
      HAL_I2C_Slave_Seq_Transmit_IT(hi2c, txBuffer, sizeof(txBuffer), I2C_NEXT_FRAME);
    } else {
      LOG_WARN("i2c: Address Callback, unknown command");
    }
  }
}

void HAL_I2C_SlaveRxCpltCallback(I2C_HandleTypeDef *hi2c)
{
  if(getCommand) {
    char buf[100];
    sprintf(buf, "i2c: RX complete Callback -> Command = %hhu", cmd);
    LOG_INFO(buf);
    getCommand = false;
  } else {
    // Implement else if received value from command
    LOG_WARN("i2c: RX complete Callback, no command");
  }
}

void HAL_I2C_SlaveTxCpltCallback(I2C_HandleTypeDef *hi2c)
{
  char buf[100];
  sprintf(buf, "i2c: TX complete Callback -> Command = %hhu", cmd);
  LOG_INFO(buf);
}

void HAL_I2C_ErrorCallback(I2C_HandleTypeDef *hi2c)
{
  uint32_t errorCode = HAL_I2C_GetError(hi2c);
  if( errorCode == HAL_I2C_ERROR_AF ) {
    // transaction terminated by master
    LOG_ERROR("i2c: Error Callback -> transaction terminated by master" );
  } else {
    char buf[100];
    sprintf(buf, "i2c: Error Callback -> err=0x%02lX", errorCode);
    LOG_ERROR(buf);
  }
  getCommand = true;
  cmd = 0;
}

void HAL_I2C_AbortCpltCallback(I2C_HandleTypeDef *hi2c)
{
  LOG_INFO("i2c: Abort comptete callback" );  // never seen...
}

/* USER CODE END 1 */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
