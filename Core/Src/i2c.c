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
#include "stdio.h"
#include "itm_log.h"

/* USER CODE END 0 */

I2C_HandleTypeDef hi2c1;
DMA_HandleTypeDef hdma_i2c1_rx;
DMA_HandleTypeDef hdma_i2c1_tx;

/* I2C1 init function */
void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */
  // Lecture des flags pour l'adresse I2C
  int i2cSlaveAddress = I2C_SLAVE_ADDRESS_BASE << 1;
  // Uncomment to use pin to change I2C address
  //i2cSlaveAddress += HAL_GPIO_ReadPin(I2C_ADD_1_GPIO_Port, I2C_ADD_1_Pin) == GPIO_PIN_SET ? 2 : 0;
  //i2cSlaveAddress += HAL_GPIO_ReadPin(I2C_ADD_2_GPIO_Port, I2C_ADD_2_Pin) == GPIO_PIN_SET ? 4 : 0;
  //i2cSlaveAddress += HAL_GPIO_ReadPin(I2C_ADD_3_GPIO_Port, I2C_ADD_3_Pin) == GPIO_PIN_SET ? 8 : 0;

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 400000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_16_9;
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

    /* I2C1 DMA Init */
    /* I2C1_RX Init */
    hdma_i2c1_rx.Instance = DMA1_Channel7;
    hdma_i2c1_rx.Init.Direction = DMA_PERIPH_TO_MEMORY;
    hdma_i2c1_rx.Init.PeriphInc = DMA_PINC_DISABLE;
    hdma_i2c1_rx.Init.MemInc = DMA_MINC_ENABLE;
    hdma_i2c1_rx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
    hdma_i2c1_rx.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
    hdma_i2c1_rx.Init.Mode = DMA_NORMAL;
    hdma_i2c1_rx.Init.Priority = DMA_PRIORITY_HIGH;
    if (HAL_DMA_Init(&hdma_i2c1_rx) != HAL_OK)
    {
      Error_Handler();
    }

    __HAL_LINKDMA(i2cHandle,hdmarx,hdma_i2c1_rx);

    /* I2C1_TX Init */
    hdma_i2c1_tx.Instance = DMA1_Channel6;
    hdma_i2c1_tx.Init.Direction = DMA_MEMORY_TO_PERIPH;
    hdma_i2c1_tx.Init.PeriphInc = DMA_PINC_DISABLE;
    hdma_i2c1_tx.Init.MemInc = DMA_MINC_ENABLE;
    hdma_i2c1_tx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
    hdma_i2c1_tx.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
    hdma_i2c1_tx.Init.Mode = DMA_NORMAL;
    hdma_i2c1_tx.Init.Priority = DMA_PRIORITY_HIGH;
    if (HAL_DMA_Init(&hdma_i2c1_tx) != HAL_OK)
    {
      Error_Handler();
    }

    __HAL_LINKDMA(i2cHandle,hdmatx,hdma_i2c1_tx);

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

    /* I2C1 DMA DeInit */
    HAL_DMA_DeInit(i2cHandle->hdmarx);
    HAL_DMA_DeInit(i2cHandle->hdmatx);

    /* I2C1 interrupt Deinit */
    HAL_NVIC_DisableIRQ(I2C1_EV_IRQn);
    HAL_NVIC_DisableIRQ(I2C1_ER_IRQn);
  /* USER CODE BEGIN I2C1_MspDeInit 1 */

  /* USER CODE END I2C1_MspDeInit 1 */
  }
}

/* USER CODE BEGIN 1 */
static uint8_t cmd;  // index of current cmd

void HAL_I2C_ListenCpltCallback(I2C_HandleTypeDef *hi2c) {
  if (i2cErrorCode == HAL_I2C_ERROR_BERR) {
    __HAL_I2C_CLEAR_FLAG(hi2c, I2C_FLAG_BERR);
    LOG_WARN("i2c: Listen Complete Callback -> flag BERR cleared");

  } else if (i2cErrorCode == HAL_I2C_ERROR_ARLO) {
    __HAL_I2C_CLEAR_FLAG(hi2c, I2C_FLAG_ARLO);
    LOG_WARN("i2c: Listen Complete Callback -> flag ARLO cleared");

  } else if (i2cErrorCode == HAL_I2C_ERROR_AF) {
    __HAL_I2C_CLEAR_FLAG(hi2c, I2C_FLAG_AF);
    LOG_WARN("i2c: Listen Complete Callback -> flag AF cleared");

  } else if (i2cErrorCode == HAL_I2C_ERROR_OVR) {
    __HAL_I2C_CLEAR_FLAG(hi2c, I2C_FLAG_OVR);
    LOG_WARN("i2c: Listen Complete Callback -> flag OVR cleared");
  }

  char buf[100];
  sprintf(buf, "i2c: Listen Complete Callback -> Command 0x%02X", cmd);
  LOG_INFO(buf);
  i2cErrorCode = HAL_I2C_ERROR_NONE;
  HAL_I2C_EnableListen_IT(hi2c); // slave is ready again
}

void HAL_I2C_AddrCallback(I2C_HandleTypeDef *hi2c, uint8_t TransferDirection, uint16_t AddrMatchCode) {
  char buf[100];
  sprintf(buf, "i2c: Address Callback (Dir : %s)", TransferDirection == I2C_DIRECTION_RECEIVE ? "RX" : "TX");
  LOG_INFO(buf);

  if (TransferDirection == I2C_DIRECTION_TRANSMIT) {
      HAL_I2C_Slave_Seq_Receive_DMA(hi2c, &cmd, 1, I2C_FIRST_FRAME);

  } else {
    if (cmd == I2C_CMD_VERSION) {
      sprintf(buf, "i2c: Address Callback send version %s", FIRMWARE_VERSION);
      LOG_INFO(buf);
      HAL_I2C_Slave_Seq_Transmit_DMA(hi2c, FIRMWARE_VERSION, sizeof(FIRMWARE_VERSION), I2C_NEXT_FRAME);

    } else if (cmd == I2C_CMD_GET_DATA) {
      LOG_INFO("i2c: Address Callback send all ADC data");
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

      HAL_I2C_Slave_Seq_Transmit_DMA(hi2c, txBuffer, sizeof(txBuffer), I2C_NEXT_FRAME);

    } else {
      LOG_WARN("i2c: Address Callback, unknown command");
      sprintf(buf, "i2c: Address Callback (Dir : %s) : Command = 0x%02X", TransferDirection == I2C_DIRECTION_RECEIVE ? "RX" : "TX", cmd);
      LOG_WARN(buf);

      uint8_t txBuffer[1] = {0xFF};
      HAL_I2C_Slave_Seq_Transmit_DMA(hi2c, txBuffer, 1, I2C_LAST_FRAME);
    }
  }
}

void HAL_I2C_SlaveRxCpltCallback(I2C_HandleTypeDef *hi2c) {
  char buf[100];
  sprintf(buf, "i2c: RX complete Callback -> Command = 0x%02X", cmd);
  LOG_INFO(buf);
}

void HAL_I2C_SlaveTxCpltCallback(I2C_HandleTypeDef *hi2c) {
  char buf[100];
  sprintf(buf, "i2c: TX complete Callback -> Command = 0x%02X", cmd);
  LOG_INFO(buf);
}

void HAL_I2C_ErrorCallback(I2C_HandleTypeDef *hi2c) {
  i2cErrorCode = HAL_I2C_GetError(hi2c);
  if (i2cErrorCode == HAL_I2C_ERROR_BERR) {
    // Bus error: This error happens when the interface detects an SDA’s rising or falling edge
    // while SCL is high, occurring in a non-valid position during a byte transfer.
    // During the byte transfer, if any invalid transition happens on the SDA line,
    // then the bus error is triggered, and the bus error flag in the status register will be set,
    // and if the interrupt is enabled, then an interrupt will be triggered on the interrupt IRQ line.
    LOG_WARN("i2c: Error Callback -> BUS Error");

  } else if (i2cErrorCode == HAL_I2C_ERROR_ARLO) {
    // Arbitration loss error: This error can occur when the interface loses the arbitration
    // of the bus to another master. Basically, arbitration loss error may happen only in the
    // multi-master bus configuration. The circuit having multiple masters is known as multi-master
    // configuration.
    LOG_WARN("i2c: Error Callback -> Arbitration loss error");

  } else if (i2cErrorCode == HAL_I2C_ERROR_AF) {
    // ACK failure error: This error occurs when no ACK is returned for the byte sent.
    LOG_WARN("i2c: Error Callback -> ACK failure error");

  } else if (i2cErrorCode == HAL_I2C_ERROR_OVR) {
    // Overrun error: Remember that the overrun always happens during the reception. When a new byte
    // is received before reading the previously received values in the data register, the newly
    // received data will be lost. In such cases, the overrun error occurs.
    //
    // For example, consider that there is one data register (DR) and a shift register (SR) in I2C.
    // The software reads the data from the data register. When SR receives one byte, it is moved
    // to the DR. Now assume that DR is full, one byte that is not yet read by the software is
    // already there in the DR, and the second byte is received in the SR. Now the SR has the second
    // byte, and DR has the first data byte. When the third byte comes, the I2C peripheral discards
    // the third byte since there is no place to keep the third byte received. This condition is
    // called overrun.
    //
    // The overrun error will not occur in I2C if clock stretching is enabled because this error
    // happens only when both the DR and SR register is filled. Another flag called BTF is set
    // immediately, indicating that both SR and DR registers are full, and the clock will be stretched
    // automatically. When the clock is stretched, both slave and master enter into the waiting state.
    // The BTF flag will get cleared only when the software reads a byte from the data register.
    //
    // If the clock stretching feature is not supported by the I2C peripheral, then overrun error
    // may happen, and the software has to be careful with that.
    //
    // Note: In I2C, the overrun and underrun will not happen if clock stretching is enabled because
    // in those conditions, the clock will be held LOW, and both communicating devices will enter
    // the wait state.
    LOG_WARN("i2c: Error Callback -> Overrun error");

  } else if (i2cErrorCode == HAL_I2C_ERROR_TIMEOUT) {
    // Time-Out error: The time-out error occurs when the master or slave stretches the clock by holding it low for more than the recommended amount of time. The recommended amount of time will be mentioned in the reference manual. If the stretching of the clock to low exceeds the recommended amount of time, then the time-out error will be triggered by the I2C peripheral.
    LOG_WARN("i2c: Error Callback -> Time-Out error");

  } else {
    char buf[100];
    sprintf(buf, "i2c: Error Callback -> err=0x%02lX", i2cErrorCode);
    LOG_ERROR(buf);
  }
}

void HAL_I2C_AbortCpltCallback(I2C_HandleTypeDef *hi2c) {
  LOG_INFO("i2c: Abort comptete callback");  // never seen...
}

/* USER CODE END 1 */
