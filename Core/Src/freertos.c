/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
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
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "i2c.h"
#include "itm_log.h"
#include <stdbool.h>
#include <string.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */

typedef struct {
    float tension;
    float current;
    bool fault;
} Alimentation;

Alimentation alim1;
Alimentation alim2;

/* USER CODE END Variables */
/* Definitions for heartBeatTask */
osThreadId_t heartBeatTaskHandle;
const osThreadAttr_t heartBeatTask_attributes = {
  .name = "heartBeatTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for i2cTask */
osThreadId_t i2cTaskHandle;
const osThreadAttr_t i2cTask_attributes = {
  .name = "i2cTask",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for ioTask */
osThreadId_t ioTaskHandle;
const osThreadAttr_t ioTask_attributes = {
  .name = "ioTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityLow,
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void StartHeartBeatTask(void *argument);
void StartI2CTask(void *argument);
void StartIOTask(void *argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of heartBeatTask */
  heartBeatTaskHandle = osThreadNew(StartHeartBeatTask, NULL, &heartBeatTask_attributes);

  /* creation of i2cTask */
  i2cTaskHandle = osThreadNew(StartI2CTask, NULL, &i2cTask_attributes);

  /* creation of ioTask */
  ioTaskHandle = osThreadNew(StartIOTask, NULL, &ioTask_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

}

/* USER CODE BEGIN Header_StartHeartBeatTask */
/**
* @brief Function implementing the heartBeatTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartHeartBeatTask */
void StartHeartBeatTask(void *argument)
{
  /* USER CODE BEGIN StartHeartBeatTask */
  LOG_INFO("heartBeatTask: Start");
  /* Infinite loop */
#pragma clang diagnostic push
#pragma ide diagnostic ignored "EndlessLoop"
  while (true) {
    if (!alim1.fault && !alim2.fault) {
      //LOG_INFO("heartBeatTask: Toggle LED");
      HAL_GPIO_TogglePin(HEART_BEAT_GPIO_Port, HEART_BEAT_Pin);
      osDelay(1000);
    } else {
      LOG_INFO("heartBeatTask: Fault blink");
      HAL_GPIO_WritePin(HEART_BEAT_GPIO_Port, HEART_BEAT_Pin, GPIO_PIN_RESET);
      osDelay(100);
      for (int i = 0; i < 6; i++) {
        HAL_GPIO_TogglePin(HEART_BEAT_GPIO_Port, HEART_BEAT_Pin);
        osDelay(100);
      }
      HAL_GPIO_WritePin(HEART_BEAT_GPIO_Port, HEART_BEAT_Pin, GPIO_PIN_RESET);
      osDelay(5000);
    }
  }
#pragma clang diagnostic pop
  /* USER CODE END StartHeartBeatTask */
}

/* USER CODE BEGIN Header_StartI2CTask */
/**
* @brief Function implementing the i2cTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartI2CTask */
void StartI2CTask(void *argument)
{
  /* USER CODE BEGIN StartI2CTask */
  LOG_INFO("i2cTask: Start");

  /* Infinite loop */
  uint8_t cmdBuffer[2];
  bool transmit;
#pragma clang diagnostic push
#pragma ide diagnostic ignored "EndlessLoop"
  while (true) {
    transmit = true;

    LOG_INFO("i2cTask: Wait command");
    if (HAL_I2C_Slave_Receive_IT(&hi2c1, (uint8_t *) cmdBuffer, 2) != HAL_OK) {
      LOG_ERROR("i2cTask: Erreur de reception de commande");
      continue;
    }
    LOG_INFO("i2cTask: Command received");

//    LOG_INFO("i2cTask: Wait end transfer");
//    while (HAL_I2C_GetState(&hi2c1) != HAL_I2C_STATE_READY) {
//      osDelay(1);
//    }

    if (cmdBuffer[0] == I2C_CMD_VERSION) {
      LOG_INFO("i2cTask: Command Version");
      if (HAL_I2C_Slave_Transmit_IT(&hi2c1, (uint8_t *) FIRMWARE_VERSION, strlen(FIRMWARE_VERSION)) != HAL_OK) {
        LOG_ERROR("i2cTask: Erreur de transmission de la version");
        continue;
      }

    } else if (cmdBuffer[0] == I2C_CMD_GET_DATA) {
      // 0 : Alim 1 current
      // 1 : Alim 1 tension
      // 2 : Alim 1 fault
      // 3 : Alim 2 current
      // 4 : Alim 2 tension
      // 5 : Alim 2 fault
      uint8_t txBuffer[6] = {3, 2 , alim1.fault, 1, 5, alim2.fault};
      if (HAL_I2C_Slave_Transmit_IT(&hi2c1, txBuffer, 6) != HAL_OK) {
        LOG_ERROR("i2cTask: Erreur de transmission des données de mesure");
        continue;
      }

    } else {
      LOG_WARN("i2cTask: Invalid command");
      transmit = false;
    }

//    if (transmit) {
//      LOG_INFO("i2cTask: Wait end transfer for data");
//      while (HAL_I2C_GetState(&hi2c1) != HAL_I2C_STATE_READY) {
//        osDelay(1);
//      }
//    }
  }
#pragma clang diagnostic pop
  /* USER CODE END StartI2CTask */
}

/* USER CODE BEGIN Header_StartIOTask */
/**
* @brief Function implementing the ioTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartIOTask */
void StartIOTask(void *argument)
{
  /* USER CODE BEGIN StartIOTask */
  LOG_INFO("ioTask: Start");
  /* Infinite loop */
#pragma clang diagnostic push
#pragma ide diagnostic ignored "EndlessLoop"
  while (true) {
    //LOG_INFO("ioTask: Read fault states");
    alim1.fault = HAL_GPIO_ReadPin(FAULT_1_GPIO_Port, FAULT_1_Pin) == GPIO_PIN_RESET;
    alim2.fault = HAL_GPIO_ReadPin(FAULT_2_GPIO_Port, FAULT_2_Pin) == GPIO_PIN_RESET;
    osDelay(2000);
  }
#pragma clang diagnostic pop
  /* USER CODE END StartIOTask */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
