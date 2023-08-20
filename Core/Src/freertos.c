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
#include "adc.h"
#include "itm_log.h"
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

/* USER CODE END Variables */
/* Definitions for mainTask */
osThreadId_t mainTaskHandle;
const osThreadAttr_t mainTask_attributes = {
  .name = "mainTask",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for heartBeatTimer */
osTimerId_t heartBeatTimerHandle;
const osTimerAttr_t heartBeatTimer_attributes = {
  .name = "heartBeatTimer"
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void StartMainTask(void *argument);
void heartBeatCallback(void *argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */
  LOG_INFO("freertos: Init FreeRTOS");
  /* USER CODE END Init */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* Create the timer(s) */
  /* creation of heartBeatTimer */
  heartBeatTimerHandle = osTimerNew(heartBeatCallback, osTimerPeriodic, NULL, &heartBeatTimer_attributes);

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of mainTask */
  mainTaskHandle = osThreadNew(StartMainTask, NULL, &mainTask_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

}

/* USER CODE BEGIN Header_StartMainTask */
/**
  * @brief  Function implementing the mainTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartMainTask */
void StartMainTask(void *argument)
{
  /* USER CODE BEGIN StartMainTask */
  LOG_INFO("mainTask: Start");

  // Start timers after boot init
  osTimerStart(heartBeatTimerHandle, 1000);

  /* Infinite loop */
#pragma clang diagnostic push
#pragma ide diagnostic ignored "EndlessLoop"
  while (true) {
    LOG_INFO("ioTask: Read fault states");
    alim1.fault = HAL_GPIO_ReadPin(FAULT_1_GPIO_Port, FAULT_1_Pin) == GPIO_PIN_RESET;
    alim2.fault = HAL_GPIO_ReadPin(FAULT_2_GPIO_Port, FAULT_2_Pin) == GPIO_PIN_RESET;

    uint32_t rawAdc;

    LOG_INFO("ioTask: Read ADC Alim 1 Volt");
    adcSelectAlim1Volt();
    HAL_ADC_Start(&hadc1);
    HAL_ADC_PollForConversion(&hadc1, 1000);
    rawAdc = HAL_ADC_GetValue(&hadc1);
    HAL_ADC_Stop(&hadc1);
    alim1.tension = (rawAdc / ADC_RESOLUTION) * (V_REF / DIVISEUR_TENSION);

    LOG_INFO("ioTask: Read ADC Alim 1 Current");
    adcSelectAlim1Current();
    HAL_ADC_Start(&hadc1);
    HAL_ADC_PollForConversion(&hadc1, 1000);
    rawAdc = HAL_ADC_GetValue(&hadc1);
    HAL_ADC_Stop(&hadc1);
    alim1.current = (rawAdc / ADC_RESOLUTION) * V_REF * ACS_RESOLUTION;

    LOG_INFO("ioTask: Read ADC Alim 2 Volt");
    adcSelectAlim2Volt();
    HAL_ADC_Start(&hadc1);
    HAL_ADC_PollForConversion(&hadc1, 1000);
    rawAdc = HAL_ADC_GetValue(&hadc1);
    HAL_ADC_Stop(&hadc1);
    alim2.tension = (rawAdc / ADC_RESOLUTION) * (V_REF / DIVISEUR_TENSION);

    LOG_INFO("ioTask: Read ADC Alim 2 Current");
    adcSelectAlim2Current();
    HAL_ADC_Start(&hadc1);
    HAL_ADC_PollForConversion(&hadc1, 1000);
    rawAdc = HAL_ADC_GetValue(&hadc1);
    HAL_ADC_Stop(&hadc1);
    alim2.current = (rawAdc / ADC_RESOLUTION) * V_REF * ACS_RESOLUTION;

    osDelay(1000);
  }
#pragma clang diagnostic pop
  /* USER CODE END StartMainTask */
}

/* heartBeatCallback function */
void heartBeatCallback(void *argument)
{
  /* USER CODE BEGIN heartBeatCallback */
  if (i2cErrorCode == HAL_I2C_ERROR_NONE && !alim1.fault && !alim2.fault) { // Pas d'erreur
    HAL_GPIO_TogglePin(HEART_BEAT_GPIO_Port, HEART_BEAT_Pin);

  } else { // Erreur I2C ou alim
    HAL_GPIO_WritePin(HEART_BEAT_GPIO_Port, HEART_BEAT_Pin, GPIO_PIN_SET);
  }
  /* USER CODE END heartBeatCallback */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */

