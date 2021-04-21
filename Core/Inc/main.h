/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f1xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdbool.h>
/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */
typedef struct {
    double tension;
    double current;
    bool fault;
} Alimentation;

extern Alimentation alim1;
extern Alimentation alim2;

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
#define I2C_SLAVE_ADDRESS_BASE 0x20
#define ANA_VOLT_2_Pin GPIO_PIN_0
#define ANA_VOLT_2_GPIO_Port GPIOA
#define ANA_CURRENT_2_Pin GPIO_PIN_1
#define ANA_CURRENT_2_GPIO_Port GPIOA
#define FAULT_2_Pin GPIO_PIN_2
#define FAULT_2_GPIO_Port GPIOA
#define ANA_VOLT_1_Pin GPIO_PIN_3
#define ANA_VOLT_1_GPIO_Port GPIOA
#define ANA_CURRENT_1_Pin GPIO_PIN_4
#define ANA_CURRENT_1_GPIO_Port GPIOA
#define FAULT_1_Pin GPIO_PIN_5
#define FAULT_1_GPIO_Port GPIOA
#define BOOT_1_Pin GPIO_PIN_2
#define BOOT_1_GPIO_Port GPIOB
#define I2C_ADD_1_Pin GPIO_PIN_13
#define I2C_ADD_1_GPIO_Port GPIOB
#define I2C_ADD_2_Pin GPIO_PIN_14
#define I2C_ADD_2_GPIO_Port GPIOB
#define I2C_ADD_3_Pin GPIO_PIN_15
#define I2C_ADD_3_GPIO_Port GPIOB
#define HEART_BEAT_Pin GPIO_PIN_8
#define HEART_BEAT_GPIO_Port GPIOA
#define SCL_Pin GPIO_PIN_6
#define SCL_GPIO_Port GPIOB
#define SDA_Pin GPIO_PIN_7
#define SDA_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */
// Tension d'alimentation de référence
#define V_REF 3.3

// Résolution des convertisseurs ADC de la STM32F103C8
#define ADC_RESOLUTION 4096.0

// Valeur de convertion du pont diviseur de tension (13.6 V => 3.3 V)
// R1 = 1.8K ; R2 = 5.6K
// Vo = Vi * R1 / (R1 + R2) = Vi * 0.243243243
#define DIVISEUR_TENSION 0.243243243

// Résolution pour le courant de l'ACS711 15A alimenté en 3.3V, 90mV / A
#define ACS_RESOLUTION 90.0/1000.0

// Commande I2C
#define I2C_CMD_VERSION 'v'
#define I2C_CMD_GET_DATA 'g'
/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
