/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
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
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32g4xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

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
#define BUT3_Pin GPIO_PIN_13
#define BUT3_GPIO_Port GPIOC
#define BUT2_Pin GPIO_PIN_14
#define BUT2_GPIO_Port GPIOC
#define BUT1_Pin GPIO_PIN_15
#define BUT1_GPIO_Port GPIOC
#define SPI1_INT_Pin GPIO_PIN_4
#define SPI1_INT_GPIO_Port GPIOC
#define SPI1_CSN_Pin GPIO_PIN_0
#define SPI1_CSN_GPIO_Port GPIOB
#define LED_NOFF_Pin GPIO_PIN_2
#define LED_NOFF_GPIO_Port GPIOB
#define NRF_IRQ_Pin GPIO_PIN_10
#define NRF_IRQ_GPIO_Port GPIOB
#define NRF_CE_Pin GPIO_PIN_11
#define NRF_CE_GPIO_Port GPIOB
#define NRF_SS_Pin GPIO_PIN_12
#define NRF_SS_GPIO_Port GPIOB
#define SELF_TURN_ON_Pin GPIO_PIN_6
#define SELF_TURN_ON_GPIO_Port GPIOC
#define MOT_ENABLE_Pin GPIO_PIN_11
#define MOT_ENABLE_GPIO_Port GPIOA
#define CHARGE_ENABLE_Pin GPIO_PIN_12
#define CHARGE_ENABLE_GPIO_Port GPIOA
#define SPI3_CSN_Pin GPIO_PIN_11
#define SPI3_CSN_GPIO_Port GPIOC

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
