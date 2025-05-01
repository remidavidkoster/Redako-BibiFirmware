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
#define BUT2_Pin GPIO_PIN_13
#define BUT2_GPIO_Port GPIOC
#define VBUS_PRESENT_Pin GPIO_PIN_14
#define VBUS_PRESENT_GPIO_Port GPIOC
#define BOOST_ENABLE_Pin GPIO_PIN_1
#define BOOST_ENABLE_GPIO_Port GPIOA
#define BUT1_Pin GPIO_PIN_4
#define BUT1_GPIO_Port GPIOA
#define SELF_TURN_ON_Pin GPIO_PIN_5
#define SELF_TURN_ON_GPIO_Port GPIOA
#define NRF_IRQ_Pin GPIO_PIN_1
#define NRF_IRQ_GPIO_Port GPIOB
#define CHRG_Pin GPIO_PIN_2
#define CHRG_GPIO_Port GPIOB
#define NRF_CE_Pin GPIO_PIN_11
#define NRF_CE_GPIO_Port GPIOB
#define NRF_SS_Pin GPIO_PIN_12
#define NRF_SS_GPIO_Port GPIOB
#define STDBY_Pin GPIO_PIN_6
#define STDBY_GPIO_Port GPIOC
#define ENC_CSN_Pin GPIO_PIN_15
#define ENC_CSN_GPIO_Port GPIOA
#define CHARGE_ENABLE_Pin GPIO_PIN_10
#define CHARGE_ENABLE_GPIO_Port GPIOC
#define DRIVER_ENABLE1_Pin GPIO_PIN_3
#define DRIVER_ENABLE1_GPIO_Port GPIOB
#define DRIVER_FAULT1_Pin GPIO_PIN_4
#define DRIVER_FAULT1_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
