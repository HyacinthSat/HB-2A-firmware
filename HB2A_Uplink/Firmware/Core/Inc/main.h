/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
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
#include "stm32f1xx_hal.h"

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
#define RA02_NSS_Pin GPIO_PIN_4
#define RA02_NSS_GPIO_Port GPIOA
#define RA02_DIO0_Pin GPIO_PIN_1
#define RA02_DIO0_GPIO_Port GPIOA
#define RA02_DIO1_Pin GPIO_PIN_2
#define RA02_DIO1_GPIO_Port GPIOA
#define RA02_DIO2_Pin GPIO_PIN_3
#define RA02_DIO2_GPIO_Port GPIOA
#define RA02_RST_Pin GPIO_PIN_0
#define RA02_RST_GPIO_Port GPIOB
#define RA02_DIO3_Pin GPIO_PIN_14
#define RA02_DIO3_GPIO_Port GPIOB
#define RA02_DIO4_Pin GPIO_PIN_5
#define RA02_DIO4_GPIO_Port GPIOB
#define RA02_DIO5_Pin GPIO_PIN_6
#define RA02_DIO5_GPIO_Port GPIOB
#define LED_RX_Pin GPIO_PIN_8
#define LED_RX_GPIO_Port GPIOB
#define LED_TX_Pin GPIO_PIN_9
#define LED_TX_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */
/*
 * If receive blinks the silkscreen "TX" LED instead of "RX", your PCB likely swaps
 * the two nets — exchange LED_RX_Pin / LED_TX_Pin values above (e.g. 8 <-> 9).
 */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
