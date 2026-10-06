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
#include "stm32u5xx_hal.h"

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
/* TAMP BKP1 is a reset-surviving camera breadcrumb; BKP0 is the boot count. */
typedef enum {
  BOARD_STAGE_IDLE = 0,
  BOARD_STAGE_CAMERA_OFF,
  BOARD_STAGE_CAMERA_CLOCK,
  BOARD_STAGE_CAMERA_RAILS,
  BOARD_STAGE_CAMERA_CONFIG,
  BOARD_STAGE_DCMI_INIT,
  BOARD_STAGE_CAPTURE,
  BOARD_STAGE_IMAGE_STORE,
  BOARD_STAGE_SSDV
} BoardStage;
void board_set_stage(BoardStage stage);

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define DIO0_Pin GPIO_PIN_13
#define DIO0_GPIO_Port GPIOC
#define DIO0_EXTI_IRQn EXTI13_IRQn
#define FLASH_NSS_Pin GPIO_PIN_0
#define FLASH_NSS_GPIO_Port GPIOC
#define CAM_DATA_CTRL_Pin GPIO_PIN_1
#define CAM_DATA_CTRL_GPIO_Port GPIOC
#define I2C_EN_Pin GPIO_PIN_3
#define I2C_EN_GPIO_Port GPIOC
#define DEBUG_UART_TX_Pin GPIO_PIN_0
#define DEBUG_UART_TX_GPIO_Port GPIOA
#define DEBUG_UART_RX_Pin GPIO_PIN_1
#define DEBUG_UART_RX_GPIO_Port GPIOA
#define GPS_UART_TX_Pin GPIO_PIN_2
#define GPS_UART_TX_GPIO_Port GPIOA
#define GPS_UART_RX_Pin GPIO_PIN_3
#define GPS_UART_RX_GPIO_Port GPIOA
#define OV_HREF_Pin GPIO_PIN_4
#define OV_HREF_GPIO_Port GPIOA
#define FLASH_RST_Pin GPIO_PIN_5
#define FLASH_RST_GPIO_Port GPIOA
#define OV_PCLK_Pin GPIO_PIN_6
#define OV_PCLK_GPIO_Port GPIOA
#define SYS_SCL_Pin GPIO_PIN_7
#define SYS_SCL_GPIO_Port GPIOA
#define CAM_A_EN_Pin GPIO_PIN_0
#define CAM_A_EN_GPIO_Port GPIOB
#define GNSS_RST_Pin GPIO_PIN_1
#define GNSS_RST_GPIO_Port GPIOB
#define OV_RESET_Pin GPIO_PIN_2
#define OV_RESET_GPIO_Port GPIOB
#define OV_SCL_Pin GPIO_PIN_10
#define OV_SCL_GPIO_Port GPIOB
#define SX1278_NSS_Pin GPIO_PIN_12
#define SX1278_NSS_GPIO_Port GPIOB
#define OV_SDA_Pin GPIO_PIN_14
#define OV_SDA_GPIO_Port GPIOB
#define OV_D2_Pin GPIO_PIN_8
#define OV_D2_GPIO_Port GPIOC
#define OV_D3_Pin GPIO_PIN_9
#define OV_D3_GPIO_Port GPIOC
#define CAMOSC_EN_Pin GPIO_PIN_8
#define CAMOSC_EN_GPIO_Port GPIOA
#define OV_D0_Pin GPIO_PIN_9
#define OV_D0_GPIO_Port GPIOA
#define OV_D1_Pin GPIO_PIN_10
#define OV_D1_GPIO_Port GPIOA
#define DIO1_Pin GPIO_PIN_11
#define DIO1_GPIO_Port GPIOA
#define DIO1_EXTI_IRQn EXTI11_IRQn
#define DIO2_Pin GPIO_PIN_12
#define DIO2_GPIO_Port GPIOA
#define DIO2_EXTI_IRQn EXTI12_IRQn
#define RSY_Pin GPIO_PIN_15
#define RSY_GPIO_Port GPIOA
#define OV_D4_Pin GPIO_PIN_11
#define OV_D4_GPIO_Port GPIOC
#define OVRESET_EN_Pin GPIO_PIN_3
#define OVRESET_EN_GPIO_Port GPIOB
#define SYS_SDA_Pin GPIO_PIN_4
#define SYS_SDA_GPIO_Port GPIOB
#define CAM_B_EN_Pin GPIO_PIN_5
#define CAM_B_EN_GPIO_Port GPIOB
#define OV_D5_Pin GPIO_PIN_6
#define OV_D5_GPIO_Port GPIOB
#define OV_VSYNC_Pin GPIO_PIN_7
#define OV_VSYNC_GPIO_Port GPIOB
#define OV_D6_Pin GPIO_PIN_8
#define OV_D6_GPIO_Port GPIOB
#define OV_D7_Pin GPIO_PIN_9
#define OV_D7_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
