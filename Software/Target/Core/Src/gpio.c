/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    gpio.c
  * @brief   This file provides code for the configuration
  *          of all used GPIO pins.
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

/* Includes ------------------------------------------------------------------*/
#include "gpio.h"

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/*----------------------------------------------------------------------------*/
/* Configure GPIO                                                             */
/*----------------------------------------------------------------------------*/
/* USER CODE BEGIN 1 */

/* USER CODE END 1 */

/** Configure pins
     PC14-OSC32_IN (PC14)   ------> RCC_OSC32_IN
     PC15-OSC32_OUT (PC15)   ------> RCC_OSC32_OUT
     PH0-OSC_IN (PH0)   ------> RCC_OSC_IN
     PH1-OSC_OUT (PH1)   ------> RCC_OSC_OUT
     PA13 (JTMS/SWDIO)   ------> DEBUG_JTMS-SWDIO
     PA14 (JTCK/SWCLK)   ------> DEBUG_JTCK-SWCLK
*/
void MX_GPIO_Init(void)
{

  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(FLASH_NSS_GPIO_Port, FLASH_NSS_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, CAM_DATA_CTRL_Pin|I2C_EN_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, FLASH_RST_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, CAM_A_EN_Pin|GNSS_RST_Pin|OV_RESET_Pin|OVRESET_EN_Pin
                          |CAM_B_EN_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(SX1278_NSS_GPIO_Port, SX1278_NSS_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin : DIO0_Pin */
  GPIO_InitStruct.Pin = DIO0_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(DIO0_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : FLASH_NSS_Pin CAM_DATA_CTRL_Pin I2C_EN_Pin */
  GPIO_InitStruct.Pin = FLASH_NSS_Pin|CAM_DATA_CTRL_Pin|I2C_EN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : FLASH_RST_Pin */
  GPIO_InitStruct.Pin = FLASH_RST_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* PA8/CAMOSC_EN is no longer used: the camera XCLK is generated on PC7. */
  GPIO_InitStruct.Pin = CAMOSC_EN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(CAMOSC_EN_GPIO_Port, &GPIO_InitStruct);

  /* SX1278 NRESET has an internal pull-up; release it with open-drain high. */
  GPIO_InitStruct.Pin = RSY_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(RSY_GPIO_Port, &GPIO_InitStruct);
  HAL_GPIO_WritePin(RSY_GPIO_Port, RSY_Pin, GPIO_PIN_SET);

  /*Configure GPIO pins : CAM_A_EN_Pin GNSS_RST_Pin SX1278_NSS_Pin
                           OVRESET_EN_Pin CAM_B_EN_Pin */
  GPIO_InitStruct.Pin = CAM_A_EN_Pin|GNSS_RST_Pin|SX1278_NSS_Pin
                          |OVRESET_EN_Pin|CAM_B_EN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* OV_RESET shares the load-switch output U22. Open drain lets U22 drive
     the released high level without a push-pull contention on the net. */
  GPIO_InitStruct.Pin = OV_RESET_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(OV_RESET_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : DIO1_Pin DIO2_Pin */
  GPIO_InitStruct.Pin = DIO1_Pin|DIO2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI11_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(EXTI11_IRQn);

  HAL_NVIC_SetPriority(EXTI12_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(EXTI12_IRQn);

  HAL_NVIC_SetPriority(EXTI13_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(EXTI13_IRQn);

}

/* USER CODE BEGIN 2 */

/* USER CODE END 2 */
