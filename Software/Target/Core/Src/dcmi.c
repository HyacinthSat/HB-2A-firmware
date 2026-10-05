/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    dcmi.c
  * @brief   This file provides code for the configuration
  *          of the DCMI instances.
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
#include "dcmi.h"

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

DCMI_HandleTypeDef hdcmi;
DMA_HandleTypeDef hdma_dcmi;

/* DCMI init function */
void MX_DCMI_Init(void)
{

  /* USER CODE BEGIN DCMI_Init 0 */

  /* USER CODE END DCMI_Init 0 */

  /* USER CODE BEGIN DCMI_Init 1 */
  __HAL_RCC_GPDMA1_CLK_ENABLE();
  hdma_dcmi.Instance = GPDMA1_Channel0;
  hdma_dcmi.Init.Request = GPDMA1_REQUEST_DCMI_PSSI;
  hdma_dcmi.Init.BlkHWRequest = DMA_BREQ_SINGLE_BURST;
  hdma_dcmi.Init.Direction = DMA_PERIPH_TO_MEMORY;
  hdma_dcmi.Init.SrcInc = DMA_SINC_FIXED;
  hdma_dcmi.Init.DestInc = DMA_DINC_INCREMENTED;
  hdma_dcmi.Init.SrcDataWidth = DMA_SRC_DATAWIDTH_WORD;
  hdma_dcmi.Init.DestDataWidth = DMA_DEST_DATAWIDTH_WORD;
  hdma_dcmi.Init.Priority = DMA_HIGH_PRIORITY;
  hdma_dcmi.Init.SrcBurstLength = 1U;
  hdma_dcmi.Init.DestBurstLength = 1U;
  hdma_dcmi.Init.TransferAllocatedPort = DMA_SRC_ALLOCATED_PORT0 | DMA_DEST_ALLOCATED_PORT1;
  hdma_dcmi.Init.TransferEventMode = DMA_TCEM_BLOCK_TRANSFER;
  hdma_dcmi.Init.Mode = DMA_NORMAL;
  if (HAL_DMA_Init(&hdma_dcmi) != HAL_OK) Error_Handler();
  /* USER CODE END DCMI_Init 1 */
  hdcmi.Instance = DCMI;
  hdcmi.Init.SynchroMode = DCMI_SYNCHRO_HARDWARE;
  hdcmi.Init.PCKPolarity = DCMI_PCKPOLARITY_RISING;
  hdcmi.Init.VSPolarity = DCMI_VSPOLARITY_HIGH;
  hdcmi.Init.HSPolarity = DCMI_HSPOLARITY_HIGH;
  hdcmi.Init.CaptureRate = DCMI_CR_ALL_FRAME;
  hdcmi.Init.ExtendedDataMode = DCMI_EXTEND_DATA_8B;
  hdcmi.Init.JPEGMode = DCMI_JPEG_ENABLE;
  hdcmi.Init.ByteSelectMode = DCMI_BSM_ALL;
  hdcmi.Init.ByteSelectStart = DCMI_OEBS_ODD;
  hdcmi.Init.LineSelectMode = DCMI_LSM_ALL;
  hdcmi.Init.LineSelectStart = DCMI_OELS_ODD;
  if (HAL_DCMI_Init(&hdcmi) != HAL_OK)
  {
    Error_Handler();
  }
  __HAL_LINKDMA(&hdcmi, DMA_Handle, hdma_dcmi);
  HAL_NVIC_SetPriority(GPDMA1_Channel0_IRQn, 4U, 0U);
  HAL_NVIC_EnableIRQ(GPDMA1_Channel0_IRQn);
  HAL_NVIC_SetPriority(DCMI_PSSI_IRQn, 5U, 0U);
  HAL_NVIC_EnableIRQ(DCMI_PSSI_IRQn);
  /* USER CODE BEGIN DCMI_Init 2 */

  /* USER CODE END DCMI_Init 2 */

}

void HAL_DCMI_MspInit(DCMI_HandleTypeDef* dcmiHandle)
{

  GPIO_InitTypeDef GPIO_InitStruct = {0};
  if(dcmiHandle->Instance==DCMI)
  {
  /* USER CODE BEGIN DCMI_MspInit 0 */

  /* USER CODE END DCMI_MspInit 0 */
    /* DCMI clock enable */
    __HAL_RCC_DCMI_PSSI_CLK_ENABLE();

    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOC_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();
    /**DCMI GPIO Configuration
    PA4     ------> DCMI_HSYNC
    PA6     ------> DCMI_PIXCLK
    PC8     ------> DCMI_D2
    PC9     ------> DCMI_D3
    PA9     ------> DCMI_D0
    PA10     ------> DCMI_D1
    PC11     ------> DCMI_D4
    PB6     ------> DCMI_D5
    PB7     ------> DCMI_VSYNC
    PB8     ------> DCMI_D6
    PB9     ------> DCMI_D7
    */
    GPIO_InitStruct.Pin = OV_HREF_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.Alternate = GPIO_AF10_DCMI;
    HAL_GPIO_Init(OV_HREF_GPIO_Port, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = OV_PCLK_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.Alternate = GPIO_AF4_DCMI;
    HAL_GPIO_Init(OV_PCLK_GPIO_Port, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = OV_D2_Pin|OV_D4_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.Alternate = GPIO_AF10_DCMI;
    HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = OV_D3_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.Alternate = GPIO_AF4_DCMI;
    HAL_GPIO_Init(OV_D3_GPIO_Port, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = OV_D0_Pin|OV_D1_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.Alternate = GPIO_AF5_DCMI;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = OV_D5_Pin|OV_VSYNC_Pin|OV_D6_Pin|OV_D7_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.Alternate = GPIO_AF10_DCMI;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* USER CODE BEGIN DCMI_MspInit 1 */

  /* USER CODE END DCMI_MspInit 1 */
  }
}

void HAL_DCMI_MspDeInit(DCMI_HandleTypeDef* dcmiHandle)
{

  if(dcmiHandle->Instance==DCMI)
  {
  /* USER CODE BEGIN DCMI_MspDeInit 0 */

  /* USER CODE END DCMI_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_DCMI_PSSI_CLK_DISABLE();

    /**DCMI GPIO Configuration
    PA4     ------> DCMI_HSYNC
    PA6     ------> DCMI_PIXCLK
    PC8     ------> DCMI_D2
    PC9     ------> DCMI_D3
    PA9     ------> DCMI_D0
    PA10     ------> DCMI_D1
    PC11     ------> DCMI_D4
    PB6     ------> DCMI_D5
    PB7     ------> DCMI_VSYNC
    PB8     ------> DCMI_D6
    PB9     ------> DCMI_D7
    */
    HAL_GPIO_DeInit(GPIOA, OV_HREF_Pin|OV_PCLK_Pin|OV_D0_Pin|OV_D1_Pin);

    HAL_GPIO_DeInit(GPIOC, OV_D2_Pin|OV_D3_Pin|OV_D4_Pin);

    HAL_GPIO_DeInit(GPIOB, OV_D5_Pin|OV_VSYNC_Pin|OV_D6_Pin|OV_D7_Pin);

  /* USER CODE BEGIN DCMI_MspDeInit 1 */

  /* USER CODE END DCMI_MspDeInit 1 */
  }
}

/* USER CODE BEGIN 1 */

/* USER CODE END 1 */

void GPDMA1_Channel0_IRQHandler(void)
{
  HAL_DMA_IRQHandler(&hdma_dcmi);
}

void DCMI_PSSI_IRQHandler(void)
{
  HAL_DCMI_IRQHandler(&hdcmi);
}


