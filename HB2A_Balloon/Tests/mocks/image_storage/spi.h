#ifndef TEST_IMAGE_STORAGE_SPI_H
#define TEST_IMAGE_STORAGE_SPI_H

#include "main.h"

HAL_StatusTypeDef HAL_SPI_Transmit(SPI_HandleTypeDef *handle, uint8_t *data,
                                   uint16_t length, uint32_t timeout);
HAL_StatusTypeDef HAL_SPI_Receive(SPI_HandleTypeDef *handle, uint8_t *data,
                                  uint16_t length, uint32_t timeout);
HAL_StatusTypeDef HAL_SPI_TransmitReceive(SPI_HandleTypeDef *handle,
                                          uint8_t *tx, uint8_t *rx,
                                          uint16_t length, uint32_t timeout);

#endif
