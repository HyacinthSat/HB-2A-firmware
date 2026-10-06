#include "main.h"
typedef struct { int unused; } SPI_HandleTypeDef;
extern SPI_HandleTypeDef hspi2;
HAL_StatusTypeDef HAL_SPI_TransmitReceive(SPI_HandleTypeDef*,uint8_t*,uint8_t*,uint16_t,uint32_t);
HAL_StatusTypeDef HAL_SPI_Transmit(SPI_HandleTypeDef*,uint8_t*,uint16_t,uint32_t);
uint32_t HAL_SPI_GetError(SPI_HandleTypeDef*);
