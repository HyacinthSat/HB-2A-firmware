#ifndef TEST_IMAGE_STORAGE_MAIN_H
#define TEST_IMAGE_STORAGE_MAIN_H

#include <stdint.h>

typedef struct { uint32_t unused; } GPIO_TypeDef;
typedef enum { GPIO_PIN_RESET = 0, GPIO_PIN_SET = 1 } GPIO_PinState;
typedef enum { HAL_OK = 0, HAL_ERROR = 1 } HAL_StatusTypeDef;
typedef struct { uint32_t unused; } SPI_HandleTypeDef;

extern GPIO_TypeDef mock_gpio;
extern SPI_HandleTypeDef hspi2;

#define FLASH_NSS_GPIO_Port (&mock_gpio)
#define FLASH_NSS_Pin       1U

void HAL_GPIO_WritePin(GPIO_TypeDef *port, uint32_t pin, GPIO_PinState state);
uint32_t HAL_GetTick(void);
void HAL_Delay(uint32_t milliseconds);

#endif
