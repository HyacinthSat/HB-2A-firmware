#ifndef FAKE_MAIN_H
#define FAKE_MAIN_H
#include <stdint.h>
typedef struct { int unused; } SPI_HandleTypeDef;
typedef enum { HAL_OK, HAL_ERROR } HAL_StatusTypeDef;
#define GPIO_PIN_RESET 0
#define GPIO_PIN_SET 1
#define RA02_NSS_GPIO_Port 1
#define RA02_NSS_Pin 4
#define RA02_RST_GPIO_Port 2
#define RA02_RST_Pin 0
#define LED_TX_GPIO_Port 2
#define LED_TX_Pin 9
uint32_t HAL_GetTick(void);
void HAL_Delay(uint32_t delay);
void HAL_GPIO_WritePin(int port, int pin, int value);
HAL_StatusTypeDef HAL_SPI_Transmit(SPI_HandleTypeDef *, uint8_t *, uint16_t, uint32_t);
HAL_StatusTypeDef HAL_SPI_TransmitReceive(SPI_HandleTypeDef *, uint8_t *, uint8_t *, uint16_t, uint32_t);
#define __DMB() ((void)0)
#define __get_PRIMASK() 0U
#define __disable_irq() ((void)0)
#define __set_PRIMASK(value) ((void)(value))
#endif
