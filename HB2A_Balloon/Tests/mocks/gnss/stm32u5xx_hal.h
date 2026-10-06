#ifndef TEST_GNSS_HAL_H
#define TEST_GNSS_HAL_H
#include <stdint.h>
typedef enum { HAL_OK, HAL_ERROR, HAL_BUSY } HAL_StatusTypeDef;
typedef struct { unsigned RxState; } UART_HandleTypeDef;
#define HAL_UART_STATE_READY 0U
#define HAL_UART_STATE_BUSY_RX 1U
#define __HAL_UART_CLEAR_OREFLAG(h) ((void)(h))
#define __DMB() ((void)0)
uint32_t HAL_GetTick(void);
void HAL_Delay(uint32_t milliseconds);
HAL_StatusTypeDef HAL_UART_AbortReceive(UART_HandleTypeDef *handle);
HAL_StatusTypeDef HAL_UART_Receive_IT(UART_HandleTypeDef *, uint8_t *, uint16_t);
HAL_StatusTypeDef HAL_UART_Transmit(UART_HandleTypeDef *, const uint8_t *, uint16_t, uint32_t);
#endif
