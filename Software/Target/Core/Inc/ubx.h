#ifndef UBX_H
#define UBX_H
#include "stm32u5xx_hal.h"
#include <stdint.h>
/* MAX-M10S: configure RAM with VALSET, ACK and VALGET readback. */
int Ubx_Init(UART_HandleTypeDef *huart);
void Ubx_OnByte(uint8_t b);
int Ubx_ReadByte(uint8_t *b);
uint32_t Ubx_RxErrors(void);
int Ubx_ConfigureModule(void (*service)(void));
uint8_t Ubx_NavPvtHas3DFix(const uint8_t *p, uint16_t len);
uint8_t GNSS_OnUbxNavPvt(const uint8_t *payload, uint16_t len);
#endif
