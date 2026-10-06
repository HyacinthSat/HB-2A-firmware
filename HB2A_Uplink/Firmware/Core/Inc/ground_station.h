#ifndef GROUND_STATION_H
#define GROUND_STATION_H
#include <stdint.h>
void GroundStation_Init(void);
void GroundStation_Poll(void);
void GroundStation_Receive(const uint8_t *data, uint32_t length);
void GroundStation_ResetInput(void);
void GroundStation_SetLineCoding(const uint8_t *data);
void GroundStation_GetLineCoding(uint8_t *data);
#endif
