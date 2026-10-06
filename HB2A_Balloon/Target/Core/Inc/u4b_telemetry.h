#ifndef U4B_TELEMETRY_H
#define U4B_TELEMETRY_H

#include <stdint.h>

typedef struct {
  int32_t latitude_e7;
  int32_t longitude_e7;
  uint32_t altitude_m;
  int16_t temperature_c;
  uint16_t voltage_mv;
  uint16_t speed_knots;
  uint8_t gps_valid;
} U4B_Sample;

typedef struct {
  uint32_t call_value;
  uint32_t grid_value;
  uint64_t big_number;
  char callsign[7];
  char grid4[5];
  uint8_t power_dbm;
} U4B_Message;

void U4B_BuildBasic(const U4B_Sample *sample, const char id13[3], U4B_Message *message);
void U4B_BuildBasicWithGrid56(const U4B_Sample *sample, const char id13[3], const char grid56[3], U4B_Message *message);
void U4B_Grid56FromLatLonE7(int32_t latitude_e7, int32_t longitude_e7, char grid56[3]);

#endif
