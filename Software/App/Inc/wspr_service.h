#ifndef HB2A_WSPR_SERVICE_H
#define HB2A_WSPR_SERVICE_H

#include "telemetry_service.h"

#include <stddef.h>
#include <stdint.h>

#define WSPR_SERVICE_SYMBOL_COUNT 162U
#define WSPR_SERVICE_TEXT_CAPACITY 32U

typedef struct {
  uint8_t symbols[WSPR_SERVICE_SYMBOL_COUNT];
  char callsign[7];
  char grid[5];
  char text[WSPR_SERVICE_TEXT_CAPACITY];
  uint8_t power_dbm;
  uint8_t ready;
  int last_error;
} WsprFrame;

int wspr_service_build(const TelemetryFrame *telemetry, WsprFrame *out);
int wspr_service_build_standard(WsprFrame *out);

#endif
