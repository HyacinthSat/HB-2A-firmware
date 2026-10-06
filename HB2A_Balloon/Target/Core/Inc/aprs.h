#ifndef APRS_H
#define APRS_H

#include <stddef.h>
#include <stdint.h>
#include "u4b_telemetry.h"
#include "sensors.h"

typedef struct {
  uint8_t valid;
  uint16_t year;
  uint8_t month;
  uint8_t day;
  uint8_t hour;
  uint8_t minute;
  uint8_t second;
} APRS_Utc;

typedef enum {
  APRS_REGION_ASIA = 0,
  APRS_REGION_NORTH_AMERICA,
  APRS_REGION_EUROPE,
  APRS_REGION_AUSTRALIA,
  APRS_REGION_JAPAN,
  APRS_REGION_NEW_ZEALAND
} APRS_Region;

/* Returns the regional 2 m APRS frequency in Hz. */
uint32_t APRS_SelectFrequencyHz(int32_t latitude_e7, int32_t longitude_e7,
                                uint8_t valid, APRS_Region *region);

/* Builds the project APRS telemetry text, including the AX.25 information field. */
int APRS_BuildTelemetry(const U4B_Sample *sample, const APRS_Utc *utc,
                        uint8_t health_mask, char *out, size_t out_size);
int APRS_BuildTelemetryWithSensors(const U4B_Sample *sample, const APRS_Utc *utc,
                                  const SensorSample *sensors, uint8_t health_mask,
                                  char *out, size_t out_size);

#endif
