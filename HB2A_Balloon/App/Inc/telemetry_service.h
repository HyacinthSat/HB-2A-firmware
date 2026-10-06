#ifndef HB2A_TELEMETRY_SERVICE_H
#define HB2A_TELEMETRY_SERVICE_H

#include "aprs.h"
#include "sensors.h"
#include "u4b_telemetry.h"

#include <stddef.h>
#include <stdint.h>

#define TELEMETRY_APRS_TEXT_CAPACITY 256U

typedef struct {
  uint8_t position_valid;
  uint8_t utc_valid;
  uint8_t fully_resolved;
  int32_t latitude_e7;
  int32_t longitude_e7;
  int32_t height_msl_mm;
  uint32_t ground_speed_mm_s;
  uint16_t year;
  uint8_t month;
  uint8_t day;
  uint8_t hour;
  uint8_t minute;
  uint8_t second;
} TelemetryGnssSnapshot;

typedef struct {
  U4B_Sample wspr_sample;
  U4B_Message wspr_message;
  APRS_Utc aprs_utc;
  APRS_Region aprs_region;
  uint32_t aprs_frequency_hz;
  char aprs_text[TELEMETRY_APRS_TEXT_CAPACITY];
  uint8_t position_valid;
  uint8_t sensor_data_valid;
  uint8_t wspr_ready;
} TelemetryFrame;

/* Build one consistent snapshot for both scheduled telemetry transmitters. */
int telemetry_service_build(const TelemetryGnssSnapshot *gnss,
                            const SensorSample *sensors,
                            uint8_t health_mask,
                            TelemetryFrame *out);

#endif
