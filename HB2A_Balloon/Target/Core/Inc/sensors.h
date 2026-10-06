#ifndef HB_SENSORS_H
#define HB_SENSORS_H
#include <stdint.h>
#include <stddef.h>
#define SENSOR_RAILS_VALID 1U
#define SENSOR_TEMP_VALID 2U
#define SENSOR_RH_VALID 4U
#define SENSOR_PRESS_VALID 8U
typedef struct {
  /* Telemetry order: 3V3, 2V8, 1V5. Current remains signed. */
  uint16_t rail_mv[3];
  int32_t current_ua[3];
  int32_t temperature_centi_c;
  uint32_t humidity_centi_pct;
  uint32_t pressure_pa;
  uint32_t rail_tick, env_tick;
  uint8_t valid;
  uint8_t out_of_range; /* SENSOR_PRESS_VALID bit: append * to pressure. */
  int8_t ina_status, bme_status;
} SensorSample;
void Sensors_Init(void);
/* At most once/second, bounded I2C accesses, foreground only. */
void Sensors_Poll(void);
void Sensors_Get(SensorSample *out);
/* Format the nine documented T fields, no printf floating-point support. */
int Sensors_Format(const SensorSample *sample, char *out, size_t size);
#endif
