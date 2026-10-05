#include "sensors.h"
#include <stdio.h>
#include <string.h>

static void decimal(char *out, int32_t value, uint32_t divisor, unsigned digits)
{
  uint32_t magnitude = value < 0 ? (uint32_t)(-(int64_t)value) : (uint32_t)value;
  uint32_t rounded = (magnitude + divisor / 2U) / divisor;
  uint32_t scale = digits == 1U ? 10U : 100U;
  (void)snprintf(out, 20U, "%s%lu.%0*lu", value < 0 ? "-" : "",
                 (unsigned long)(rounded / scale), (int)digits, (unsigned long)(rounded % scale));
}

int Sensors_Format(const SensorSample *s, char *out, size_t size)
{
  char fields[9][20];
  for (unsigned i = 0; i < 9; ++i) (void)strcpy(fields[i], "NA");
  if (s != NULL) {
    if (s->valid & SENSOR_RAILS_VALID) {
      for (unsigned i = 0; i < 3; ++i) {
        decimal(fields[2*i], s->rail_mv[i], 10U, 2U); /* V */
        decimal(fields[2*i+1], s->current_ua[i], 10000U, 2U); /* A */
      }
    }
    if (s->valid & SENSOR_TEMP_VALID) decimal(fields[6], s->temperature_centi_c, 10U, 1U);
    if (s->valid & SENSOR_RH_VALID) decimal(fields[7], (int32_t)s->humidity_centi_pct, 10U, 1U);
    if (s->valid & SENSOR_PRESS_VALID) {
      decimal(fields[8], (int32_t)s->pressure_pa, 10U, 1U); /* hPa */
      if (s->out_of_range & SENSOR_PRESS_VALID) (void)strcat(fields[8], "*");
    }
  }
  if (out == NULL || !size) return -1;
  int n = snprintf(out, size, "T,%s,%s,%s,%s,%s,%s,%s,%s,%s",
                   fields[0],fields[1],fields[2],fields[3],fields[4],fields[5],fields[6],fields[7],fields[8]);
  return n < 0 || (size_t)n >= size ? -1 : n;
}
