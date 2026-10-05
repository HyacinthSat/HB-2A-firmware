#include "telemetry_service.h"
#include "wspr_service.h"

#include <assert.h>
#include <stdio.h>
#include <string.h>

int main(void)
{
  TelemetryGnssSnapshot gnss = {0};
  SensorSample sensors = {0};
  TelemetryFrame telemetry;
  WsprFrame frame;

  gnss.position_valid = 1U;
  gnss.utc_valid = 1U;
  gnss.fully_resolved = 1U;
  gnss.latitude_e7 = 380208333;
  gnss.longitude_e7 = 1124583333;
  gnss.height_msl_mm = 850000;
  gnss.year = 2026U;
  gnss.month = 10U;
  gnss.day = 6U;
  gnss.hour = 12U;
  sensors.valid = SENSOR_RAILS_VALID | SENSOR_TEMP_VALID;
  sensors.rail_mv[0] = 3250U;
  sensors.temperature_centi_c = 3520;
  assert(telemetry_service_build(&gnss, &sensors, 0x14U, &telemetry) == 0);
  assert(wspr_service_build(&telemetry, &frame) == 0);
  assert(frame.ready == 1U);
  assert(strlen(frame.callsign) == 6U);
  assert(strlen(frame.grid) == 4U);
  assert(strlen(frame.text) > 8U);
  assert(strstr(frame.text, "dBm") != NULL);
  assert(frame.symbols[0] <= 3U && frame.symbols[161] <= 3U);

  telemetry.wspr_ready = 0U;
  assert(wspr_service_build(&telemetry, &frame) == -2);
  assert(frame.ready == 0U);
  puts("wspr_service tests passed");
  return 0;
}
