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
  gnss.position_known = 1U;
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

  assert(wspr_service_build_standard(&telemetry, &frame) == 0);
  assert(strcmp(frame.callsign, "BG7ZDQ") == 0);
  assert(strcmp(frame.grid, "OM68") == 0);
  assert(frame.power_dbm == 10U);
  assert(frame.ready == 1U);

  TelemetryFrame no_fix = {0};
  assert(wspr_service_build_standard(&no_fix, &frame) == 0);
  assert(strcmp(frame.grid, "RR73") == 0);
  assert(strstr(frame.text, "RR73") != NULL);

  TelemetryFrame fallback;
  SensorSample missing_sensors = {0};
  TelemetryGnssSnapshot missing_gnss = {0};
  assert(telemetry_service_build(&missing_gnss, &missing_sensors, 0U,
                                 &fallback) != 0);

  telemetry.wspr_ready = 0U;
  assert(wspr_service_build(&telemetry, &frame) == -2);
  assert(frame.ready == 0U);
  puts("wspr_service tests passed");
  return 0;
}
