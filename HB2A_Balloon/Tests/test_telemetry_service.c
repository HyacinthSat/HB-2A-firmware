#include "telemetry_service.h"

#include <assert.h>
#include <stdio.h>
#include <string.h>

static void test_valid_snapshot(void)
{
  TelemetryGnssSnapshot gnss = {0};
  SensorSample sensors = {0};
  TelemetryFrame frame;

  gnss.position_valid = 1U;
  gnss.position_known = 1U;
  gnss.utc_valid = 1U;
  gnss.fully_resolved = 1U;
  gnss.latitude_e7 = 380208333;
  gnss.longitude_e7 = 1124583333;
  gnss.height_msl_mm = 850000;
  gnss.ground_speed_mm_s = 18520U;
  gnss.year = 2026U;
  gnss.month = 10U;
  gnss.day = 6U;
  gnss.hour = 12U;
  gnss.minute = 34U;
  gnss.second = 56U;
  sensors.valid = SENSOR_RAILS_VALID | SENSOR_TEMP_VALID;
  sensors.rail_mv[0] = 3304U;
  sensors.temperature_centi_c = 2508;

  assert(telemetry_service_build(&gnss, &sensors, 0x15U, &frame) == 0);
  assert(frame.position_valid == 1U);
  assert(frame.sensor_data_valid == 1U);
  assert(frame.wspr_ready == 1U);
  assert(frame.wspr_sample.altitude_m == 850U);
  assert(frame.wspr_sample.speed_knots == 36U);
  assert(frame.wspr_sample.temperature_c == 25);
  assert(frame.wspr_sample.voltage_mv == 3304U);
  assert(frame.aprs_frequency_hz == 144640000UL);
  assert(strstr(frame.aprs_text, "BG7ZDQ-11>APZDQ7:/061234z") == frame.aprs_text);
  assert(strstr(frame.aprs_text, "T,3.30") != NULL);
  assert(strstr(frame.aprs_text, " B,15") != NULL);
}

static void test_invalid_position_and_sensor(void)
{
  TelemetryGnssSnapshot gnss = {0};
  SensorSample sensors = {0};
  TelemetryFrame frame;

  gnss.latitude_e7 = 380208333;
  gnss.longitude_e7 = 1124583333;
  gnss.position_known = 1U;
  assert(telemetry_service_build(&gnss, &sensors, 0U, &frame) == 0);
  assert(frame.position_valid == 0U);
  assert(frame.sensor_data_valid == 0U);
  assert(frame.wspr_ready == 1U);
  assert(frame.wspr_sample.latitude_e7 == 380208333);
  assert(frame.wspr_sample.longitude_e7 == 1124583333);
  assert(frame.wspr_sample.gps_valid == 0U);
  assert(strstr(frame.aprs_text, "3801.25N/11227.50E") != NULL);
  assert(frame.aprs_frequency_hz == 144640000UL);
  assert(strstr(frame.aprs_text, "BG7ZDQ-11>APZDQ7:!") == frame.aprs_text);
  assert(strstr(frame.aprs_text, "T,NA,NA") != NULL);
}

static void test_no_historical_position(void)
{
  TelemetryGnssSnapshot gnss = {0};
  SensorSample sensors = {0};
  TelemetryFrame frame;

  assert(telemetry_service_build(&gnss, &sensors, 0U, &frame) != 0);
}

static void test_known_but_invalid_position(void)
{
  TelemetryGnssSnapshot gnss = {0};
  SensorSample sensors = {0};
  TelemetryFrame frame;

  gnss.position_known = 1U;
  gnss.latitude_e7 = 900000001L;
  assert(telemetry_service_build(&gnss, &sensors, 0U, &frame) != 0);
}

int main(void)
{
  test_valid_snapshot();
  test_invalid_position_and_sensor();
  test_no_historical_position();
  test_known_but_invalid_position();
  puts("telemetry_service tests passed");
  return 0;
}
