#include "telemetry_service.h"

#include "app_config.h"

#include <string.h>

static int32_t round_centi_c(int32_t value)
{
  if (value >= 0) return (value + 50) / 100;
  return -(((-value) + 50) / 100);
}

static uint16_t speed_mm_s_to_knots(uint32_t speed_mm_s)
{
  uint64_t knots = ((uint64_t)speed_mm_s * 1943844ULL + 500000000ULL) / 1000000000ULL;
  return knots > 65535ULL ? 65535U : (uint16_t)knots;
}

static uint32_t height_mm_to_m(int32_t height_mm)
{
  if (height_mm <= 0) return 0U;
  return ((uint32_t)height_mm + 500U) / 1000U;
}

static uint8_t valid_position(const TelemetryGnssSnapshot *gnss)
{
  if (gnss == NULL || gnss->position_valid == 0U) return 0U;
  return gnss->latitude_e7 >= -900000000L && gnss->latitude_e7 <= 900000000L &&
         gnss->longitude_e7 >= -1800000000L && gnss->longitude_e7 <= 1800000000L;
}

static void fill_position_sample(const TelemetryGnssSnapshot *gnss, uint8_t position_valid,
                                 U4B_Sample *sample)
{
  memset(sample, 0, sizeof(*sample));
  sample->gps_valid = position_valid;
  /* Keep the last coordinate in the packet when the fix drops, while the
     gps_valid bit tells consumers that it is held rather than current. */
  if (gnss == NULL || gnss->position_known == 0U ||
      gnss->latitude_e7 < -900000000L ||
      gnss->latitude_e7 > 900000000L ||
      gnss->longitude_e7 < -1800000000L ||
      gnss->longitude_e7 > 1800000000L) return;
  sample->latitude_e7 = gnss->latitude_e7;
  sample->longitude_e7 = gnss->longitude_e7;
  sample->altitude_m = height_mm_to_m(gnss->height_msl_mm);
  sample->speed_knots = speed_mm_s_to_knots(gnss->ground_speed_mm_s);
}

static void fill_sensor_sample(const SensorSample *sensors, U4B_Sample *sample,
                               uint8_t *sensor_data_valid)
{
  *sensor_data_valid = 0U;
  sample->temperature_c = 0;
  sample->voltage_mv = 0U;
  if (sensors == NULL) return;
  if ((sensors->valid & SENSOR_TEMP_VALID) != 0U) {
    sample->temperature_c = (int16_t)round_centi_c(sensors->temperature_centi_c);
  }
  if ((sensors->valid & SENSOR_RAILS_VALID) != 0U) {
    sample->voltage_mv = sensors->rail_mv[0]; /* 3V3 load-side rail. */
  }
  if ((sensors->valid & (SENSOR_RAILS_VALID | SENSOR_TEMP_VALID)) ==
      (SENSOR_RAILS_VALID | SENSOR_TEMP_VALID)) {
    *sensor_data_valid = 1U;
  }
}

int telemetry_service_build(const TelemetryGnssSnapshot *gnss,
                            const SensorSample *sensors,
                            uint8_t health_mask,
                            TelemetryFrame *out)
{
  uint8_t position_valid;
  if (out == NULL) return -1;
  memset(out, 0, sizeof(*out));

  position_valid = valid_position(gnss);
  out->position_valid = position_valid;
  fill_position_sample(gnss, position_valid, &out->wspr_sample);
  if (gnss == NULL || gnss->position_known == 0U ||
      gnss->latitude_e7 < -900000000L || gnss->latitude_e7 > 900000000L ||
      gnss->longitude_e7 < -1800000000L || gnss->longitude_e7 > 1800000000L)
    return -3;
  fill_sensor_sample(sensors, &out->wspr_sample, &out->sensor_data_valid);
  U4B_BuildBasic(&out->wspr_sample, APP_WSPR_ID13, &out->wspr_message);
  /* Missing sensors are represented by zero/default fields in the U4B frame;
     keep the telemetry slot usable and expose validity through health bits. */
  out->wspr_ready = 1U;

  if (gnss != NULL && gnss->utc_valid != 0U && gnss->fully_resolved != 0U) {
    out->aprs_utc.valid = 1U;
    out->aprs_utc.year = gnss->year;
    out->aprs_utc.month = gnss->month;
    out->aprs_utc.day = gnss->day;
    out->aprs_utc.hour = gnss->hour;
    out->aprs_utc.minute = gnss->minute;
    out->aprs_utc.second = gnss->second;
  }

  out->aprs_frequency_hz = APRS_SelectFrequencyHz(
      out->wspr_sample.latitude_e7, out->wspr_sample.longitude_e7,
      position_valid, &out->aprs_region);
  if (APRS_BuildTelemetryWithSensors(&out->wspr_sample, &out->aprs_utc,
                                     sensors, health_mask,
                                     out->aprs_text, sizeof(out->aprs_text)) < 0) {
    return -2;
  }
  return 0;
}
