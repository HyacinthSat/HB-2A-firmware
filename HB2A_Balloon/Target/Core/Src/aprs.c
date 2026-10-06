#include "aprs.h"
#include "app_config.h"

#include <stdio.h>
#include <string.h>

static uint32_t abs_i32(int32_t value)
{
  return value < 0 ? (uint32_t)(-(int64_t)value) : (uint32_t)value;
}

static void format_latitude(int32_t value_e7, char out[10])
{
  uint32_t value = abs_i32(value_e7);
  uint32_t degrees = value / 10000000U;
  uint32_t remainder = value % 10000000U;
  uint32_t hundredths = (uint32_t)(((uint64_t)remainder * 6000ULL + 5000000ULL) / 10000000ULL);
  if (hundredths >= 6000U) {
    hundredths = 0U;
    ++degrees;
  }
  (void)snprintf(out, 10U, "%02lu%02lu.%02lu%c",
                 (unsigned long)degrees,
                 (unsigned long)(hundredths / 100U),
                 (unsigned long)(hundredths % 100U),
                 value_e7 < 0 ? 'S' : 'N');
}

static void format_longitude(int32_t value_e7, char out[11])
{
  uint32_t value = abs_i32(value_e7);
  uint32_t degrees = value / 10000000U;
  uint32_t remainder = value % 10000000U;
  uint32_t hundredths = (uint32_t)(((uint64_t)remainder * 6000ULL + 5000000ULL) / 10000000ULL);
  if (hundredths >= 6000U) {
    hundredths = 0U;
    ++degrees;
  }
  (void)snprintf(out, 11U, "%03lu%02lu.%02lu%c",
                 (unsigned long)degrees,
                 (unsigned long)(hundredths / 100U),
                 (unsigned long)(hundredths % 100U),
                 value_e7 < 0 ? 'W' : 'E');
}

static uint32_t altitude_feet(uint32_t altitude_m)
{
  uint64_t feet = ((uint64_t)altitude_m * 328084ULL + 50000ULL) / 100000ULL;
  return feet > 999999ULL ? 999999U : (uint32_t)feet;
}

uint32_t APRS_SelectFrequencyHz(int32_t latitude_e7, int32_t longitude_e7,
                                uint8_t valid, APRS_Region *region)
{
  int32_t lat = latitude_e7 / 10000000L;
  int32_t lon = longitude_e7 / 10000000L;
  APRS_Region selected = APRS_REGION_ASIA;
  uint32_t frequency = APP_APRS_DEFAULT_FREQ_HZ;

  if (valid != 0U) {
    if (lat >= 7 && lat <= 72 && lon >= -170 && lon <= -50) {
      selected = APRS_REGION_NORTH_AMERICA;
      frequency = 144390000UL;
    } else if (lat >= 35 && lat <= 72 && lon >= -15 && lon <= 45) {
      selected = APRS_REGION_EUROPE;
      frequency = 144800000UL;
    } else if (lat >= -45 && lat <= -10 && lon >= 110 && lon <= 155) {
      selected = APRS_REGION_AUSTRALIA;
      frequency = 145175000UL;
    } else if (lat >= 24 && lat <= 46 && lon >= 123 && lon <= 146) {
      selected = APRS_REGION_JAPAN;
      frequency = 144640000UL;
    } else if (lat >= -48 && lat <= -33 && lon >= 165 && lon <= 180) {
      selected = APRS_REGION_NEW_ZEALAND;
      frequency = 144575000UL;
    }
  }
  if (region != NULL) *region = selected;
  return frequency;
}

int APRS_BuildTelemetry(const U4B_Sample *sample, const APRS_Utc *utc,
                        uint8_t health_mask, char *out, size_t out_size)
{
  return APRS_BuildTelemetryWithSensors(sample, utc, NULL, health_mask, out, out_size);
}

int APRS_BuildTelemetryWithSensors(const U4B_Sample *sample, const APRS_Utc *utc,
                                  const SensorSample *sensors, uint8_t health_mask,
                                  char *out, size_t out_size)
{
  char lat[10];
  char lon[11];
  uint32_t feet;
  uint8_t day = 1U;
  uint8_t hour = 0U;
  uint8_t minute = 0U;
  char timestamp[9] = "!";
  char readings[184];
  int count;

  if (sample == NULL || out == NULL || out_size == 0U) return -1;
  if (sample->latitude_e7 < -900000000L || sample->latitude_e7 > 900000000L ||
      sample->longitude_e7 < -1800000000L || sample->longitude_e7 > 1800000000L) return -1;
  if (utc != NULL && utc->valid != 0U) {
    day = utc->day;
    hour = utc->hour;
    minute = utc->minute;
    if (day < 1U || day > 31U || hour > 23U || minute > 59U) return -1;
    (void)snprintf(timestamp, sizeof(timestamp), "/%02u%02u%02uz",
                   (unsigned)day, (unsigned)hour, (unsigned)minute);
  }
  format_latitude(sample->latitude_e7, lat);
  format_longitude(sample->longitude_e7, lon);
  feet = altitude_feet(sample->altitude_m);
  if (Sensors_Format(sensors, readings, sizeof(readings)) < 0) return -2;

  /* Never transmit fabricated rail readings. NA means not yet measured.
     No valid UTC: use an untimestamped position (!) rather than inventing a date. */
  count = snprintf(out, out_size,
                   "%s>%s:%s%s/%sO/A=%06lu %s B,%02X%s",
                   APP_APRS_CALL, APP_APRS_PATH,
                   timestamp,
                   lat, lon, (unsigned long)feet,
                   readings, (unsigned)health_mask,
                   APP_INDOOR_TEST_MODE ? " DEV" : "");
  if (count < 0 || (size_t)count >= out_size) return -2;
  return count;
}
