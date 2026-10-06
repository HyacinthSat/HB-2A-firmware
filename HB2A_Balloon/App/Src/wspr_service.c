#include "wspr_service.h"

#include "app_config.h"
#include "wspr_enc.h"

#include <stdio.h>
#include <string.h>

static void grid4_from_position(const TelemetryFrame *telemetry, char grid[5])
{
  int64_t latitude;
  int64_t longitude;

  if (telemetry == NULL || telemetry->position_valid == 0U) {
    (void)strcpy(grid, APP_WSPR_DEFAULT_GRID);
    return;
  }
  latitude = (int64_t)telemetry->wspr_sample.latitude_e7 + 900000000LL;
  longitude = (int64_t)telemetry->wspr_sample.longitude_e7 + 1800000000LL;
  if (latitude < 0 || latitude >= 1800000000LL || longitude < 0 ||
      longitude >= 3600000000LL) {
    (void)strcpy(grid, APP_WSPR_DEFAULT_GRID);
    return;
  }
  grid[0] = (char)('A' + longitude / 200000000LL);
  grid[1] = (char)('A' + latitude / 100000000LL);
  grid[2] = (char)('0' + (longitude % 200000000LL) / 20000000LL);
  grid[3] = (char)('0' + (latitude % 100000000LL) / 10000000LL);
  grid[4] = 0;
}

int wspr_service_build(const TelemetryFrame *telemetry, WsprFrame *out)
{
  char power[4];
  int count;
  if (telemetry == NULL || out == NULL) return -1;
  memset(out, 0, sizeof(*out));
  if (telemetry->wspr_ready == 0U) {
    out->last_error = -2;
    return -2;
  }
  memcpy(out->callsign, telemetry->wspr_message.callsign,
         sizeof(out->callsign));
  memcpy(out->grid, telemetry->wspr_message.grid4, sizeof(out->grid));
  out->power_dbm = telemetry->wspr_message.power_dbm;
  (void)snprintf(power, sizeof(power), "%u", (unsigned)out->power_dbm);
  if (wspr_enc(out->callsign, out->grid, power, out->symbols) == 0U) {
    out->last_error = -3;
    return -3;
  }
  count = snprintf(out->text, sizeof(out->text), "%s %s %udBm",
                   out->callsign, out->grid, (unsigned)out->power_dbm);
  if (count < 0 || (size_t)count >= sizeof(out->text)) {
    out->last_error = -4;
    return -4;
  }
  out->ready = 1U;
  out->last_error = 0;
  return 0;
}

int wspr_service_build_standard(const TelemetryFrame *telemetry,
                                WsprFrame *out)
{
  char power[4];
  int count;

  if (out == NULL) return -1;
  memset(out, 0, sizeof(*out));
  memcpy(out->callsign, APP_WSPR_CALL, sizeof(out->callsign));
  grid4_from_position(telemetry, out->grid);
  out->power_dbm = APP_WSPR_DEFAULT_POWER_DBM;
  (void)snprintf(power, sizeof(power), "%u", (unsigned)out->power_dbm);
  if (wspr_enc(out->callsign, out->grid, power, out->symbols) == 0U) {
    out->last_error = -2;
    return -2;
  }
  count = snprintf(out->text, sizeof(out->text), "%s %s %udBm",
                   out->callsign, out->grid, (unsigned)out->power_dbm);
  if (count < 0 || (size_t)count >= sizeof(out->text)) {
    out->last_error = -3;
    return -3;
  }
  out->ready = 1U;
  out->last_error = 0;
  return 0;
}
