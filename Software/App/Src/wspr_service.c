#include "wspr_service.h"

#include "wspr_enc.h"

#include <stdio.h>
#include <string.h>

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
