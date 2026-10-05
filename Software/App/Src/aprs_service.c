#include "aprs_service.h"

#include "sx1278_aprs.h"

#include <string.h>

int aprs_service_init(AprsService *service)
{
  uint8_t version = 0U;
  int result;
  if (service == NULL) return -1;
  memset(service, 0, sizeof(*service));
  result = SX1278_APRS_Init(&version);
  service->version = version;
  service->last_error = result;
  service->initialized = result == 0 ? 1U : 0U;
  return result;
}

int aprs_service_send_text(AprsService *service, uint32_t frequency_hz,
                           const char *text, size_t length)
{
  int duration;
  int result;
  if (service == NULL || service->initialized == 0U ||
      text == NULL || length == 0U) return -1;
  duration = SX1278_APRS_Prepare(text, length);
  if (duration < 0) {
    service->last_error = duration;
    return duration;
  }
  result = SX1278_APRS_Send(frequency_hz, (uint32_t)duration + 300U, NULL);
  service->last_error = result;
  if (result == 0) {
    service->last_frequency_hz = frequency_hz;
    ++service->tx_count;
  }
  return result;
}

int aprs_service_send_telemetry(AprsService *service,
                                const TelemetryFrame *frame)
{
  const char *separator;
  if (frame == NULL) return -1;
  separator = strchr(frame->aprs_text, ':');
  if (separator == NULL || separator[1] == 0) return -2;
  return aprs_service_send_text(service, frame->aprs_frequency_hz,
                                frame->aprs_text + (separator - frame->aprs_text) + 1U,
                                strlen(separator + 1U));
}
