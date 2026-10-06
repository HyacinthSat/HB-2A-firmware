#ifndef HB2A_APRS_SERVICE_H
#define HB2A_APRS_SERVICE_H

#include "telemetry_service.h"

#include <stddef.h>
#include <stdint.h>

typedef struct {
  uint8_t initialized;
  uint8_t version;
  uint32_t last_frequency_hz;
  uint32_t tx_count;
  int last_error;
} AprsService;

typedef void (*AprsServicePollFn)(void *context);

int aprs_service_init(AprsService *service);
int aprs_service_send_text(AprsService *service, uint32_t frequency_hz,
                           const char *text, size_t length);
int aprs_service_send_text_with_poll(AprsService *service,
                                     uint32_t frequency_hz,
                                     const char *text, size_t length,
                                     AprsServicePollFn poll,
                                     void *context);
int aprs_service_send_telemetry(AprsService *service,
                                const TelemetryFrame *frame);
int aprs_service_send_telemetry_with_poll(AprsService *service,
                                          const TelemetryFrame *frame,
                                          AprsServicePollFn poll,
                                          void *context);

#endif
