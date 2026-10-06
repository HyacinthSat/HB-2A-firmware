#ifndef HB2A_WSPR_RADIO_SERVICE_H
#define HB2A_WSPR_RADIO_SERVICE_H

#include "wspr_service.h"

#include <stdint.h>

typedef void (*WsprRadioPollFn)(void *context);

typedef struct {
  uint8_t initialized;
  uint8_t transmitting;
  uint32_t center_frequency_hz;
  uint32_t tx_count;
  int last_error;
} WsprRadioService;

int wspr_radio_service_init(WsprRadioService *service,
                            uint32_t center_frequency_hz);
int wspr_radio_service_send(WsprRadioService *service,
                            const WsprFrame *frame,
                            WsprRadioPollFn poll,
                            void *poll_context);
int wspr_radio_service_carrier_test(WsprRadioService *service,
                                     uint32_t frequency_hz,
                                     uint32_t duration_ms);

#endif
