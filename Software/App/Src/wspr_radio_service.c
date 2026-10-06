#include "wspr_radio_service.h"

#include "i2c.h"
#include "si5351.h"

#include "main.h"

#include <string.h>

#define SI5351_I2C_ADDRESS 0x60U
#define WSPR_SYMBOL_RATE_NUM 12000ULL
#define WSPR_SYMBOL_RATE_DEN 8192ULL
#define WSPR_TONE_STEP_NUM 12000ULL
#define WSPR_TONE_STEP_DEN 8192ULL

static uint32_t symbol_deadline(uint32_t start, uint32_t symbol)
{
  uint64_t elapsed_ms = ((uint64_t)(symbol + 1U) *
                         WSPR_SYMBOL_RATE_DEN * 1000ULL) /
                        WSPR_SYMBOL_RATE_NUM;
  return start + (uint32_t)elapsed_ms;
}

static int32_t symbol_frequency(uint32_t center_frequency_hz, uint8_t symbol)
{
  uint64_t offset = ((uint64_t)symbol * WSPR_TONE_STEP_NUM +
                     (WSPR_TONE_STEP_DEN / 2ULL)) /
                    WSPR_TONE_STEP_DEN;
  return (int32_t)((int64_t)center_frequency_hz + (int64_t)offset);
}

int wspr_radio_service_init(WsprRadioService *service,
                            uint32_t center_frequency_hz)
{
  if (service == NULL || center_frequency_hz == 0U) return -1;
  memset(service, 0, sizeof(*service));
  service->center_frequency_hz = center_frequency_hz;
  if (HAL_I2C_IsDeviceReady(&hi2c3, (uint16_t)(SI5351_I2C_ADDRESS << 1),
                            3U, 100U) != HAL_OK) {
    service->last_error = -2;
    return -2;
  }
  si5351_Init(0);
  si5351_EnableOutputs(0U);
  service->initialized = 1U;
  service->last_error = 0;
  return 0;
}

int wspr_radio_service_send(WsprRadioService *service,
                            const WsprFrame *frame,
                            WsprRadioPollFn poll,
                            void *poll_context)
{
  uint32_t start;
  uint32_t i;

  if (service == NULL || frame == NULL || service->initialized == 0U ||
      frame->ready == 0U) return -1;
  if (service->transmitting != 0U) return -2;

  service->transmitting = 1U;
  start = HAL_GetTick();
  for (i = 0U; i < WSPR_SERVICE_SYMBOL_COUNT; ++i) {
    uint32_t target = symbol_deadline(start, i);
    int32_t frequency = symbol_frequency(service->center_frequency_hz,
                                         frame->symbols[i]);
    si5351_SetupCLK0(frequency, SI5351_DRIVE_STRENGTH_8MA);
    if (i == 0U) si5351_EnableOutputs(1U);
    while ((int32_t)(HAL_GetTick() - target) < 0) {
      if (poll != NULL) poll(poll_context);
      HAL_Delay(1U);
    }
  }
  si5351_EnableOutputs(0U);
  service->transmitting = 0U;
  ++service->tx_count;
  service->last_error = 0;
  return 0;
}

int wspr_radio_service_carrier_test(WsprRadioService *service,
                                    uint32_t frequency_hz,
                                    uint32_t duration_ms)
{
  if (service == NULL || service->initialized == 0U ||
      service->transmitting != 0U || frequency_hz == 0U ||
      duration_ms == 0U || duration_ms > 5000U) return -1;
  service->transmitting = 1U;
  si5351_SetupCLK0((int32_t)frequency_hz, SI5351_DRIVE_STRENGTH_8MA);
  si5351_EnableOutputs(1U);
  HAL_Delay(duration_ms);
  si5351_EnableOutputs(0U);
  service->transmitting = 0U;
  service->last_error = 0;
  return 0;
}
