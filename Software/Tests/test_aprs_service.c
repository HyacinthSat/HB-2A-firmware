#include "aprs_service.h"

#include <assert.h>
#include <stdio.h>
#include <string.h>

static int init_result;
static int prepare_result;
static int send_result;
static uint8_t mock_version;
static uint32_t prepared_frequency;
static uint32_t prepared_budget;
static char prepared_text[128];

int SX1278_APRS_Init(uint8_t *version)
{
  if (version != NULL) *version = mock_version;
  return init_result;
}

int SX1278_APRS_Prepare(const char *info, size_t length)
{
  assert(info != NULL && length < sizeof(prepared_text));
  memcpy(prepared_text, info, length);
  prepared_text[length] = 0;
  return prepare_result;
}

int SX1278_APRS_Send(uint32_t frequency_hz, uint32_t budget_ms,
                     void (*service)(void))
{
  (void)service;
  prepared_frequency = frequency_hz;
  prepared_budget = budget_ms;
  return send_result;
}

static void reset_mocks(void)
{
  init_result = 0;
  prepare_result = 1200;
  send_result = 0;
  mock_version = 0x12U;
  prepared_frequency = 0U;
  prepared_budget = 0U;
  memset(prepared_text, 0, sizeof(prepared_text));
}

int main(void)
{
  AprsService service;
  TelemetryFrame frame;

  reset_mocks();
  assert(aprs_service_init(&service) == 0);
  assert(service.initialized == 1U && service.version == 0x12U);
  strcpy(frame.aprs_text, "BG7ZDQ-11>APZDQ7:TEST");
  frame.aprs_frequency_hz = 144640000UL;
  assert(aprs_service_send_telemetry(&service, &frame) == 0);
  assert(service.tx_count == 1U);
  assert(service.last_frequency_hz == 144640000UL);
  assert(prepared_frequency == 144640000UL && prepared_budget == 1500U);
  assert(strcmp(prepared_text, "TEST") == 0);

  send_result = -8;
  assert(aprs_service_send_text(&service, 144800000UL, "X", 1U) == -8);
  assert(service.tx_count == 1U && service.last_error == -8);

  reset_mocks();
  init_result = -2;
  assert(aprs_service_init(&service) == -2);
  assert(service.initialized == 0U && service.version == 0x12U);
  puts("aprs_service tests passed");
  return 0;
}
