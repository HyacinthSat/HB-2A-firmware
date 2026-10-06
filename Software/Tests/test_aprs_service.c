#include "aprs_service.h"
#include "aprs_schedule_service.h"

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
static uint32_t fake_tick;
static uint32_t poll_count;
static uint32_t send_call_count;
static uint32_t fail_on_send_call;

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

int SX1278_APRS_SendWithContext(uint32_t frequency_hz, uint32_t budget_ms,
                                void (*service)(void *), void *context)
{
  (void)service;
  (void)context;
  ++send_call_count;
  prepared_frequency = frequency_hz;
  prepared_budget = budget_ms;
  if (service != NULL) service(context);
  if (fail_on_send_call != 0U && send_call_count == fail_on_send_call) {
    return -8;
  }
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
  fake_tick = 0U;
  poll_count = 0U;
  send_call_count = 0U;
  fail_on_send_call = 0U;
}

static uint32_t fake_now(void *context)
{
  (void)context;
  return fake_tick;
}

static void fake_sleep(uint32_t milliseconds, void *context)
{
  (void)context;
  fake_tick += milliseconds;
}

static void fake_poll(void *context)
{
  (void)context;
  ++poll_count;
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

  AprsScheduleService schedule;
  APRS_Utc utc = {1U, 2026U, 10U, 6U, 12U, 0U, 0U};
  aprs_schedule_service_init(&schedule);
  assert(aprs_schedule_service_slot_due(&utc, &schedule) == 0);
  utc.minute = 1U;
  assert(aprs_schedule_service_slot_due(&utc, &schedule) == 1);
  assert(aprs_schedule_service_slot_due(&utc, &schedule) == 1);
  utc.second = 2U;
  assert(aprs_schedule_service_slot_due(&utc, &schedule) == 1);
  utc.minute = 6U;
  utc.second = 0U;
  assert(aprs_schedule_service_slot_due(&utc, &schedule) == 1);
  assert(aprs_schedule_service_send_burst(
             &schedule, &service, &frame, 2U, 3000U,
             fake_now, fake_sleep, fake_poll, NULL) == 0);
  assert(schedule.burst_count == 1U);
  assert(service.tx_count == 3U);
  assert(fake_tick == 3000U && poll_count == 3002U);
  assert(aprs_schedule_service_slot_due(&utc, &schedule) == 0);

  AprsScheduleService retry_schedule;
  APRS_Utc retry_utc = {1U, 2026U, 10U, 6U, 13U, 1U, 0U};
  aprs_schedule_service_init(&retry_schedule);
  assert(aprs_schedule_service_slot_due(&retry_utc, &retry_schedule) == 1);
  send_result = -8;
  assert(aprs_schedule_service_send_burst(
             &retry_schedule, &service, &frame, 2U, 3000U,
             fake_now, fake_sleep, fake_poll, NULL) == -8);
  assert(retry_schedule.pending_valid == 1U);
  assert(aprs_schedule_service_slot_due(&retry_utc, &retry_schedule) == 1);
  send_result = 0;
  assert(aprs_schedule_service_send_burst(
             &retry_schedule, &service, &frame, 2U, 3000U,
             fake_now, fake_sleep, fake_poll, NULL) == 0);
  assert(retry_schedule.pending_valid == 0U);

  AprsScheduleService partial_schedule;
  APRS_Utc partial_utc = {1U, 2026U, 10U, 6U, 14U, 1U, 0U};
  aprs_schedule_service_init(&partial_schedule);
  assert(aprs_schedule_service_slot_due(&partial_utc, &partial_schedule) == 1);
  send_result = 0;
  fail_on_send_call = send_call_count + 2U;
  assert(aprs_schedule_service_send_burst(
             &partial_schedule, &service, &frame, 2U, 3000U,
             fake_now, fake_sleep, fake_poll, NULL) == -8);
  assert(partial_schedule.next_repeat == 1U);
  assert(partial_schedule.frame_valid == 1U);
  fail_on_send_call = 0U;
  assert(aprs_schedule_service_send_burst(
             &partial_schedule, &service, &frame, 2U, 3000U,
             fake_now, fake_sleep, fake_poll, NULL) == 0);
  assert(partial_schedule.next_repeat == 0U);
  assert(partial_schedule.frame_valid == 0U);

  AprsScheduleService delayed_schedule;
  APRS_Utc delayed_utc = {1U, 2026U, 10U, 6U, 12U, 50U, 0U};
  aprs_schedule_service_init(&delayed_schedule);
  assert(aprs_schedule_service_slot_due(&delayed_utc, &delayed_schedule) == 0);
  delayed_utc.minute = 57U;
  assert(aprs_schedule_service_slot_due(&delayed_utc, &delayed_schedule) == 1);

  AprsScheduleService startup_schedule;
  APRS_Utc startup_utc = {1U, 2026U, 10U, 6U, 12U, 56U, 30U};
  aprs_schedule_service_init(&startup_schedule);
  assert(aprs_schedule_service_slot_due(&startup_utc, &startup_schedule) == 1);

  AprsScheduleService post_wspr_schedule;
  APRS_Utc post_wspr_utc = {1U, 2026U, 10U, 6U, 12U, 57U, 50U};
  aprs_schedule_service_init(&post_wspr_schedule);
  assert(aprs_schedule_service_slot_due(&post_wspr_utc, &post_wspr_schedule) == 1);

  send_result = -8;
  assert(aprs_service_send_text(&service, 144800000UL, "X", 1U) == -8);
  assert(service.tx_count == 7U && service.last_error == -8);

  reset_mocks();
  init_result = -2;
  assert(aprs_service_init(&service) == -2);
  assert(service.initialized == 0U && service.version == 0x12U);
  puts("aprs_service tests passed");
  return 0;
}
