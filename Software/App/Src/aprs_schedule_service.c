#include "aprs_schedule_service.h"

#include <stddef.h>
#include <string.h>

static uint64_t utc_minute_key(const APRS_Utc *utc)
{
  return ((uint64_t)utc->year << 40U) |
         ((uint64_t)utc->month << 32U) |
         ((uint64_t)utc->day << 24U) |
         ((uint64_t)utc->hour << 16U) |
         ((uint64_t)utc->minute << 8U);
}

static uint8_t leap_year(uint16_t year)
{
  return (uint8_t)(((year % 4U) == 0U && (year % 100U) != 0U) ||
                   (year % 400U) == 0U);
}

static uint8_t days_in_month(uint16_t year, uint8_t month)
{
  static const uint8_t days[] =
      {31U, 28U, 31U, 30U, 31U, 30U, 31U, 31U, 30U, 31U, 30U, 31U};
  if (month == 2U && leap_year(year) != 0U) return 29U;
  return days[month - 1U];
}

static void decrement_minute(APRS_Utc *utc)
{
  if (utc->minute != 0U) {
    --utc->minute;
    return;
  }
  utc->minute = 59U;
  if (utc->hour != 0U) {
    --utc->hour;
    return;
  }
  utc->hour = 23U;
  if (utc->day > 1U) {
    --utc->day;
    return;
  }
  if (utc->month > 1U) {
    --utc->month;
  } else {
    utc->month = 12U;
    if (utc->year > 0U) --utc->year;
  }
  utc->day = days_in_month(utc->year, utc->month);
}

static uint8_t most_recent_slot(const APRS_Utc *utc, APRS_Utc *slot)
{
  uint8_t offset;

  *slot = *utc;
  for (offset = 0U; offset < 10U; ++offset) {
    if ((*slot).minute % 10U == 1U || (*slot).minute % 10U == 6U) {
      return 1U;
    }
    decrement_minute(slot);
  }
  return 0U;
}

static uint8_t slot_age_minutes(const APRS_Utc *utc, const APRS_Utc *slot)
{
  APRS_Utc cursor = *utc;
  uint8_t age;

  for (age = 0U; age < 10U; ++age) {
    if (utc_minute_key(&cursor) == utc_minute_key(slot)) return age;
    decrement_minute(&cursor);
  }
  return 10U;
}

void aprs_schedule_service_init(AprsScheduleService *service)
{
  if (service == NULL) return;
  memset(service, 0, sizeof(*service));
  service->last_slot = UINT64_MAX;
}

int aprs_schedule_service_slot_due(const APRS_Utc *utc,
                                   AprsScheduleService *service)
{
  APRS_Utc slot;
  uint64_t key;

  if (utc == NULL || service == NULL || utc->valid == 0U ||
      utc->month < 1U || utc->month > 12U || utc->day < 1U ||
      utc->day > 31U || utc->hour > 23U || utc->minute > 59U) return 0;
  if (most_recent_slot(utc, &slot) == 0U) return 0;
  key = utc_minute_key(&slot);
  if (service->baseline_set == 0U) {
    service->baseline_set = 1U;
    if (slot_age_minutes(utc, &slot) > 1U) {
      service->last_slot = key;
      return 0;
    }
  }
  if (service->pending_valid != 0U) {
    if (service->pending_slot != key) {
      service->pending_slot = key;
      service->next_repeat = 0U;
      service->frame_valid = 0U;
    }
    return 1;
  }
  if (key == service->last_slot) return 0;
  service->pending_slot = key;
  service->pending_valid = 1U;
  return 1;
}

int aprs_schedule_service_send_burst(AprsScheduleService *schedule,
                                     AprsService *radio,
                                     const TelemetryFrame *frame,
                                     uint8_t repeat_count,
                                     uint32_t gap_ms,
                                     AprsScheduleNowFn now,
                                     AprsScheduleSleepFn sleep,
                                     AprsSchedulePollFn poll,
                                     void *context)
{
  uint8_t repeat;
  const TelemetryFrame *tx_frame;
  int result;

  if (schedule == NULL || radio == NULL || frame == NULL ||
      repeat_count == 0U || now == NULL || sleep == NULL) return -1;
  if (schedule->next_repeat >= repeat_count) schedule->next_repeat = 0U;
  if (schedule->frame_valid == 0U) {
    schedule->pending_frame = *frame;
    schedule->frame_valid = 1U;
  }
  tx_frame = &schedule->pending_frame;
  for (repeat = schedule->next_repeat; repeat < repeat_count; ++repeat) {
    result = aprs_service_send_telemetry_with_poll(radio, tx_frame, poll, context);
    if (result != 0) {
      schedule->next_repeat = repeat;
      schedule->last_error = result;
      return result;
    }
    if ((repeat + 1U) < repeat_count) {
      uint32_t start = now(context);
      while ((uint32_t)(now(context) - start) < gap_ms) {
        if (poll != NULL) poll(context);
        sleep(1U, context);
      }
    }
  }
  ++schedule->burst_count;
  if (schedule->pending_valid != 0U) {
    schedule->last_slot = schedule->pending_slot;
    schedule->pending_valid = 0U;
  }
  schedule->next_repeat = 0U;
  schedule->frame_valid = 0U;
  schedule->last_error = 0;
  return 0;
}
