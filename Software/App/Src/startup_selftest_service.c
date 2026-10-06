#include "startup_selftest_service.h"

#include <stdio.h>
#include <string.h>

static StartupSelfTestDevice *device_for(StartupSelfTestReport *report,
                                         uint8_t device_number)
{
  if (report == NULL || device_number < 1U ||
      device_number > STARTUP_SELFTEST_DEVICE_COUNT) return NULL;
  return &report->devices[device_number - 1U];
}

void startup_selftest_report_init(StartupSelfTestReport *report,
                                  uint32_t reset_times)
{
  uint8_t number;
  if (report == NULL) return;
  memset(report, 0, sizeof(*report));
  report->reset_times = reset_times;
  for (number = 1U; number <= STARTUP_SELFTEST_DEVICE_COUNT; ++number)
    startup_selftest_report_ok(report, number);
}

void startup_selftest_report_ok(StartupSelfTestReport *report,
                                uint8_t device_number)
{
  StartupSelfTestDevice *device = device_for(report, device_number);
  if (device == NULL) return;
  memset(device, 0, sizeof(*device));
  device->code_count = 1U;
  memcpy(device->codes[0], "0000", 4U);
  device->codes[0][4] = 0;
}

void startup_selftest_report_error(StartupSelfTestReport *report,
                                   uint8_t device_number,
                                   const char code[5])
{
  StartupSelfTestDevice *device = device_for(report, device_number);
  uint8_t i;
  if (device == NULL || code == NULL) return;
  if (device->code_count == 1U && strcmp(device->codes[0], "0000") == 0)
    device->code_count = 0U;
  for (i = 0U; i < device->code_count; ++i) {
    if (memcmp(device->codes[i], code, 4U) == 0) return;
  }
  if (device->code_count >= STARTUP_SELFTEST_CODE_CAPACITY) return;
  memcpy(device->codes[device->code_count], code, 4U);
  device->codes[device->code_count][4] = 0;
  ++device->code_count;
}

int startup_selftest_report_build(const StartupSelfTestReport *report,
                                  const char *call,
                                  const char *path,
                                  char *out,
                                  size_t capacity)
{
  size_t used = 0U;
  uint8_t number;
  int written;

  if (report == NULL || call == NULL || path == NULL || out == NULL ||
      capacity == 0U) return -1;
  written = snprintf(out, capacity, "%s>%s:%lu", call, path,
                     (unsigned long)report->reset_times);
  if (written < 0 || (size_t)written >= capacity) return -2;
  used = (size_t)written;
  for (number = 1U; number <= STARTUP_SELFTEST_DEVICE_COUNT; ++number) {
    const StartupSelfTestDevice *device = &report->devices[number - 1U];
    uint8_t i;
    written = snprintf(out + used, capacity - used,
                       number == 1U ? ",%02u" : ";%02u",
                       (unsigned)number);
    if (written < 0 || (size_t)written >= capacity - used) return -2;
    used += (size_t)written;
    for (i = 0U; i < device->code_count; ++i) {
      written = snprintf(out + used, capacity - used, ",%s",
                         device->codes[i]);
      if (written < 0 || (size_t)written >= capacity - used) return -2;
      used += (size_t)written;
    }
  }
  return (int)used;
}

int startup_selftest_run(const StartupSelfTestPlan *plan,
                         StartupSelfTestReport *report)
{
  uint8_t device_number;
  int send_error = 0;

  if (plan == NULL || report == NULL || plan->send_report == NULL) return -1;
  startup_selftest_report_init(report, plan->reset_times);
  for (device_number = 1U;
       device_number <= STARTUP_SELFTEST_DEVICE_COUNT;
       ++device_number) {
    StartupSelfTestAttemptFn attempt_fn = plan->attempt[device_number - 1U];
    uint8_t attempt;
    int result = -1;
    if (attempt_fn == NULL) return -2;
    for (attempt = 1U; attempt <= 3U; ++attempt) {
      result = attempt_fn(plan->context, attempt, report);
      if (result == 0) {
        startup_selftest_report_ok(report, device_number);
      }
      if (result == 0) break;
    }
    if (plan->delay != NULL &&
        device_number < STARTUP_SELFTEST_DEVICE_COUNT &&
        plan->device_gap_ms != 0U) {
      plan->delay(plan->device_gap_ms, plan->context);
    }
  }
  /* Build and send one complete report after every device has been tested.
     Intermediate reports could describe a still-unchecked device as healthy. */
  if (plan->send_report(report, plan->context) != 0) send_error = -3;
  return send_error;
}
