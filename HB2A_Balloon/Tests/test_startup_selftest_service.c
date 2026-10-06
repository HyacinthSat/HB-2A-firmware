#include "startup_selftest_service.h"

#include <assert.h>
#include <stdio.h>
#include <string.h>

static unsigned attempts[STARTUP_SELFTEST_DEVICE_COUNT];
static unsigned reports;
static unsigned delays;

static int attempt_common(uint8_t device, StartupSelfTestReport *report)
{
  ++attempts[device - 1U];
  if (device == 2U && attempts[device - 1U] == 1U) {
    startup_selftest_report_error(report, device, "SPIE");
    startup_selftest_report_error(report, device, "TXER");
    return -1;
  }
  return 0;
}

#define ATTEMPT_WRAPPER(number) \
  static int attempt_##number(void *context, uint8_t attempt, \
                              StartupSelfTestReport *report) { \
    (void)context; (void)attempt; return attempt_common(number, report); \
  }
ATTEMPT_WRAPPER(1U)
ATTEMPT_WRAPPER(2U)
ATTEMPT_WRAPPER(3U)
ATTEMPT_WRAPPER(4U)
ATTEMPT_WRAPPER(5U)
ATTEMPT_WRAPPER(6U)
ATTEMPT_WRAPPER(7U)
ATTEMPT_WRAPPER(8U)

static int send_report(const StartupSelfTestReport *report, void *context)
{
  (void)context;
  assert(report != NULL);
  ++reports;
  return 0;
}

static void delay_report(uint32_t milliseconds, void *context)
{
  (void)context;
  assert(milliseconds == 500U);
  ++delays;
}

static void test_report_format(void)
{
  StartupSelfTestReport report;
  char text[256];
  startup_selftest_report_init(&report, 7U);
  startup_selftest_report_error(&report, 2U, "SPIE");
  startup_selftest_report_error(&report, 2U, "TXER");
  assert(startup_selftest_report_build(&report, "BG7ZDQ-11", "APZDQ7",
                                       text, sizeof(text)) > 0);
  assert(strcmp(text,
                "BG7ZDQ-11>APZDQ7:7,01,0000;02,SPIE,TXER;03,0000;04,0000;"
                "05,0000;06,0000;07,0000;08,0000") == 0);
}

static void test_plan(void)
{
  StartupSelfTestPlan plan = {0};
  StartupSelfTestReport report;
  plan.attempt[0] = attempt_1U;
  plan.attempt[1] = attempt_2U;
  plan.attempt[2] = attempt_3U;
  plan.attempt[3] = attempt_4U;
  plan.attempt[4] = attempt_5U;
  plan.attempt[5] = attempt_6U;
  plan.attempt[6] = attempt_7U;
  plan.attempt[7] = attempt_8U;
  plan.send_report = send_report;
  plan.delay = delay_report;
  plan.reset_times = 3U;
  plan.device_gap_ms = 500U;
  assert(startup_selftest_run(&plan, &report) == 0);
  assert(reports == 1U);
  assert(delays == 7U);
  assert(attempts[1] == 2U);
  assert(strcmp(report.devices[1].codes[0], "0000") == 0);
}

int main(void)
{
  test_report_format();
  test_plan();
  puts("startup_selftest_service tests passed");
  return 0;
}
