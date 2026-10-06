#ifndef HB2A_STARTUP_SELFTEST_SERVICE_H
#define HB2A_STARTUP_SELFTEST_SERVICE_H

#include <stddef.h>
#include <stdint.h>

#define STARTUP_SELFTEST_DEVICE_COUNT 8U
#define STARTUP_SELFTEST_CODE_CAPACITY 5U

typedef struct {
  uint8_t code_count;
  char codes[STARTUP_SELFTEST_CODE_CAPACITY][5];
} StartupSelfTestDevice;

typedef struct {
  uint32_t reset_times;
  StartupSelfTestDevice devices[STARTUP_SELFTEST_DEVICE_COUNT];
} StartupSelfTestReport;

typedef int (*StartupSelfTestAttemptFn)(void *context,
                                       uint8_t attempt,
                                       StartupSelfTestReport *report);
typedef int (*StartupSelfTestSendFn)(const StartupSelfTestReport *report,
                                     void *context);
typedef void (*StartupSelfTestDelayFn)(uint32_t milliseconds, void *context);

typedef struct {
  StartupSelfTestAttemptFn attempt[STARTUP_SELFTEST_DEVICE_COUNT];
  /* Called once after all devices have completed their attempts. */
  StartupSelfTestSendFn send_report;
  StartupSelfTestDelayFn delay;
  void *context;
  uint32_t reset_times;
  uint32_t device_gap_ms;
} StartupSelfTestPlan;

void startup_selftest_report_init(StartupSelfTestReport *report,
                                  uint32_t reset_times);
void startup_selftest_report_ok(StartupSelfTestReport *report,
                                uint8_t device_number);
void startup_selftest_report_error(StartupSelfTestReport *report,
                                   uint8_t device_number,
                                   const char code[5]);
int startup_selftest_report_build(const StartupSelfTestReport *report,
                                  const char *call,
                                  const char *path,
                                  char *out,
                                  size_t capacity);

int startup_selftest_run(const StartupSelfTestPlan *plan,
                         StartupSelfTestReport *report);

#endif
