#ifndef HB2A_APRS_SCHEDULE_SERVICE_H
#define HB2A_APRS_SCHEDULE_SERVICE_H

#include "aprs_service.h"

#include <stdint.h>

typedef uint32_t (*AprsScheduleNowFn)(void *context);
typedef void (*AprsScheduleSleepFn)(uint32_t milliseconds, void *context);
typedef void (*AprsSchedulePollFn)(void *context);

typedef struct {
  uint64_t last_slot;
  uint64_t pending_slot;
  uint8_t baseline_set;
  uint8_t pending_valid;
  uint8_t next_repeat;
  uint8_t frame_valid;
  TelemetryFrame pending_frame;
  uint32_t burst_count;
  int last_error;
} AprsScheduleService;

void aprs_schedule_service_init(AprsScheduleService *service);
int aprs_schedule_service_slot_due(const APRS_Utc *utc,
                                   AprsScheduleService *service);
int aprs_schedule_service_send_burst(AprsScheduleService *schedule,
                                     AprsService *radio,
                                     const TelemetryFrame *frame,
                                     uint8_t repeat_count,
                                     uint32_t gap_ms,
                                     AprsScheduleNowFn now,
                                     AprsScheduleSleepFn sleep,
                                     AprsSchedulePollFn poll,
                                     void *context);

#endif
