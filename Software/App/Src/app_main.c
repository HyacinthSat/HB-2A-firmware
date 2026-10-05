#include "app_main.h"

#include "app_config.h"
#include "app_flow.h"
#include "aprs_service.h"
#include "gnss_service.h"
#include "i2c.h"
#include "main.h"
#include "sensors.h"
#include "spi.h"
#include "telemetry_service.h"
#include "usart.h"
#include "wspr_radio_service.h"
#include "wspr_service.h"

#include <stdio.h>
#include <string.h>

static void app_debug(const char *text)
{
  if (text == 0) return;
  (void)HAL_UART_Transmit(&huart4, (uint8_t *)text,
                         (uint16_t)strlen(text), 1000U);
}

static void gnss_reset_pulse(void)
{
  HAL_GPIO_WritePin(GNSS_RST_GPIO_Port, GNSS_RST_Pin, GPIO_PIN_RESET);
  HAL_Delay(20U);
  HAL_GPIO_WritePin(GNSS_RST_GPIO_Port, GNSS_RST_Pin, GPIO_PIN_SET);
  HAL_Delay(100U);
}

static int gnss_startup(GnssService *service)
{
  for (uint8_t attempt = 1U; attempt <= 3U; ++attempt) {
    if (attempt != 1U) gnss_reset_pulse();
    if (gnss_service_init(service, &huart2) != 0) {
      app_debug("GNSS RX init failed\r\n");
      continue;
    }
    app_debug("GNSS configuring M10\r\n");
    if (gnss_service_configure(service) != 0) {
      app_debug("GNSS configuration failed\r\n");
      continue;
    }
    app_debug("GNSS waiting for 3D fix + UTC\r\n");
    if (gnss_service_wait_for_lock(service, 300000U) == 0) {
      app_debug("GNSS lock accepted\r\n");
      return 0;
    }
    app_debug("GNSS five-minute timeout\r\n");
  }
  gnss_reset_pulse();
  return -1;
}

static uint64_t gnss_minute_key(const GnssSolution *solution)
{
  return ((uint64_t)solution->year << 40U) |
         ((uint64_t)solution->month << 32U) |
         ((uint64_t)solution->day << 24U) |
         ((uint64_t)solution->hour << 16U) |
         ((uint64_t)solution->minute << 8U);
}

static uint8_t wspr_slot_due(const GnssSolution *solution,
                             uint8_t *telemetry,
                             uint8_t *first_pending,
                             uint64_t *last_slot)
{
  uint64_t key;
  uint8_t minute;

  if (solution == NULL || telemetry == NULL || first_pending == NULL ||
      last_slot == NULL || solution->position_valid == 0U ||
      solution->utc_valid == 0U || solution->fully_resolved == 0U ||
      solution->second > 1U) return 0U;
  minute = solution->minute;
  if (*first_pending != 0U) {
    if ((minute & 1U) != 0U) return 0U;
    *telemetry = 0U;
  } else if ((minute % 10U) == APP_WSPR_REGULAR_MINUTE) {
    *telemetry = 0U;
  } else if ((minute % 10U) == APP_WSPR_TELEMETRY_MINUTE) {
    *telemetry = 1U;
  } else {
    return 0U;
  }
  key = gnss_minute_key(solution);
  if (key == *last_slot) return 0U;
  *last_slot = key;
  *first_pending = 0U;
  return 1U;
}

static void wspr_radio_gnss_poll(void *context)
{
  gnss_service_poll((GnssService *)context);
}

static int telemetry_report(const GnssService *gnss, TelemetryFrame *reported)
{
  const GnssSolution *solution;
  TelemetryGnssSnapshot snapshot;
  SensorSample sensors;
  TelemetryFrame frame;
  WsprFrame wspr;
  char debug_line[TELEMETRY_APRS_TEXT_CAPACITY + 64U];
  uint8_t health_mask;

  if (gnss == 0) return -1;
  solution = gnss_service_solution(gnss);
  if (solution == 0) return -1;
  memset(&snapshot, 0, sizeof(snapshot));
  snapshot.position_valid = solution->position_valid;
  snapshot.utc_valid = solution->utc_valid;
  snapshot.fully_resolved = solution->fully_resolved;
  snapshot.latitude_e7 = solution->latitude_e7;
  snapshot.longitude_e7 = solution->longitude_e7;
  snapshot.height_msl_mm = solution->height_msl_mm;
  snapshot.ground_speed_mm_s = solution->ground_speed_mm_s;
  snapshot.year = solution->year;
  snapshot.month = solution->month;
  snapshot.day = solution->day;
  snapshot.hour = solution->hour;
  snapshot.minute = solution->minute;
  snapshot.second = solution->second;
  Sensors_Get(&sensors);
  health_mask = 0U;
  if ((sensors.valid & SENSOR_RAILS_VALID) != 0U) health_mask |= (uint8_t)(1U << 2);
  if (snapshot.position_valid != 0U) health_mask |= (uint8_t)(1U << 4);
  if (telemetry_service_build(&snapshot, &sensors, health_mask, &frame) != 0) {
    app_debug("NORMAL telemetry build failed\r\n");
    return -2;
  }
  if (reported != 0) *reported = frame;
  (void)snprintf(debug_line, sizeof(debug_line),
                 "NORMAL APRS telemetry: %s\r\n", frame.aprs_text);
  app_debug(debug_line);
  if (wspr_service_build(&frame, &wspr) == 0) {
    (void)snprintf(debug_line, sizeof(debug_line),
                   "NORMAL WSPR telemetry frame: %s\r\n", wspr.text);
    app_debug(debug_line);
  } else {
    app_debug("NORMAL WSPR telemetry waiting for valid sensors\r\n");
  }
  return 0;
}

void app_main(void)
{
  AppFlow flow;
  GnssService gnss;
  AprsService aprs;
  WsprRadioService wspr_radio;
  TelemetryFrame telemetry;
  char debug_line[WSPR_SERVICE_TEXT_CAPACITY + 64U];
  uint8_t wspr_first_pending = 1U;
  uint64_t wspr_last_slot = UINT64_MAX;

  app_flow_init(&flow);
  (void)app_flow_dispatch(&flow, APP_EVENT_BOOT_CHECK_DONE);
  app_debug("HB-2A formal software: boot -> GNSS_WAIT\r\n");

  MX_USART2_UART_Init();
  if (gnss_startup(&gnss) != 0) {
    (void)app_flow_dispatch(&flow, APP_EVENT_GNSS_TIMEOUT);
    app_debug("GNSS failed three attempts; restarting MCU\r\n");
    NVIC_SystemReset();
    for (;;) { }
  }
  (void)app_flow_dispatch(&flow, APP_EVENT_GNSS_LOCKED);
  app_debug("HB-2A formal software: GNSS_WAIT -> NORMAL\r\n");
  MX_I2C3_Init();
  Sensors_Init();
  app_debug("NORMAL sensors initialized\r\n");
  if (wspr_radio_service_init(&wspr_radio, APP_WSPR_CENTER_HZ) == 0) {
    app_debug("NORMAL WSPR SI5351 initialized\r\n");
  } else {
    app_debug("NORMAL WSPR SI5351 unavailable\r\n");
  }
  if (telemetry_report(&gnss, &telemetry) == 0) {
    MX_SPI2_Init();
    if (aprs_service_init(&aprs) == 0) {
      app_debug("NORMAL APRS SX1278 initialized\r\n");
      if (aprs_service_send_telemetry(&aprs, &telemetry) == 0) {
        app_debug("NORMAL APRS telemetry TX complete\r\n");
      } else {
        app_debug("NORMAL APRS telemetry TX failed\r\n");
      }
    } else {
      app_debug("NORMAL APRS SX1278 initialization failed\r\n");
    }
  } else {
    app_debug("NORMAL APRS telemetry unavailable\r\n");
  }

  uint32_t next_telemetry_report = HAL_GetTick() + 60000U;
  for (;;) {
    uint8_t wspr_telemetry_slot;
    gnss_service_poll(&gnss);
    Sensors_Poll();
    if ((int32_t)(HAL_GetTick() - next_telemetry_report) >= 0) {
      (void)telemetry_report(&gnss, 0);
      next_telemetry_report += 60000U;
    }
    if (wspr_radio.initialized != 0U &&
        wspr_slot_due(gnss_service_solution(&gnss), &wspr_telemetry_slot,
                      &wspr_first_pending, &wspr_last_slot) != 0U) {
      WsprFrame frame;
      int build_result;
      if (wspr_telemetry_slot != 0U) {
        build_result = telemetry_report(&gnss, &telemetry);
        if (build_result == 0) {
          build_result = wspr_service_build(&telemetry, &frame);
        }
      } else {
        build_result = wspr_service_build_standard(&frame);
      }
      if (build_result != 0) {
        app_debug("NORMAL WSPR frame build failed\r\n");
      } else {
        (void)snprintf(debug_line, sizeof(debug_line),
                       "NORMAL WSPR TX start: %s\r\n", frame.text);
        app_debug(debug_line);
        if (wspr_radio_service_send(&wspr_radio, &frame,
                                    wspr_radio_gnss_poll, &gnss) == 0) {
          app_debug("NORMAL WSPR TX complete\r\n");
        } else {
          app_debug("NORMAL WSPR TX failed\r\n");
        }
      }
    }
    HAL_Delay(10U);
  }
}

