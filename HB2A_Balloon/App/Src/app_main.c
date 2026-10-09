#include "app_main.h"

#include "app_config.h"
#include "app_flow.h"
#include "aprs_schedule_service.h"
#include "aprs_service.h"
#include "camera.h"
#include "camera_ssdv_service.h"
#include "flash_selftest.h"
#include "gnss_service.h"
#include "gmsk_downlink.h"
#include "high_speed_ssdv.h"
#include "high_speed_service.h"
#include "i2c.h"
#include "image_storage.h"
#include "main.h"
#include "sensors.h"
#include "spi.h"
#include "startup_selftest_service.h"
#include "sx1278_aprs.h"
#include "transponder_service.h"
#include "telemetry_service.h"
#include "usart.h"
#include "wspr_radio_service.h"
#include "wspr_service.h"

#include <stdio.h>
#include <string.h>

/* The target linker reserves 0x1000 bytes for the main stack. Keep the
   protocol frames and APRS retry state out of that stack. The application is
   single-threaded, so these workspaces are not concurrently accessed. */
static AprsScheduleService g_aprs_schedule;
static TelemetryFrame g_telemetry;
static char g_debug_line[WSPR_SERVICE_TEXT_CAPACITY + 64U];
static TelemetryGnssSnapshot g_telemetry_snapshot;
static SensorSample g_telemetry_sensors;
static TelemetryFrame g_telemetry_frame;
static WsprFrame g_telemetry_wspr;
static char g_telemetry_debug_line[TELEMETRY_APRS_TEXT_CAPACITY + 64U];
static WsprFrame g_wspr_frame;
static char g_selftest_line[512];
static char g_radio_diagnostic[768];
static TransponderService g_transponder;
static HighSpeedService g_high_speed;
static uint8_t g_gmsk_bits[GMSK_DOWNLINK_MAX_PACKED_BYTES];
static uint8_t g_high_speed_bits[GMSK_HIGH_SPEED_MAX_PACKED_BYTES];
static uint8_t g_high_speed_frames[GMSK_HIGH_SPEED_MAX_CODEWORDS *
                                   GMSK_HIGH_SPEED_DATA_BYTES];
static uint8_t g_high_speed_frame_count;

typedef struct {
  GnssService *gnss;
  AprsService *aprs;
  WsprRadioService *wspr;
} StartupSelfTestContext;
static StartupSelfTestReport g_selftest_report;
static StartupSelfTestContext g_selftest_context;
static StartupSelfTestPlan g_selftest_plan;
static CameraPort g_ssdv_next_camera = CAMERA_A;
static uint8_t g_ssdv_next_image_id;
static uint32_t g_ssdv_next_image_sequence;
static ImageStorage g_image_storage;
static uint64_t g_ssdv_last_group_slot = UINT64_MAX;

static void app_debug(const char *text)
{
  if (text == 0) return;
  (void)HAL_UART_Transmit(&huart4, (uint8_t *)text,
                         (uint16_t)strlen(text), 1000U);
}

static void wspr_radio_gnss_poll(void *context);

static int gmsk_send_text(const char *text, GnssService *gnss)
{
  size_t bit_count;
  int result, duration;
  if (text == NULL) return -1;
  result = gmsk_downlink_encode((const uint8_t *)text, strlen(text),
                                g_gmsk_bits, sizeof(g_gmsk_bits), &bit_count);
  if (result != 0) return result;
  duration = SX1278_APRS_PrepareGmsk(g_gmsk_bits, bit_count, 4800U, 1200U);
  if (duration < 0) return duration;
  app_debug("GMSK TX start freq=144925000 rate=4800: ");
  app_debug(text);
  app_debug("\r\n");
  result = SX1278_APRS_SendGmsk(APP_GMSK_DOWNLINK_FREQ_HZ,
                                (uint32_t)duration + 500U,
                                wspr_radio_gnss_poll, gnss);
  (void)snprintf(g_debug_line, sizeof(g_debug_line),
                 "GMSK TX result=%d bits=%lu duration=%dms RX=%u\r\n",
                 result, (unsigned long)bit_count, duration,
                 SX1278_APRS_RxActive());
  app_debug(g_debug_line);
  SX1278_APRS_TxReport(g_radio_diagnostic, sizeof(g_radio_diagnostic));
  app_debug(g_radio_diagnostic);
  return result;
}

static int high_speed_rate(uint8_t rate_id, uint32_t *bitrate,
                           uint32_t *deviation_hz)
{
  static const uint32_t rates[] = {4800U, 9600U, 19200U, 38400U,
                                   57600U, 115200U};
  static const uint32_t deviations[] = {1200U, 2400U, 4800U, 9600U,
                                        14400U, 28800U};
  if (bitrate == NULL || deviation_hz == NULL || rate_id >= 6U) return -1;
  *bitrate = rates[rate_id];
  *deviation_hz = deviations[rate_id];
  return 0;
}

static int high_speed_send_group(const uint8_t *frames, uint8_t frame_count,
                                 const HighSpeedParameters *parameters,
                                 GnssService *gnss)
{
  size_t bit_count;
  uint32_t bitrate;
  uint32_t deviation_hz;
  int duration;
  int result;
  if (frames == NULL || parameters == NULL || gnss == NULL ||
      high_speed_rate(parameters->rate_id, &bitrate, &deviation_hz) != 0)
    return -1;
  result = gmsk_high_speed_encode(frames, frame_count, parameters->code_id,
                                  parameters->intlv_id, g_high_speed_bits,
                                  sizeof(g_high_speed_bits), &bit_count);
  if (result != 0) return result;
  duration = SX1278_APRS_PrepareGmsk(g_high_speed_bits, bit_count, bitrate,
                                     deviation_hz);
  if (duration < 0) return duration;
  (void)snprintf(g_debug_line, sizeof(g_debug_line),
                 "HS SSDV TX group=%u rate=%u code=%u intlv=%u bits=%lu\r\n",
                 (unsigned)frame_count, (unsigned)parameters->rate_id,
                 (unsigned)parameters->code_id, (unsigned)parameters->intlv_id,
                 (unsigned long)bit_count);
  app_debug(g_debug_line);
  result = SX1278_APRS_SendGmsk(APP_GMSK_DOWNLINK_FREQ_HZ,
                                (uint32_t)duration + 500U,
                                wspr_radio_gnss_poll, gnss);
  if (result != 0) {
    SX1278_APRS_TxReport(g_radio_diagnostic, sizeof(g_radio_diagnostic));
    app_debug(g_radio_diagnostic);
  }
  return result;
}

typedef struct {
  const HighSpeedParameters *parameters;
  GnssService *gnss;
} HighSpeedSsdvSendContext;

static int high_speed_ssdv_packet(const uint8_t *packet, size_t length,
                                  void *context)
{
  HighSpeedSsdvSendContext *send_context = (HighSpeedSsdvSendContext *)context;
  size_t depth;
  int result;
  if (packet == NULL || send_context == NULL || send_context->parameters == NULL ||
      send_context->gnss == NULL || length < APP_SSDV_PACKET_SIZE) return -1;
  switch (send_context->parameters->intlv_id) {
    case 1U: depth = 1U; break;
    case 2U: depth = 2U; break;
    case 3U: depth = 3U; break;
    case 4U: depth = 4U; break;
    case 5U: depth = 5U; break;
    case 8U: depth = 8U; break;
    default: return -1;
  }
  if (g_high_speed_frame_count >= GMSK_HIGH_SPEED_MAX_CODEWORDS) return -2;
  if (high_speed_ssdv_build_frame(packet, length, APP_WSPR_CALL,
                                  g_high_speed_frames +
                                      g_high_speed_frame_count *
                                      GMSK_HIGH_SPEED_DATA_BYTES,
                                  GMSK_HIGH_SPEED_DATA_BYTES) != 0) return -3;
  ++g_high_speed_frame_count;
  if (g_high_speed_frame_count < depth && (packet[11] & 0x04U) == 0U)
    return 0;
  result = high_speed_send_group(g_high_speed_frames, g_high_speed_frame_count,
                                 send_context->parameters, send_context->gnss);
  g_high_speed_frame_count = 0U;
  return result;
}

static void high_speed_drain_events(AppFlow *flow, GnssService *gnss)
{
  HighSpeedEvent event;
  char text[64];
  while ((event = high_speed_service_take_event(&g_high_speed)) != HS_EVENT_NONE) {
    int length = high_speed_service_event_text(&g_high_speed, event, text,
                                               sizeof(text));
    if (length > 0) {
      if (gmsk_send_text(text, gnss) != 0) {
        app_debug("HS control TX failed; negotiation cancelled\r\n");
        high_speed_service_init(&g_high_speed);
        (void)app_flow_dispatch(flow, APP_EVENT_HS_CANCEL);
        return;
      }
      if (event == HS_EVENT_SEND_CONNECTION) {
        /* Measure the proposed modulation on air before accepting CFRM.
           HSCON stays on the robust 4800 control channel. */
        size_t depth = g_high_speed.parameters.intlv_id;
        (void)snprintf(text, sizeof(text), "HSTST,%u,%u,%u",
                       g_high_speed.parameters.rate_id,
                       g_high_speed.parameters.code_id,
                       g_high_speed.parameters.intlv_id);
        for (size_t i = 0U; i < depth; ++i) {
          uint8_t *frame = g_high_speed_frames + i * GMSK_HIGH_SPEED_DATA_BYTES;
          memset(frame, 0x20, GMSK_HIGH_SPEED_DATA_BYTES);
          frame[0] = 0U; frame[1] = 2U;
          frame[2] = (uint8_t)i; frame[3] = (uint8_t)i;
          memcpy(frame + 4U, text, strlen(text));
        }
        if (high_speed_send_group(g_high_speed_frames, (uint8_t)depth,
                                   &g_high_speed.parameters, gnss) != 0) {
          app_debug("HS probe TX failed; negotiation cancelled\r\n");
          high_speed_service_init(&g_high_speed);
          (void)app_flow_dispatch(flow, APP_EVENT_HS_CANCEL);
          return;
        }
      }
      high_speed_service_control_sent(&g_high_speed, event, HAL_GetTick());
    } else if (event == HS_EVENT_TIMEOUT) {
      app_debug("HS negotiation timeout; back to experiment RX\r\n");
      (void)app_flow_dispatch(flow, APP_EVENT_HS_CANCEL);
    } else if (event == HS_EVENT_START_TRANSFER) {
      HighSpeedSsdvSendContext context;
      uint32_t image_sequence;
      int result;
      (void)app_flow_dispatch(flow, APP_EVENT_TRANSFER_STARTED);
      SX1278_APRS_RxStop();
      high_speed_ssdv_reset_counters();
      g_high_speed_frame_count = 0U;
      context.parameters = &g_high_speed.parameters;
      context.gnss = gnss;
      if (g_high_speed.request == HS_REQUEST_CAPTURE) {
        image_sequence = g_ssdv_next_image_sequence;
        result = camera_ssdv_run_high_speed_capture(
            gnss, g_high_speed.camera == 'B' ? CAMERA_B : CAMERA_A,
            image_sequence, g_high_speed.width, g_high_speed.height,
            g_high_speed.quality, high_speed_ssdv_packet, &context, app_debug);
        /* A captured and stored image keeps its number even if RF transfer
           fails; REQF must be able to retry that image later. */
        if (g_image_storage.has_image != 0U &&
            g_image_storage.latest_sequence == image_sequence)
          ++g_ssdv_next_image_sequence;
      } else if (g_high_speed.request == HS_REQUEST_FILE &&
                 g_image_storage.has_image != 0U &&
                 g_high_speed.file_index <= g_image_storage.latest_sequence) {
        image_sequence = g_image_storage.latest_sequence - g_high_speed.file_index;
        if (image_storage_contains(&g_image_storage, image_sequence) != 0) {
          result = camera_ssdv_run_high_speed_file(
              gnss, &g_image_storage, image_sequence, APP_SSDV_QUALITY,
              high_speed_ssdv_packet, &context, app_debug);
        } else {
          result = -20;
        }
      } else {
        result = -20;
      }
      if (result != 0) {
        if (result == -20) {
          (void)gmsk_send_text("HSREJ,NOFILE", gnss);
        } else {
          /* Keep the stable HSREJ,TRANSFER prefix while exposing the
             camera/SSDV callback result for ground-side diagnosis. */
          char reject[32];
          (void)snprintf(reject, sizeof(reject), "HSREJ,TRANSFER,%d", result);
          (void)gmsk_send_text(reject, gnss);
        }
        high_speed_service_transfer_done(&g_high_speed, 1U);
        app_debug("HS SSDV transfer failed\r\n");
      } else {
        high_speed_service_transfer_done(&g_high_speed, 0U);
        (void)gmsk_send_text("HSDONE", gnss);
        app_debug("HS SSDV transfer complete\r\n");
      }
      (void)app_flow_dispatch(flow, result == 0 ? APP_EVENT_HS_COMPLETE :
                               APP_EVENT_HS_CANCEL);
      (void)SX1278_APRS_RxStart();
    } else if (event == HS_EVENT_TRANSFER_DONE) {
      (void)app_flow_dispatch(flow, APP_EVENT_HS_COMPLETE);
    } else if (event == HS_EVENT_CANCELLED) {
      (void)app_flow_dispatch(flow, APP_EVENT_HS_CANCEL);
    }
  }
}

static void gnss_reset_pulse(void)
{
  HAL_GPIO_WritePin(GNSS_RST_GPIO_Port, GNSS_RST_Pin, GPIO_PIN_RESET);
  HAL_Delay(20U);
  HAL_GPIO_WritePin(GNSS_RST_GPIO_Port, GNSS_RST_Pin, GPIO_PIN_SET);
  HAL_Delay(100U);
}

static uint32_t app_reset_times_next(void)
{
  uint32_t count;
  __HAL_RCC_PWR_CLK_ENABLE();
  HAL_PWR_EnableBkUpAccess();
  count = TAMP->BKP0R;
  if (count == 0xFFFFFFFFUL) count = 0U;
  TAMP->BKP0R = count + 1U;
  return count;
}

static int gnss_wait_after_selftest(GnssService *service)
{
  uint8_t attempt;
  if (service == NULL || gnss_service_state(service) != GNSS_SERVICE_CONFIGURED)
    return -1;
  for (attempt = 1U; attempt <= 3U; ++attempt) {
    if (attempt != 1U) {
      gnss_reset_pulse();
      if (gnss_service_init(service, &huart2) != 0 ||
          gnss_service_configure(service) != 0) continue;
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

static int selftest_sensors_ina(void *context, uint8_t attempt,
                                StartupSelfTestReport *report)
{
  SensorSample sample;
  (void)context;
  (void)attempt;
  Sensors_Init();
  Sensors_Get(&sample);
  if (sample.ina_status != 0) {
    startup_selftest_report_error(report, 1U, "I2CE");
    return -1;
  }
  if ((sample.valid & SENSOR_RAILS_VALID) == 0U) {
    startup_selftest_report_error(report, 1U, "READ");
    return -1;
  }
  return 0;
}

static int selftest_sx1278(void *context, uint8_t attempt,
                           StartupSelfTestReport *report)
{
  StartupSelfTestContext *selftest = (StartupSelfTestContext *)context;
  int result;
  (void)attempt;
  if (selftest == NULL || selftest->aprs == NULL) {
    startup_selftest_report_error(report, 2U, "SPIE");
    return -1;
  }
  if (attempt != 1U || selftest->aprs->initialized == 0U) {
    if (aprs_service_init(selftest->aprs) != 0) {
      startup_selftest_report_error(report, 2U, "SPIE");
      return -1;
    }
  }
  (void)snprintf(g_radio_diagnostic, sizeof(g_radio_diagnostic),
                 "SELFTEST SX1278 carrier start freq=144640000 power=%udBm duration=1000ms\r\n",
                 (unsigned)APP_SX1278_SELFTEST_POWER_DBM);
  app_debug(g_radio_diagnostic);
  result = SX1278_APRS_SetPowerDbm(APP_SX1278_SELFTEST_POWER_DBM);
  if (result == 0) result = SX1278_APRS_CarrierTest(144640000UL, 1000U, NULL);
  (void)snprintf(g_radio_diagnostic, sizeof(g_radio_diagnostic),
                 "SELFTEST SX1278 carrier result rc=%d attempt=%u\r\n",
                 result, (unsigned)attempt);
  app_debug(g_radio_diagnostic);
  SX1278_APRS_TxReport(g_radio_diagnostic, sizeof(g_radio_diagnostic));
  app_debug(g_radio_diagnostic);
  if (result != 0) {
    SX1278_APRS_Diagnose(g_radio_diagnostic, sizeof(g_radio_diagnostic));
    app_debug(g_radio_diagnostic);
  }
  (void)SX1278_APRS_SetPowerDbm(APP_APRS_POWER_DBM);
  if (result != 0) {
    startup_selftest_report_error(report, 2U,
                                   result == -2 ? "TIME" : "TXER");
    return -1;
  }
  return 0;
}

static int selftest_si5351(void *context, uint8_t attempt,
                           StartupSelfTestReport *report)
{
  StartupSelfTestContext *selftest = (StartupSelfTestContext *)context;
  (void)attempt;
  if (selftest == NULL || selftest->wspr == NULL) {
    startup_selftest_report_error(report, 3U, "I2CE");
    return -1;
  }
  if (attempt != 1U || selftest->wspr->initialized == 0U) {
    if (wspr_radio_service_init(selftest->wspr, APP_WSPR_CENTER_HZ) != 0) {
      startup_selftest_report_error(report, 3U, "I2CE");
      return -1;
    }
  }
  if (wspr_radio_service_carrier_test(selftest->wspr, 14095600UL, 1000U) != 0) {
    startup_selftest_report_error(report, 3U, "CONF");
    return -1;
  }
  return 0;
}

static int selftest_flash(void *context, uint8_t attempt,
                          StartupSelfTestReport *report)
{
  int result;
  const char *code;
  (void)context;
  (void)attempt;
  result = flash_selftest_run(FLASH_SELFTEST_ADDRESS);
  if (result == 0) return 0;
  code = result == FLASH_SELFTEST_TIMEOUT ? "TIME" :
         result == FLASH_SELFTEST_SPI_ERROR ? "SPIE" :
         result == FLASH_SELFTEST_ID_ERROR ? "READ" :
         result == FLASH_SELFTEST_WRITE_ERROR ? "WRIT" :
         result == FLASH_SELFTEST_ERASE_ERROR ? "ERAS" : "READ";
  startup_selftest_report_error(report, 4U, code);
  return -1;
}

static int selftest_sensors_bme(void *context, uint8_t attempt,
                                StartupSelfTestReport *report)
{
  SensorSample sample;
  (void)context;
  (void)attempt;
  Sensors_Init();
  Sensors_Get(&sample);
  if (sample.bme_status != 0) {
    startup_selftest_report_error(report, 5U, "I2CE");
    return -1;
  }
  if ((sample.valid & (SENSOR_TEMP_VALID | SENSOR_RH_VALID |
                       SENSOR_PRESS_VALID)) == 0U) {
    startup_selftest_report_error(report, 5U, "READ");
    return -1;
  }
  return 0;
}

static int selftest_gnss(void *context, uint8_t attempt,
                         StartupSelfTestReport *report)
{
  StartupSelfTestContext *selftest = (StartupSelfTestContext *)context;
  if (selftest == NULL || selftest->gnss == NULL) {
    startup_selftest_report_error(report, 6U, "UART");
    return -1;
  }
  if (attempt != 1U) gnss_reset_pulse();
  app_debug("GNSS configuring M10\r\n");
  if (gnss_service_init(selftest->gnss, &huart2) != 0) {
    startup_selftest_report_error(report, 6U, "UART");
    return -1;
  }
  if (gnss_service_configure(selftest->gnss) != 0) {
    startup_selftest_report_error(report, 6U, "CONF");
    return -1;
  }
  return 0;
}

static void selftest_camera_error(uint8_t device, int result,
                                  StartupSelfTestReport *report)
{
  const char *code = (result == -10 || result == -11 || result == -13) ? "I2CE" :
                     (result == -2 || result == -3) ? "IDER" :
                     result == -5 ? "CLCK" : "CONF";
  startup_selftest_report_error(report, device, code);
}

static int selftest_camera(void *context, uint8_t attempt,
                           StartupSelfTestReport *report, CameraPort port,
                           uint8_t device)
{
  int result;
  (void)context;
  (void)attempt;
  result = camera_init(port);
  if (result != 0) selftest_camera_error(device, result, report);
  camera_power_off();
  return result;
}

static int selftest_camera_a(void *context, uint8_t attempt,
                             StartupSelfTestReport *report)
{
  return selftest_camera(context, attempt, report, CAMERA_A, 7U);
}

static int selftest_camera_b(void *context, uint8_t attempt,
                             StartupSelfTestReport *report)
{
  return selftest_camera(context, attempt, report, CAMERA_B, 8U);
}

static void wspr_radio_gnss_poll(void *context);

static int selftest_send_report(const StartupSelfTestReport *report,
                                void *context)
{
  StartupSelfTestContext *selftest = (StartupSelfTestContext *)context;
  const char *payload;
  int length;
  /* Startup self-test is location-independent.  Send the same report twice
     on every regional APRS frequency used by the normal scheduler. */
  static const uint32_t frequencies[] = {
    144390000UL, /* North America */
    144800000UL, /* Europe */
    145175000UL, /* Australia */
    144640000UL, /* Japan and Asia/default */
    144575000UL  /* New Zealand */
  };
  const uint8_t frequency_count =
      (uint8_t)(sizeof(frequencies) / sizeof(frequencies[0]));
  if (selftest == NULL || selftest->aprs == NULL) return -1;
  length = startup_selftest_report_build(report, APP_APRS_CALL,
                                         APP_APRS_PATH, g_selftest_line,
                                         sizeof(g_selftest_line));
  if (length < 0) return -2;
  app_debug("SELFTEST APRS: ");
  app_debug(g_selftest_line);
  app_debug("\r\n");
  payload = strchr(g_selftest_line, ':');
  if (payload == NULL || selftest->aprs->initialized == 0U) return -3;
  ++payload;
  for (uint8_t f = 0U; f < frequency_count; ++f) {
    for (uint8_t repeat = 0U; repeat < 2U; ++repeat) {
      int result;
      (void)snprintf(g_radio_diagnostic, sizeof(g_radio_diagnostic),
                     "SELFTEST APRS TX start freq=%lu repeat=%u/2\r\n",
                     (unsigned long)frequencies[f], (unsigned)(repeat + 1U));
      app_debug(g_radio_diagnostic);
      result = aprs_service_send_text_with_poll(
          selftest->aprs, frequencies[f], payload, strlen(payload),
          wspr_radio_gnss_poll, selftest->gnss);
      (void)snprintf(g_radio_diagnostic, sizeof(g_radio_diagnostic),
                     "SELFTEST APRS TX result rc=%d count=%lu freq=%lu\r\n",
                     result, (unsigned long)selftest->aprs->tx_count,
                     (unsigned long)frequencies[f]);
      app_debug(g_radio_diagnostic);
      SX1278_APRS_TxReport(g_radio_diagnostic, sizeof(g_radio_diagnostic));
      app_debug(g_radio_diagnostic);
      if (result != 0) {
        SX1278_APRS_Diagnose(g_radio_diagnostic, sizeof(g_radio_diagnostic));
        app_debug(g_radio_diagnostic);
        return -4;
      }
      if (repeat == 0U) HAL_Delay(APP_APRS_REPEAT_GAP_MS);
    }
  }
  return 0;
}

static void selftest_delay(uint32_t milliseconds, void *context)
{
  (void)context;
  HAL_Delay(milliseconds);
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
                             uint64_t *last_slot,
                             uint64_t *due_slot)
{
  uint64_t key;
  uint8_t minute;

  if (solution == NULL || telemetry == NULL ||
      last_slot == NULL || due_slot == NULL || solution->utc_valid == 0U ||
      solution->fully_resolved == 0U) return 0U;
  key = gnss_minute_key(solution);
  minute = solution->minute;
  if ((minute % 10U) == APP_WSPR_REGULAR_MINUTE) {
    *telemetry = 0U;
  } else if ((minute % 10U) == APP_WSPR_TELEMETRY_MINUTE) {
    *telemetry = 1U;
  } else {
    return 0U;
  }
  if (solution->second > 1U) return 0U;
  if (((uint32_t)(120U - solution->second) * 1000U) <
      APP_WSPR_FRAME_DURATION_MS) return 0U;
  if (key == *last_slot) return 0U;
  *due_slot = key;
  return 1U;
}

static uint8_t wspr_tx_window_open(const GnssSolution *solution,
                                   uint8_t telemetry,
                                   uint64_t due_slot)
{
  if (solution == NULL || solution->utc_valid == 0U ||
      solution->fully_resolved == 0U || solution->second > 1U ||
      gnss_minute_key(solution) != due_slot) return 0U;
  (void)telemetry;
  if (((uint32_t)(120U - solution->second) * 1000U) <
      APP_WSPR_FRAME_DURATION_MS) return 0U;
  return 1U;
}

static uint8_t wspr_start_imminent(const GnssSolution *solution)
{
  uint8_t next_minute;

  if (solution == NULL || solution->utc_valid == 0U ||
      solution->fully_resolved == 0U || solution->second < 50U) return 0U;
  next_minute = (uint8_t)(solution->minute + 1U);
  if (next_minute >= 60U) next_minute = 0U;
  return (uint8_t)((next_minute % 10U) == APP_WSPR_REGULAR_MINUTE ||
                   (next_minute % 10U) == APP_WSPR_TELEMETRY_MINUTE);
}

static uint8_t aprs_start_imminent(const GnssSolution *solution)
{
  uint8_t next_minute;
  if (solution == NULL || solution->utc_valid == 0U ||
      solution->fully_resolved == 0U || solution->second < 50U) return 0U;
  next_minute = (uint8_t)(solution->minute + 1U);
  if (next_minute >= 60U) next_minute = 0U;
  return (uint8_t)((next_minute % 10U) == 1U ||
                   (next_minute % 10U) == 6U);
}

static uint8_t ssdv_group_window(const GnssSolution *solution)
{
  uint8_t minute;
  if (solution == NULL || solution->utc_valid == 0U ||
      solution->fully_resolved == 0U) return 0U;
  minute = (uint8_t)(solution->minute % 10U);
  /* 06 APRS has completed before the first group; groups run at 06,07,08,09,00. */
  return (uint8_t)(minute == 0U || minute == 6U || minute == 7U ||
                   minute == 8U || minute == 9U);
}

static uint8_t ssdv_group_due(const GnssSolution *solution,
                              uint64_t *slot_key)
{
  uint64_t key;
  if (slot_key == NULL || ssdv_group_window(solution) == 0U ||
      solution->second > 1U) return 0U;
  key = gnss_minute_key(solution);
  if (key == g_ssdv_last_group_slot) return 0U;
  *slot_key = key;
  return 1U;
}

static void ssdv_capture_after_aprs06(const GnssSolution *solution,
                                      AprsService *aprs, GnssService *gnss)
{
  int result;
  if (solution == NULL || aprs == NULL || gnss == NULL ||
      solution->utc_valid == 0U || solution->fully_resolved == 0U ||
      (solution->minute % 10U) != 6U) return;
  if (camera_ssdv_active() == 0U) {
    result = camera_ssdv_capture_start_sequence(
        aprs, gnss, g_ssdv_next_camera, g_ssdv_next_image_sequence,
        APP_SSDV_QUALITY, app_debug);
    if (result == 0) {
      g_ssdv_next_camera = g_ssdv_next_camera == CAMERA_A ? CAMERA_B : CAMERA_A;
      ++g_ssdv_next_image_sequence;
      ++g_ssdv_next_image_id;
    } else {
      app_debug("SSDV scheduled capture failed; retrying camera next 06\r\n");
      return;
    }
  }
  if (camera_ssdv_active() != 0U) {
    const GnssSolution *after_capture = gnss_service_solution(gnss);
    int group_result = camera_ssdv_send_next_group(aprs, gnss, app_debug);
    if (after_capture != NULL)
      g_ssdv_last_group_slot = gnss_minute_key(after_capture);
    if (group_result < 0) {
      app_debug("SSDV scheduled group failed\r\n");
    } else if (group_result > 0) {
      app_debug("SSDV scheduled image complete\r\n");
    }
  }
}

static void wspr_radio_gnss_poll(void *context)
{
  gnss_service_poll((GnssService *)context);
}

static uint32_t app_schedule_now(void *context)
{
  (void)context;
  return HAL_GetTick();
}

static void app_schedule_sleep(uint32_t milliseconds, void *context)
{
  (void)context;
  HAL_Delay(milliseconds);
}

static int telemetry_report(const GnssService *gnss, TelemetryFrame *reported,
                            uint8_t log_aprs_candidate,
                            uint8_t log_wspr_candidate)
{
  const GnssSolution *solution;
  TelemetryGnssSnapshot *snapshot = &g_telemetry_snapshot;
  SensorSample *sensors = &g_telemetry_sensors;
  TelemetryFrame *frame = &g_telemetry_frame;
  WsprFrame *wspr = &g_telemetry_wspr;
  char *debug_line = g_telemetry_debug_line;
  uint8_t health_mask;

  if (gnss == 0) return -1;
  solution = gnss_service_solution(gnss);
  if (solution == 0) return -1;
  /* A radio burst can block the foreground loop for several seconds.  Try
     one sensor refresh before taking the snapshot so the age check does not
     turn an otherwise healthy reading into an all-NA telemetry frame. */
  Sensors_Poll();
  memset(snapshot, 0, sizeof(*snapshot));
  snapshot->position_valid = solution->position_valid;
  snapshot->position_known = solution->position_known;
  snapshot->utc_valid = solution->utc_valid;
  snapshot->fully_resolved = solution->fully_resolved;
  snapshot->latitude_e7 = solution->latitude_e7;
  snapshot->longitude_e7 = solution->longitude_e7;
  snapshot->height_msl_mm = solution->height_msl_mm;
  snapshot->ground_speed_mm_s = solution->ground_speed_mm_s;
  snapshot->year = solution->year;
  snapshot->month = solution->month;
  snapshot->day = solution->day;
  snapshot->hour = solution->hour;
  snapshot->minute = solution->minute;
  snapshot->second = solution->second;
  Sensors_Get(sensors);
  health_mask = 0U;
  if ((sensors->valid & SENSOR_RAILS_VALID) != 0U) health_mask |= (uint8_t)(1U << 2);
  if (snapshot->position_valid != 0U) health_mask |= (uint8_t)(1U << 4);
  if (telemetry_service_build(snapshot, sensors, health_mask, frame) != 0) {
    app_debug("NORMAL telemetry build failed\r\n");
    return -2;
  }
  if (reported != 0) *reported = *frame;
  if (log_aprs_candidate != 0U) {
    (void)snprintf(debug_line, TELEMETRY_APRS_TEXT_CAPACITY + 64U,
                   "NORMAL APRS telemetry candidate: %s\r\n",
                   frame->aprs_text);
    app_debug(debug_line);
  }
  if (log_wspr_candidate != 0U && wspr_service_build(frame, wspr) == 0) {
    (void)snprintf(debug_line, TELEMETRY_APRS_TEXT_CAPACITY + 64U,
                   "NORMAL WSPR telemetry frame: %s\r\n", wspr->text);
    app_debug(debug_line);
  }
  return 0;
}

void app_main(void)
{
  AppFlow flow;
  GnssService gnss;
  AprsService aprs = {0};
  WsprRadioService wspr_radio = {0};
  TelemetryFrame *telemetry = &g_telemetry;
  char *debug_line = g_debug_line;
  uint64_t wspr_last_slot = UINT64_MAX;
  uint64_t wspr_failed_slot = UINT64_MAX;
  uint32_t wspr_retry_after = 0U;
  uint32_t experiment_deadline_ms = 0U;

  transponder_service_init(&g_transponder);
  high_speed_service_init(&g_high_speed);

  aprs_schedule_service_init(&g_aprs_schedule);
  app_flow_init(&flow);
  (void)app_flow_dispatch(&flow, APP_EVENT_BOOT_CHECK_DONE);
  app_debug("HB-2A formal software: boot -> GNSS_WAIT\r\n");

  MX_USART2_UART_Init();
  MX_I2C3_Init();
  MX_SPI2_Init();
  if (image_storage_init(&g_image_storage) == 0) {
    camera_ssdv_set_storage(&g_image_storage);
    g_ssdv_next_image_sequence = g_image_storage.has_image != 0U ?
        g_image_storage.latest_sequence + 1U : 0U;
    g_ssdv_next_image_id = (uint8_t)g_ssdv_next_image_sequence;
    (void)snprintf(g_radio_diagnostic, sizeof(g_radio_diagnostic),
                   "IMAGE storage ready latest=%lu\r\n",
                   (unsigned long)image_storage_latest(&g_image_storage));
    app_debug(g_radio_diagnostic);
  } else {
    app_debug("IMAGE storage init failed\r\n");
  }
  MX_I2C2_Init();
  if (aprs_service_init(&aprs) == 0) {
    app_debug("SELFTEST SX1278 transport initialized\r\n");
  } else {
    app_debug("SELFTEST SX1278 transport unavailable\r\n");
  }
  if (wspr_radio_service_init(&wspr_radio, APP_WSPR_CENTER_HZ) == 0) {
    app_debug("SELFTEST SI5351 transport initialized\r\n");
  } else {
    app_debug("SELFTEST SI5351 transport unavailable\r\n");
  }

  g_selftest_context.gnss = &gnss;
  g_selftest_context.aprs = &aprs;
  g_selftest_context.wspr = &wspr_radio;
  memset(&g_selftest_plan, 0, sizeof(g_selftest_plan));
  g_selftest_plan.attempt[0] = selftest_sensors_ina;
  g_selftest_plan.attempt[1] = selftest_sx1278;
  g_selftest_plan.attempt[2] = selftest_si5351;
  g_selftest_plan.attempt[3] = selftest_flash;
  g_selftest_plan.attempt[4] = selftest_sensors_bme;
  g_selftest_plan.attempt[5] = selftest_gnss;
  g_selftest_plan.attempt[6] = selftest_camera_a;
  g_selftest_plan.attempt[7] = selftest_camera_b;
  g_selftest_plan.send_report = selftest_send_report;
  g_selftest_plan.delay = selftest_delay;
  g_selftest_plan.context = &g_selftest_context;
  g_selftest_plan.reset_times = app_reset_times_next();
  g_selftest_plan.device_gap_ms = 500U;
  app_debug("SELFTEST startup sequence begin\r\n");
  (void)startup_selftest_run(&g_selftest_plan, &g_selftest_report);
  app_debug("SELFTEST startup sequence complete\r\n");

  if (gnss_wait_after_selftest(&gnss) != 0) {
    (void)app_flow_dispatch(&flow, APP_EVENT_GNSS_TIMEOUT);
    app_debug("GNSS failed three attempts; restarting MCU\r\n");
    NVIC_SystemReset();
    for (;;) { }
  }
  (void)app_flow_dispatch(&flow, APP_EVENT_GNSS_LOCKED);
  app_debug("HB-2A formal software: GNSS_WAIT -> NORMAL\r\n");
#if APP_STARTUP_SSDV_TEST
  app_debug("SSDV startup test begin: CAM_A then CAM_B\r\n");
  {
    int image_result = camera_ssdv_run_sequence(
        &aprs, &gnss, APP_SSDV_QUALITY, app_debug);
    (void)snprintf(g_debug_line, sizeof(g_debug_line),
                   "SSDV startup test complete rc=%d\r\n", image_result);
    app_debug(g_debug_line);
  }
  /* The optional startup capture also uses the persistent image directory.
     Continue numbering after anything it stored before accepting CAPTR. */
  if (g_image_storage.has_image != 0U) {
    g_ssdv_next_image_sequence = g_image_storage.latest_sequence + 1U;
    g_ssdv_next_image_id = (uint8_t)g_ssdv_next_image_sequence;
  }
#endif
  Sensors_Init();
  app_debug("NORMAL sensors initialized\r\n");
  app_debug(wspr_radio.initialized != 0U ?
            "NORMAL WSPR SI5351 initialized\r\n" :
            "NORMAL WSPR SI5351 unavailable\r\n");
  if (telemetry_report(&gnss, telemetry, 1U, 0U) == 0) {
    if (aprs.initialized != 0U) {
      app_debug("NORMAL APRS SX1278 initialized\r\n");
      const GnssSolution *startup_solution = gnss_service_solution(&gnss);
      if (wspr_radio.initialized != 0U &&
          wspr_start_imminent(startup_solution) != 0U) {
        app_debug("NORMAL startup APRS deferred for WSPR slot\r\n");
      } else if (aprs_service_send_telemetry_with_poll(
                     &aprs, telemetry, wspr_radio_gnss_poll, &gnss) == 0) {
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
  if (aprs.initialized != 0U && SX1278_APRS_RxStart() == 0) {
    app_debug("NORMAL uplink RX ready freq=435400000 rate=4800\r\n");
  } else if (aprs.initialized != 0U) {
    app_debug("NORMAL uplink RX unavailable; retry pending\r\n");
  }

  uint32_t next_telemetry_report = HAL_GetTick() + 60000U;
  for (;;) {
    uint8_t wspr_telemetry_slot;
    gnss_service_poll(&gnss);
    if ((app_flow_mode(&flow) == APP_MODE_EXPERIMENT ||
         app_flow_mode(&flow) == APP_MODE_HS_NEGOTIATION) &&
        experiment_deadline_ms != 0U &&
        (int32_t)(HAL_GetTick() - experiment_deadline_ms) >= 0) {
      app_debug("EXPR five-minute timeout; restarting MCU\r\n");
      (void)app_flow_dispatch(&flow, APP_EVENT_EXPERIMENT_TIMEOUT);
      NVIC_SystemReset();
      for (;;) { }
    }
    const GnssSolution *solution = gnss_service_solution(&gnss);
    if (app_flow_mode(&flow) == APP_MODE_NORMAL &&
        wspr_radio.transmitting == 0U && wspr_radio.initialized == 0U &&
        (int32_t)(HAL_GetTick() - wspr_retry_after) >= 0) {
      if (wspr_radio_service_init(&wspr_radio, APP_WSPR_CENTER_HZ) == 0) {
        app_debug("NORMAL WSPR SI5351 reinitialized\r\n");
        wspr_retry_after = 0U;
      } else {
        wspr_retry_after = HAL_GetTick() + 1000U;
      }
    }
    uint64_t wspr_due_slot = UINT64_MAX;
    if (app_flow_mode(&flow) == APP_MODE_NORMAL &&
        wspr_radio.initialized != 0U &&
        wspr_slot_due(solution, &wspr_telemetry_slot, &wspr_last_slot,
                      &wspr_due_slot) != 0U &&
        (wspr_due_slot != wspr_failed_slot ||
         (int32_t)(HAL_GetTick() - wspr_retry_after) >= 0)) {
      WsprFrame frame;
      int build_result;
      uint8_t slot_missed = 0U;
      if (wspr_telemetry_slot != 0U) {
        build_result = telemetry_report(&gnss, telemetry, 0U, 1U);
        if (build_result == 0) {
          build_result = wspr_service_build(telemetry, &frame);
        }
      } else {
        build_result = telemetry_report(&gnss, telemetry, 0U, 0U);
        if (build_result == 0) {
          build_result = wspr_service_build_standard(telemetry, &frame);
        }
      }
      if (build_result == 0 &&
          wspr_tx_window_open(gnss_service_solution(&gnss),
                              wspr_telemetry_slot, wspr_due_slot) == 0U) {
        wspr_last_slot = wspr_due_slot;
        slot_missed = 1U;
        app_debug("NORMAL WSPR slot missed before TX\r\n");
      }
      if (slot_missed != 0U) {
        /* The slot is intentionally skipped after the build window closed. */
      } else if (build_result != 0) {
        app_debug("NORMAL WSPR frame build failed\r\n");
      } else {
        (void)snprintf(debug_line, WSPR_SERVICE_TEXT_CAPACITY + 64U,
                       "NORMAL WSPR TX start: %s\r\n", frame.text);
        app_debug(debug_line);
        SX1278_APRS_RxStop();
        if (wspr_radio_service_send(&wspr_radio, &frame,
                                    wspr_radio_gnss_poll, &gnss) == 0) {
          wspr_last_slot = wspr_due_slot;
          wspr_failed_slot = UINT64_MAX;
          app_debug("NORMAL WSPR TX complete\r\n");
        } else {
          wspr_failed_slot = wspr_due_slot;
          wspr_retry_after = HAL_GetTick() + 1000U;
          app_debug("NORMAL WSPR TX failed\r\n");
        }
        (void)SX1278_APRS_RxStart();
      }
      if (build_result != 0 && slot_missed == 0U) {
        wspr_failed_slot = wspr_due_slot;
        wspr_retry_after = HAL_GetTick() + 1000U;
      }
    }
    /* WSPR polling updates the GNSS solution in place, but reacquire the
       pointer here so the APRS decision always uses the newest UTC sample. */
    solution = gnss_service_solution(&gnss);
    /* Refresh stale sensor samples after a long WSPR frame and before any
       APRS telemetry snapshot is built. This path is outside the WSPR slot. */
    Sensors_Poll();
    if (app_flow_mode(&flow) == APP_MODE_NORMAL &&
        aprs.initialized != 0U && wspr_radio.transmitting == 0U &&
        (wspr_radio.initialized == 0U ||
         wspr_start_imminent(solution) == 0U)) {
      APRS_Utc utc = {0};
      if (solution != NULL) {
        utc.valid = solution->utc_valid != 0U &&
                    solution->fully_resolved != 0U;
        utc.year = solution->year;
        utc.month = solution->month;
        utc.day = solution->day;
        utc.hour = solution->hour;
        utc.minute = solution->minute;
        utc.second = solution->second;
      }
      if (aprs_schedule_service_slot_due(&utc, &g_aprs_schedule) != 0) {
        if (telemetry_report(&gnss, telemetry, 1U, 0U) == 0) {
          solution = gnss_service_solution(&gnss);
          if (wspr_radio.initialized != 0U &&
              wspr_start_imminent(solution) != 0U) {
            app_debug("NORMAL APRS scheduled TX deferred for WSPR\r\n");
          } else {
            app_debug("NORMAL APRS scheduled TX start\r\n");
            if (aprs_schedule_service_send_burst(
                    &g_aprs_schedule, &aprs, telemetry,
                    APP_APRS_REPEAT_COUNT, APP_APRS_REPEAT_GAP_MS,
                    app_schedule_now, app_schedule_sleep,
                    wspr_radio_gnss_poll, &gnss) == 0) {
              (void)snprintf(debug_line, WSPR_SERVICE_TEXT_CAPACITY + 64U,
                             "NORMAL APRS scheduled TX complete count=%lu freq=%lu\r\n",
                             (unsigned long)aprs.tx_count,
                             (unsigned long)aprs.last_frequency_hz);
              app_debug(debug_line);
              solution = gnss_service_solution(&gnss);
              ssdv_capture_after_aprs06(solution, &aprs, &gnss);
            } else {
              app_debug("NORMAL APRS scheduled TX failed\r\n");
            }
          }
        } else {
          app_debug("NORMAL APRS scheduled telemetry unavailable\r\n");
        }
      }
    }
    solution = gnss_service_solution(&gnss);
    if (app_flow_mode(&flow) == APP_MODE_NORMAL &&
        aprs.initialized != 0U && camera_ssdv_active() != 0U &&
        wspr_radio.transmitting == 0U &&
        wspr_start_imminent(solution) == 0U) {
      uint64_t group_slot;
      if (ssdv_group_due(solution, &group_slot) != 0U) {
        int group_result = camera_ssdv_send_next_group(&aprs, &gnss, app_debug);
        g_ssdv_last_group_slot = group_slot;
        if (group_result < 0) {
          app_debug("SSDV scheduled group failed\r\n");
        } else if (group_result > 0) {
          app_debug("SSDV scheduled image complete\r\n");
        }
      }
    }
    if (app_flow_mode(&flow) == APP_MODE_NORMAL &&
        (int32_t)(HAL_GetTick() - next_telemetry_report) >= 0) {
      (void)telemetry_report(&gnss, 0, 1U, 0U);
      next_telemetry_report = HAL_GetTick() + 60000U;
    }
    {
      UplinkCommand command;
      int received = SX1278_APRS_RxPoll(&command);
      SX1278_APRS_RxReport(g_radio_diagnostic, sizeof(g_radio_diagnostic));
      if (g_radio_diagnostic[0] != 0) app_debug(g_radio_diagnostic);
      if (received != 0) {
        char command_name[8];
        AppMode mode = app_flow_mode(&flow);
        (void)uplink_field_text(&command, 0U, command_name, sizeof(command_name));
        for (unsigned i = 0U; command_name[i] != 0; ++i)
          if ((unsigned char)command_name[i] < 0x20U ||
              (unsigned char)command_name[i] > 0x7EU) command_name[i] = '?';
        (void)snprintf(g_radio_diagnostic, sizeof(g_radio_diagnostic),
                       "UPLINK command=%s blocks=%u CRC=OK mode=%u\r\n",
                       command_name, command.block_count, (unsigned)mode);
        app_debug(g_radio_diagnostic);
        if ((mode == APP_MODE_NORMAL || mode == APP_MODE_EXPERIMENT) &&
            command.block_count == 1U &&
            memcmp(command.data, "EXPR", 4U) == 0) {
          (void)app_flow_dispatch(&flow, APP_EVENT_VALID_EXPR);
          experiment_deadline_ms = HAL_GetTick() + 300000UL;
          app_debug("UPLINK EXPR accepted; experiment mode\r\n");
          (void)gmsk_send_text("ACK,EXPR", &gnss);
          if (mode == APP_MODE_NORMAL) next_telemetry_report = HAL_GetTick();
        } else if (mode == APP_MODE_HS_NEGOTIATION &&
                   (memcmp(command.data, "CAPTR", 5U) == 0 ||
                    memcmp(command.data, "REQF", 4U) == 0)) {
          /* Requests have no transaction ID. Re-advertise the existing
             session on an ACK loss; never capture a duplicate image. */
          (void)gmsk_send_text(g_high_speed.request == HS_REQUEST_CAPTURE ?
                                 "ACK,CAPTR" : "ACK,REQF", &gnss);
          (void)gmsk_send_text("HSSUP,3F,0F,3F", &gnss);
          high_speed_service_control_sent(&g_high_speed, HS_EVENT_SEND_SUPPORT,
                                            HAL_GetTick());
        } else if (mode == APP_MODE_EXPERIMENT &&
                   (memcmp(command.data, "CAPTR", 5U) == 0 ||
                    memcmp(command.data, "REQF", 4U) == 0)) {
          if (memcmp(command.data, "CAPTR", 5U) == 0) {
            char cap_camera[8], cap_width[8], cap_height[8], cap_quality[8];
            (void)uplink_field_text(&command, 1U, cap_camera, sizeof(cap_camera));
            (void)uplink_field_text(&command, 2U, cap_width, sizeof(cap_width));
            (void)uplink_field_text(&command, 3U, cap_height, sizeof(cap_height));
            (void)uplink_field_text(&command, 4U, cap_quality, sizeof(cap_quality));
            (void)snprintf(g_radio_diagnostic, sizeof(g_radio_diagnostic),
                           "UPLINK CAPTR request camera=%s width=%s height=%s quality=%s\r\n",
                           cap_camera, cap_width, cap_height, cap_quality);
            app_debug(g_radio_diagnostic);
          }
          if (high_speed_service_start(&g_high_speed, &command,
                                       HAL_GetTick()) == 0) {
            (void)app_flow_dispatch(
                &flow, memcmp(command.data, "CAPTR", 5U) == 0 ?
                           APP_EVENT_VALID_CAPTURE : APP_EVENT_VALID_FILE_REQUEST);
            experiment_deadline_ms = HAL_GetTick() + 300000UL;
            high_speed_drain_events(&flow, &gnss);
          } else {
            app_debug("UPLINK high-speed request rejected\r\n");
            high_speed_drain_events(&flow, &gnss);
          }
        } else if (app_flow_mode(&flow) == APP_MODE_HS_NEGOTIATION ||
                   app_flow_mode(&flow) == APP_MODE_HS_TRANSFER) {
          int command_result = high_speed_service_command(
              &g_high_speed, &command, HAL_GetTick());
          if (command_result == 0) {
            if (memcmp(command.data, "HSPAR", 5U) == 0)
              (void)app_flow_dispatch(&flow, APP_EVENT_HS_PARAMETERS_ACCEPTED);
            else if (memcmp(command.data, "CFRM", 4U) == 0)
              (void)app_flow_dispatch(&flow, APP_EVENT_HS_CONFIRM);
            else if (memcmp(command.data, "CANCEL", 6U) == 0)
              (void)app_flow_dispatch(&flow, APP_EVENT_HS_CANCEL);
            experiment_deadline_ms = HAL_GetTick() + 300000UL;
            high_speed_drain_events(&flow, &gnss);
          } else {
            (void)snprintf(g_radio_diagnostic, sizeof(g_radio_diagnostic),
                           "UPLINK HS command rejected rc=%d\r\n", command_result);
            app_debug(g_radio_diagnostic);
          }
        } else if (mode == APP_MODE_NORMAL) {
          int enqueue_result = transponder_service_enqueue(&g_transponder, &command);
          if (enqueue_result == 0) {
            app_debug("UPLINK TRANPD accepted\r\n");
          } else if (enqueue_result == -4) {
            app_debug("UPLINK TRANPD rejected path\r\n");
          } else if (enqueue_result == -2) {
            app_debug("UPLINK TRANPD queue full\r\n");
          } else {
            (void)snprintf(g_radio_diagnostic, sizeof(g_radio_diagnostic),
                           "UPLINK command rejected in NORMAL rc=%d\r\n", enqueue_result);
            app_debug(g_radio_diagnostic);
          }
        } else {
          app_debug("UPLINK command ignored in current mode\r\n");
        }
      }
      (void)high_speed_service_poll(&g_high_speed, HAL_GetTick());
      high_speed_drain_events(&flow, &gnss);
      if (app_flow_mode(&flow) == APP_MODE_EXPERIMENT &&
          (int32_t)(HAL_GetTick() - next_telemetry_report) >= 0) {
        if (telemetry_report(&gnss, telemetry, 0U, 0U) == 0)
          (void)gmsk_send_text(telemetry->aprs_text, &gnss);
        next_telemetry_report = HAL_GetTick() + APP_GMSK_TELEMETRY_PERIOD_MS;
      }
      if (app_flow_mode(&flow) == APP_MODE_NORMAL &&
          wspr_radio.transmitting == 0U &&
          (wspr_radio.initialized == 0U || wspr_start_imminent(solution) == 0U) &&
          aprs_start_imminent(solution) == 0U) {
        uint32_t forwarded_before = g_transponder.forwarded_count;
        transponder_service_poll(&g_transponder, &aprs);
        if (g_transponder.forwarded_count != forwarded_before)
          app_debug("TRANSPONDER APRS TX complete freq=144925000\r\n");
      }
    }
    HAL_Delay(10U);
  }
}

