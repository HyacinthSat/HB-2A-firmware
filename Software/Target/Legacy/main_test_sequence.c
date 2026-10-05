#include "main.h"
#include "dcmi.h"
#include "i2c.h"
#include "spi.h"
#include "usart.h"
#include "gpio.h"
#include "camera.h"
#include "si5351.h"
#include "u4b_telemetry.h"
#include "ubx.h"
#include "wspr_enc.h"
#include "aprs.h"
#include "app_config.h"
#include "ax25_afsk.h"
#include "sx1278_aprs.h"
#include "sensors.h"
#include "ssdv.h"
#include "factory_image.h"

#include <string.h>
#include <stdio.h>

typedef struct {
  uint8_t valid;
  uint8_t full;
  uint16_t year;
  uint8_t month;
  uint8_t day;
  uint8_t hour;
  uint8_t minute;
  uint8_t second;
  uint32_t tick;
} UtcState;

static UtcState g_utc;
static U4B_Sample g_sample = {
  APP_TEST_LAT_E7, APP_TEST_LON_E7, APP_TEST_ALTITUDE_M,
  0, 0U, /* WSPR telemetry is gated on fresh sensor readings. */
  APP_TEST_SPEED_KNOTS, 0U
};
static uint8_t g_have_fix;
static uint8_t g_gnss_3d_fix;
static uint8_t g_si5351_ready;
static uint8_t g_indoor_next_telemetry;
static uint32_t g_indoor_next_wspr_tick;
static uint32_t g_indoor_next_aprs_tick;
static uint64_t g_last_wspr_slot = ~0ULL;
static uint64_t g_last_aprs_slot = ~0ULL;
static uint8_t g_gnss_byte;
static uint8_t g_gnss_forward;
static uint8_t g_ssdv_image_id;
static ssdv_t g_ssdv;
static uint8_t g_ssdv_packet[APP_SSDV_PACKET_SIZE];
static uint8_t g_ssdv_feed[APP_SSDV_FEED_SIZE];
static uint8_t g_ssdv_aprs[AX25_MAX_INFO];
static char g_ssdv_encoded[300];
static char g_ssdv_left[140];
static char g_ssdv_right[140];
static uint8_t g_ssdv_xor[140];
static char g_ssdv_k_encoded[200];

/* basE91 alphabet used by DL7AD/aprs_ssdv_decoder. */
static const char g_base91[] =
    "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789"
    "!#$%&()*+,./:;<=>?@[]^_`{|}~\"";

static size_t base91_encode(const uint8_t *input, size_t length, char *output,
                            size_t capacity)
{
  uint32_t buffer = 0U;
  unsigned bits = 0U;
  size_t written = 0U;
  if (input == NULL || output == NULL) return 0U;
  for (size_t i = 0U; i < length; ++i) {
    buffer |= (uint32_t)input[i] << bits;
    bits += 8U;
    if (bits > 13U) {
      uint32_t value = buffer & 8191U;
      unsigned take = (value > 88U) ? 13U : 14U;
      value = buffer & ((1UL << take) - 1UL);
      buffer >>= take;
      bits -= take;
      if (written + 2U >= capacity) return 0U;
      output[written++] = g_base91[value % 91U];
      output[written++] = g_base91[value / 91U];
    }
  }
  if (bits != 0U) {
    unsigned tail = (bits > 7U || buffer > 90U) ? 2U : 1U;
    if (written + tail >= capacity) return 0U;
    output[written++] = g_base91[buffer % 91U];
    if (tail == 2U) output[written++] = g_base91[buffer / 91U];
  }
  return written;
}

static void debug_send(const char *text)
{
  (void)HAL_UART_Transmit(&huart4, (uint8_t *)text, (uint16_t)strlen(text), 1000U);
}

static uint32_t rd_u32(const uint8_t *p)
{
  return (uint32_t)p[0] | ((uint32_t)p[1] << 8) |
         ((uint32_t)p[2] << 16) | ((uint32_t)p[3] << 24);
}

static int32_t rd_i32(const uint8_t *p)
{
  return (int32_t)rd_u32(p);
}

static uint16_t rd_u16(const uint8_t *p)
{
  return (uint16_t)p[0] | ((uint16_t)p[1] << 8);
}

uint8_t GNSS_OnUbxNavPvt(const uint8_t *p, uint16_t len)
{
  int32_t h_mm;
  uint32_t speed_mm_s;
  uint8_t valid;

  if (p == NULL || len != 92U) return 0U;
  valid = p[11];
  g_gnss_3d_fix = Ubx_NavPvtHas3DFix(p, len);
  g_utc.full = ((valid & 7U) == 7U) ? 1U : 0U;
  g_utc.valid = 0U;
  if ((valid & 3U) == 3U) {
    g_utc.year = rd_u16(p + 4);
    g_utc.month = p[6];
    g_utc.day = p[7];
    g_utc.hour = p[8];
    g_utc.minute = p[9];
    g_utc.second = p[10];
    g_utc.tick = HAL_GetTick();
    g_utc.valid = 1U;
  }

  /* Test coordinates remain synthetic; actual lock is tracked separately. */
  if (APP_INDOOR_TEST_MODE != 0U) return g_gnss_3d_fix;

  h_mm = rd_i32(p + 36);
  speed_mm_s = rd_u32(p + 68);
  /* Flight operation requires a GNSS 3D fix and the receiver's fixOK bit. */
  if (g_gnss_3d_fix == 0U) {
    g_sample.gps_valid = 0U;
    g_have_fix = 0U;
    return 0U;
  }

  g_sample.longitude_e7 = rd_i32(p + 24);
  g_sample.latitude_e7 = rd_i32(p + 28);
  g_sample.altitude_m = h_mm > 0 ? (uint32_t)((h_mm + 500) / 1000) : 0U;
  uint64_t knots = ((uint64_t)speed_mm_s * 36ULL + 9260ULL) / 18520ULL;
  g_sample.speed_knots = knots > 82U ? 82U : (uint16_t)knots;
  g_sample.gps_valid = 1U;
  g_have_fix = 1U;
  return 1U;
}

static void gnss_poll(void)
{
  uint32_t n = 0U;
  while (n++ < 256U && Ubx_ReadByte(&g_gnss_byte)) {
    if (g_gnss_forward != 0U) {
      (void)HAL_UART_Transmit(&huart4, &g_gnss_byte, 1U, 20U);
    }
    Ubx_OnByte(g_gnss_byte);
    /* End raw forwarding exactly after the complete validated NAV-PVT frame. */
    if (g_gnss_forward != 0U && g_gnss_3d_fix != 0U) return;
  }
}

static void gnss_wait_for_3d_fix(void)
{
  uint32_t start = HAL_GetTick();
  char line[96];

  debug_send("GNSS waiting 3D fix + UTC; raw UBX -> UART4\r\n");
  g_gnss_forward = 1U;
  g_gnss_3d_fix = 0U;
  while (g_gnss_3d_fix == 0U) {
    gnss_poll();
    /* No RF, timeout bypass or text injected into binary frames while waiting. */
    HAL_Delay(1U);
  }
  g_gnss_forward = 0U;
  (void)snprintf(line, sizeof(line), "\r\nGNSS 3D fix ready t=%lu s rx_errors=%lu\r\n",
                 (unsigned long)((HAL_GetTick() - start) / 1000U),
                 (unsigned long)Ubx_RxErrors());
  debug_send(line);
}

static uint8_t dim(uint16_t year, uint8_t month)
{
  static const uint8_t days[12] = {31,28,31,30,31,30,31,31,30,31,30,31};
  if (month == 2U && ((year % 4U == 0U && year % 100U != 0U) || year % 400U == 0U)) return 29U;
  return (month >= 1U && month <= 12U) ? days[month - 1U] : 31U;
}

static void utc_now(uint16_t *year, uint8_t *month, uint8_t *day,
                    uint8_t *hour, uint8_t *minute, uint8_t *second)
{
  uint32_t seconds;
  uint32_t days;

  seconds = (uint32_t)g_utc.hour * 3600U + (uint32_t)g_utc.minute * 60U +
            g_utc.second + (HAL_GetTick() - g_utc.tick) / 1000U;
  days = seconds / 86400U;
  seconds %= 86400U;
  *year = g_utc.year;
  *month = g_utc.month;
  *day = g_utc.day;
  while (days-- != 0U) {
    if (++*day > dim(*year, *month)) {
      *day = 1U;
      if (++*month > 12U) {
        *month = 1U;
        ++*year;
      }
    }
  }
  *hour = (uint8_t)(seconds / 3600U);
  *minute = (uint8_t)((seconds % 3600U) / 60U);
  *second = (uint8_t)(seconds % 60U);
}

static void aprs_utc_now(APRS_Utc *utc)
{
  uint16_t year;
  if (utc == NULL) return;
  memset(utc, 0, sizeof(*utc));
  if (g_utc.valid != 0U) {
    utc_now(&year, &utc->month, &utc->day, &utc->hour, &utc->minute, &utc->second);
    utc->year = year;
    utc->valid = 1U;
  } else {
    uint32_t seconds = HAL_GetTick() / 1000U;
    utc->day = 1U;
    utc->hour = (uint8_t)((seconds / 3600U) % 24U);
    utc->minute = (uint8_t)((seconds / 60U) % 60U);
    utc->second = (uint8_t)(seconds % 60U);
  }
}

static uint8_t slot_due(uint8_t *telemetry)
{
  uint16_t year;
  uint8_t month;
  uint8_t day;
  uint8_t hour;
  uint8_t minute;
  uint8_t second;
  uint64_t key;

  if (telemetry == NULL || g_utc.valid == 0U || g_utc.full == 0U || g_have_fix == 0U) return 0U;
  utc_now(&year, &month, &day, &hour, &minute, &second);
  if (second > 1U || (minute & 1U) != 0U) return 0U;
  key = ((uint64_t)year << 40) | ((uint64_t)month << 32) |
        ((uint64_t)day << 24) | ((uint64_t)hour << 16) | ((uint64_t)minute << 8);
  if (key == g_last_wspr_slot) return 0U;
  if ((minute % 10U) == APP_WSPR_REGULAR_MINUTE) *telemetry = 0U;
  else if ((minute % 10U) == APP_WSPR_TELEMETRY_MINUTE) *telemetry = 1U;
  else return 0U;
  g_last_wspr_slot = key;
  return 1U;
}

static uint8_t aprs_slot_due(void)
{
  uint16_t year;
  uint8_t month;
  uint8_t day;
  uint8_t hour;
  uint8_t minute;
  uint8_t second;
  uint64_t key;

  if (g_utc.valid == 0U || g_utc.full == 0U || g_have_fix == 0U) return 0U;
  utc_now(&year, &month, &day, &hour, &minute, &second);
  if (second > 1U || (minute % 5U) != 1U) return 0U;
  key = ((uint64_t)year << 40) | ((uint64_t)month << 32) |
        ((uint64_t)day << 24) | ((uint64_t)hour << 16) | ((uint64_t)minute << 8);
  if (key == g_last_aprs_slot) return 0U;
  g_last_aprs_slot = key;
  return 1U;
}

static void grid4_from_fix(char grid[5])
{
  int64_t lat = (int64_t)g_sample.latitude_e7 + 900000000LL;
  int64_t lon = (int64_t)g_sample.longitude_e7 + 1800000000LL;
  if (grid == NULL) return;
  if (g_have_fix == 0U || lat < 0 || lon < 0 ||
      lat >= 1800000000L || lon >= 3600000000L) {
    (void)strcpy(grid, APP_WSPR_DEFAULT_GRID);
    return;
  }
  grid[0] = (char)('A' + lon / 200000000L);
  grid[1] = (char)('A' + lat / 100000000L);
  grid[2] = (char)('0' + (lon % 200000000L) / 20000000L);
  grid[3] = (char)('0' + (lat % 100000000L) / 10000000L);
  grid[4] = 0;
}

static void wspr_transmit(const uint8_t symbols[APP_WSPR_SYMBOL_COUNT])
{
  uint32_t start = HAL_GetTick();
  uint32_t i;

  /* Measure the indoor interval from RF start, never from frame completion. */
  if (APP_INDOOR_TEST_MODE != 0U) {
    g_indoor_next_wspr_tick = start + APP_INDOOR_WSPR_PERIOD_MS;
  }
  si5351_EnableOutputs(1U);
  for (i = 0U; i < APP_WSPR_SYMBOL_COUNT; ++i) {
    int32_t frequency = (int32_t)((int64_t)APP_WSPR_CENTER_HZ +
                                  ((int64_t)symbols[i] * 12000LL + 4096LL) / 8192LL);
    uint32_t target = start + (uint32_t)(((uint64_t)(i + 1U) * 8192ULL * 1000ULL) / 12000ULL);
    si5351_SetupCLK0(frequency, SI5351_DRIVE_STRENGTH_8MA);
    while ((int32_t)(HAL_GetTick() - target) < 0) {
      gnss_poll();
      /* Sensors share I2C3 with SI5351. Finish bounded foreground accesses
         well before the next symbol update; no I2C work from interrupts. */
      if ((int32_t)(target - HAL_GetTick()) > 200) Sensors_Poll();
      HAL_Delay(1U);
    }
  }
  si5351_EnableOutputs(0U);
  debug_send("WSPR frame done; RF off\r\n");
}

static void wspr_send_regular(void)
{
  uint8_t symbols[APP_WSPR_SYMBOL_COUNT];
  char grid[5];
  char power[4];

  grid4_from_fix(grid);
  (void)snprintf(power, sizeof(power), "%u", 10U);
  if (wspr_enc(APP_WSPR_CALL, grid, power, symbols) != 0U) {
    debug_send("TX WSPR regular BG7ZDQ\r\n");
    wspr_transmit(symbols);
  } else {
    debug_send("WSPR regular encode failed\r\n");
  }
}

static void wspr_send_telemetry(void)
{
  uint8_t symbols[APP_WSPR_SYMBOL_COUNT];
  U4B_Message message;
  char power[4];
  char line[112];
  SensorSample sensors;
  Sensors_Get(&sensors);
  if ((sensors.valid & (SENSOR_TEMP_VALID | SENSOR_RAILS_VALID)) !=
      (SENSOR_TEMP_VALID | SENSOR_RAILS_VALID)) {
    debug_send("WSPR telemetry skipped: sensors unavailable/stale\r\n");
    return;
  }
  int32_t temp = sensors.temperature_centi_c;
  g_sample.temperature_c = (int16_t)((temp + (temp < 0 ? -50 : 50)) / 100);
  /* Board monitors regulated rails, not battery/solar input voltage. */
  g_sample.voltage_mv = sensors.rail_mv[0];

  if (APP_INDOOR_TEST_MODE != 0U) {
    U4B_BuildBasicWithGrid56(&g_sample, APP_WSPR_ID13, APP_TEST_GRID56, &message);
  } else {
    U4B_BuildBasic(&g_sample, APP_WSPR_ID13, &message);
  }
  (void)snprintf(power, sizeof(power), "%u", (unsigned)message.power_dbm);
  if (wspr_enc(message.callsign, message.grid4, power, symbols) != 0U) {
    (void)snprintf(line, sizeof(line), "TX WSPR telemetry %s %s %udBm\r\n",
                   message.callsign, message.grid4, (unsigned)message.power_dbm);
    debug_send(line);
    wspr_transmit(symbols);
  } else {
    debug_send("WSPR telemetry encode failed\r\n");
  }
}

/* Return 0 only when deferred; attempted/disabled bursts return 1. */
static uint8_t aprs_send_burst(uint32_t budget_ms, uint8_t boot)
{
  APRS_Utc utc;
  APRS_Region region;
  char frame[240];
  char line[320];
  uint32_t frequency;
  uint8_t health_mask = 0U;
  uint8_t repeat;
  uint32_t start = HAL_GetTick();
  if (!SX1278_APRS_Ready()) return 1U;
  SensorSample sensors;
  Sensors_Get(&sensors);

  aprs_utc_now(&utc);
  /* Health bits are set only after the corresponding probe is implemented. */
  if (g_si5351_ready != 0U) health_mask |= (uint8_t)(1U << 1);
  health_mask |= 1U; /* SX1278 register/configuration probe passed. */
  if (sensors.valid & SENSOR_RAILS_VALID) health_mask |= (uint8_t)(1U << 2);
  if (g_sample.gps_valid != 0U) health_mask |= (uint8_t)(1U << 4);
  frequency = APRS_SelectFrequencyHz(g_sample.latitude_e7, g_sample.longitude_e7,
                                     g_sample.gps_valid, &region);
  int length;
  if (boot && APP_INDOOR_TEST_MODE == 0U) {
    length = snprintf(frame, sizeof(frame), "%s>%s:>M:BOOT B,%02X WAIT_GNSS",
                      APP_APRS_CALL, APP_APRS_PATH, (unsigned)health_mask);
  } else {
    length = APRS_BuildTelemetryWithSensors(&g_sample, &utc, &sensors, health_mask, frame, sizeof(frame));
  }
  if (length < 0 || (size_t)length >= sizeof(frame)) {
    debug_send("APRS frame build failed\r\n");
    return 1U;
  }
  const char *info = strchr(frame, ':');
  int duration = info ? SX1278_APRS_Prepare(info + 1, strlen(info + 1)) : -1;
  if (duration < 0) {
    debug_send("APRS encode failed\r\n");
    return 1U;
  }
  uint32_t required = APP_APRS_REPEAT_COUNT * ((uint32_t)duration + 250U) +
                      (APP_APRS_REPEAT_COUNT - 1U) * APP_APRS_REPEAT_GAP_MS +
                      APP_APRS_SLOT_GUARD_MS;
  if (budget_ms < required + (HAL_GetTick() - start)) return 0U;
  (void)snprintf(line, sizeof(line), "APRS RF freq=%lu region=%u power=%u duration=%ums x%u%s\r\n%s\r\n",
                 (unsigned long)frequency, (unsigned)region, APP_APRS_POWER_DBM,
                 (unsigned)duration, APP_APRS_REPEAT_COUNT, boot ? " boot" : "", frame);
  (void)HAL_UART_Transmit(&huart4, (uint8_t *)line, (uint16_t)strlen(line), 50U);
  for (repeat = 0U; repeat < APP_APRS_REPEAT_COUNT; ++repeat) {
    uint32_t elapsed = HAL_GetTick() - start;
    if (elapsed + (uint32_t)duration + 200U + APP_APRS_SLOT_GUARD_MS > budget_ms) break;
    int result = SX1278_APRS_Send(frequency, (uint32_t)duration + 200U, gnss_poll);
    (void)snprintf(line, sizeof(line), "APRS TX %u/%u %s rc=%d; RF off\r\n",
                   (unsigned)repeat + 1U, APP_APRS_REPEAT_COUNT,
                   result == 0 ? "done" : "failed", result);
    (void)HAL_UART_Transmit(&huart4, (uint8_t *)line, (uint16_t)strlen(line), 50U);
    if (repeat == 0U || result != 0) {
      SX1278_APRS_TxReport(line, sizeof(line));
      (void)HAL_UART_Transmit(&huart4, (uint8_t *)line, (uint16_t)strlen(line), 50U);
    }
    if (result != 0) break;
    if (repeat + 1U < APP_APRS_REPEAT_COUNT) {
      uint32_t gap_start = HAL_GetTick();
      while (HAL_GetTick() - gap_start < APP_APRS_REPEAT_GAP_MS) {
        gnss_poll();
        HAL_Delay(1U);
      }
    }
  }
  return 1U;
}

static int aprs_send_binary(const uint8_t *payload, size_t length, uint32_t frequency,
                            const char *destination)
{
  int duration;
  if (!SX1278_APRS_Ready() || payload == NULL || length == 0U ||
      length > AX25_MAX_INFO || destination == NULL) return -1;
  duration = SX1278_APRS_PrepareAddressed(APP_APRS_CALL, destination, payload, length);
  if (duration < 0) return -2;
  return SX1278_APRS_Send(frequency, (uint32_t)duration + 250U, gnss_poll);
}

static int hex_digit(unsigned value)
{
  return value < 10U ? (int)('0' + value) : (int)('A' + value - 10U);
}

static size_t ssdv_make_aprs_split(char kind, size_t encoded_length,
                                   uint8_t image_id, uint16_t packet_id)
{
  size_t half = (encoded_length + 1U) / 2U;
  if (half == 0U || half > sizeof(g_ssdv_left) || encoded_length > 0xFFFU) return 0U;
  g_ssdv_aprs[0] = '{'; g_ssdv_aprs[1] = '{'; g_ssdv_aprs[2] = (uint8_t)kind;
  g_ssdv_aprs[3] = (uint8_t)hex_digit(image_id >> 4); g_ssdv_aprs[4] = (uint8_t)hex_digit(image_id & 15U);
  g_ssdv_aprs[5] = (uint8_t)hex_digit(packet_id >> 12); g_ssdv_aprs[6] = (uint8_t)hex_digit((packet_id >> 8) & 15U);
  g_ssdv_aprs[7] = (uint8_t)hex_digit((packet_id >> 4) & 15U); g_ssdv_aprs[8] = (uint8_t)hex_digit(packet_id & 15U);
  g_ssdv_aprs[9] = (uint8_t)hex_digit((unsigned)(encoded_length >> 8));
  g_ssdv_aprs[10] = (uint8_t)hex_digit((encoded_length >> 4) & 15U); g_ssdv_aprs[11] = (uint8_t)hex_digit(encoded_length & 15U);
  g_ssdv_aprs[12] = (uint8_t)hex_digit((unsigned)(half >> 8));
  g_ssdv_aprs[13] = (uint8_t)hex_digit((half >> 4) & 15U); g_ssdv_aprs[14] = (uint8_t)hex_digit(half & 15U);
  g_ssdv_aprs[15] = ':';
  if (kind == 'I') memcpy(&g_ssdv_aprs[16], g_ssdv_left, half);
  else if (kind == 'J') memcpy(&g_ssdv_aprs[16], g_ssdv_right, half);
  else {
    size_t n = base91_encode(g_ssdv_xor, half, g_ssdv_k_encoded, sizeof(g_ssdv_k_encoded));
    if (n == 0U || 16U + n > AX25_MAX_INFO) return 0U;
    memcpy(&g_ssdv_aprs[16], g_ssdv_k_encoded, n);
    return 16U + n;
  }
  return 16U + half <= AX25_MAX_INFO ? 16U + half : 0U;
}

static int ssdv_send_image_quality(const uint8_t *jpeg, uint32_t length, int8_t quality)
{
  uint32_t offset = 0U;
  uint32_t packets = 0U;
  APRS_Region region;
  uint32_t frequency;
  int status;

  /* Flash-backed reference images need not fit in the capture DMA buffer.
     The encoder copies only APP_SSDV_FEED_SIZE bytes per feed. */
  if (jpeg == NULL || length < 4U || length > APP_SSDV_MAX_JPEG_BYTES ||
      jpeg[0] != 0xFFU || jpeg[1] != 0xD8U) return -1;
  frequency = APRS_SelectFrequencyHz(g_sample.latitude_e7, g_sample.longitude_e7,
                                     g_sample.gps_valid, &region);
  status = (int)ssdv_enc_init(&g_ssdv, SSDV_TYPE_NORMAL, APP_WSPR_CALL,
                              g_ssdv_image_id++, quality,
                              APP_SSDV_PACKET_SIZE);
  if (status != SSDV_OK || ssdv_enc_set_buffer(&g_ssdv, g_ssdv_packet) != SSDV_OK) return -2;

  debug_send("SSDV ARRS/APZDQ7 basE91 encode/send start\r\n");
  for (;;) {
    status = (int)ssdv_enc_get_packet(&g_ssdv);
    if (status == SSDV_FEED_ME) {
      uint32_t chunk = length - offset;
      if (chunk == 0U) return -3;
      if (chunk > sizeof(g_ssdv_feed)) chunk = sizeof(g_ssdv_feed);
      memcpy(g_ssdv_feed, jpeg + offset, chunk);
      offset += chunk;
      if (ssdv_enc_feed(&g_ssdv, g_ssdv_feed, chunk) != SSDV_OK) return -4;
      continue;
    }
    if (status == SSDV_EOI) break;
    if (status != SSDV_OK) return -5;
    /* Re-select before every image packet so a future moving-platform image
       transfer follows the same regional APRS policy as telemetry. */
    frequency = APRS_SelectFrequencyHz(g_sample.latitude_e7, g_sample.longitude_e7,
                                       g_sample.gps_valid, &region);
    size_t encoded_length = base91_encode(&g_ssdv_packet[1], 219U,
                                          g_ssdv_encoded, sizeof(g_ssdv_encoded));
    size_t half = (encoded_length + 1U) / 2U;
    if (encoded_length == 0U || half > sizeof(g_ssdv_left)) return -6;
    memcpy(g_ssdv_left, g_ssdv_encoded, half);
    memcpy(g_ssdv_right, g_ssdv_encoded + half, encoded_length - half);
    if (encoded_length - half < half)
      memset(g_ssdv_right + encoded_length - half, g_base91[0], half - (encoded_length - half));
    for (size_t i = 0U; i < half; ++i)
      g_ssdv_xor[i] = (uint8_t)g_ssdv_left[i] ^ (uint8_t)g_ssdv_right[i];
    const char kinds[3] = {'I', 'J', 'K'};
    for (unsigned part = 0U; part < 3U; ++part) {
      size_t payload_length = ssdv_make_aprs_split(kinds[part], encoded_length,
                                                   g_ssdv_packet[6],
                                                   (uint16_t)(((uint16_t)g_ssdv_packet[7] << 8) | g_ssdv_packet[8]));
      if (payload_length == 0U) return -6;
      status = aprs_send_binary(g_ssdv_aprs, payload_length, frequency,
                                APP_SSDV_APRS_DEST);
      if (status != 0) return -7;
    }
    ++packets;
    if ((packets % 10U) == 0U) {
      char line[80];
      (void)snprintf(line, sizeof(line), "SSDV packet %lu region=%u\r\n",
                     (unsigned long)packets, (unsigned)region);
      debug_send(line);
    }
  }
  {
    char line[96];
    (void)snprintf(line, sizeof(line), "SSDV done packets=%lu bytes=%lu freq=%lu\r\n",
                   (unsigned long)packets, (unsigned long)length,
                   (unsigned long)frequency);
    debug_send(line);
  }
  return 0;
}

static int ssdv_send_image(const uint8_t *jpeg, uint32_t length)
{
  return ssdv_send_image_quality(jpeg, length, APP_SSDV_QUALITY);
}

static void indoor_camera_ssdv_sequence(void)
{
  static const CameraPort ports[] = {CAMERA_A, CAMERA_B};
  MX_DCMI_Init();
  for (unsigned i = 0U; i < sizeof(ports) / sizeof(ports[0]); ++i) {
    uint8_t *jpeg = NULL;
    uint32_t length = 0U;
    char name = ports[i] == CAMERA_A ? 'A' : 'B';
    char line[64];
    (void)snprintf(line, sizeof(line), "TEST CAM_%c VGA init\r\n", name);
    debug_send(line);
    int result = camera_init(ports[i]);
    if (result != 0) {
      (void)snprintf(line, sizeof(line), "CAM_%c init failed rc=%d; skip\r\n", name, result);
      debug_send(line);
      camera_power_off();
      continue;
    }
    /* Prime auto-exposure/white-balance with discarded frames before the
       image that is sent. The sensor stays powered and configured between
       these snapshots. */
    for (unsigned warmup = 0U; warmup < APP_CAMERA_WARMUP_COUNT; ++warmup) {
      uint8_t *discard_jpeg = NULL;
      uint32_t discard_length = 0U;
      (void)snprintf(line, sizeof(line), "CAM_%c warmup %u/%u\r\n", name,
                     (unsigned)(warmup + 1U), (unsigned)APP_CAMERA_WARMUP_COUNT);
      debug_send(line);
      int warmup_result = camera_capture(&discard_jpeg, &discard_length);
      if (warmup_result != 0) {
        (void)snprintf(line, sizeof(line), "CAM_%c warmup failed rc=%d; continue\r\n",
                       name, warmup_result);
        debug_send(line);
      }
      HAL_Delay(APP_CAMERA_WARMUP_INTERVAL_MS);
      if (warmup + 1U == APP_CAMERA_WARMUP_COUNT / 2U)
        camera_log_state("warmup-mid");
    }
    camera_log_state("before-photo");
    (void)snprintf(line, sizeof(line), "CAM_%c VGA capture\r\n", name);
    debug_send(line);
    result = camera_capture(&jpeg, &length);
    /* DCMI is stopped by camera_capture. Keep its RAM buffer for SSDV. */
    camera_power_off();
    if (result != 0) {
      (void)snprintf(line, sizeof(line), "CAM_%c capture failed rc=%d; skip\r\n", name, result);
      debug_send(line);
      continue;
    }
    (void)snprintf(line, sizeof(line), "CAM_%c JPEG bytes=%lu SSDV image=%u\r\n",
                   name, (unsigned long)length, (unsigned)g_ssdv_image_id);
    debug_send(line);
    result = ssdv_send_image(jpeg, length);
    if (result != 0) {
      (void)snprintf(line, sizeof(line), "CAM_%c SSDV failed rc=%d\r\n", name, result);
      debug_send(line);
    }
  }
#if APP_TEST_FACTORY_IMAGE
  debug_send("TEST FACTORY JPEG VGA/SSDV (internal Flash)\r\n");
  int factory_result = ssdv_send_image_quality(g_factory_jpeg, g_factory_jpeg_len,
                                                APP_FACTORY_SSDV_QUALITY);
  if (factory_result != 0) {
    char line[64];
    (void)snprintf(line, sizeof(line), "FACTORY SSDV failed rc=%d\r\n", factory_result);
    debug_send(line);
  }
#endif
}

static void debug_reset_flags(void)
{
  char text[48];
  uint32_t flags = RCC->CSR;
  int count = snprintf(text, sizeof(text), "reset flags 0x%08lX\r\n", (unsigned long)flags);
  if (count > 0) {
    (void)HAL_UART_Transmit(&huart4, (uint8_t *)text, (uint16_t)count, 1000U);
  }
  __HAL_RCC_CLEAR_RESET_FLAGS();
}

void SystemClock_Config(void);

static void board_release_resets(void)
{
  HAL_Delay(10U);
  HAL_GPIO_WritePin(FLASH_RST_GPIO_Port, FLASH_RST_Pin, GPIO_PIN_SET);
  HAL_GPIO_WritePin(GNSS_RST_GPIO_Port, GNSS_RST_Pin, GPIO_PIN_SET);
  HAL_GPIO_WritePin(RSY_GPIO_Port, RSY_Pin, GPIO_PIN_RESET);
  HAL_Delay(1U);
  HAL_GPIO_WritePin(RSY_GPIO_Port, RSY_Pin, GPIO_PIN_SET);
  HAL_Delay(5U);
}

int main(void)
{
  HAL_Init();
  SystemClock_Config();
  MX_GPIO_Init();
  board_release_resets();
  MX_UART4_Init();
  debug_send("boot UART4\r\n");
  debug_reset_flags();
  MX_USART2_UART_Init();
  debug_send("USART2 up\r\n");
  if (APP_INDOOR_TEST_MODE == 0U) {
    while (Ubx_Init(&huart2) != 0) {
      debug_send("GNSS RX start failed; retry\r\n");
      HAL_Delay(1000U);
    }
    int gnss_config_rc;
    while ((gnss_config_rc = Ubx_ConfigureModule(gnss_poll)) != 0) {
      char line[96];
      (void)snprintf(line, sizeof(line), "GNSS M10 config failed rc=%d; retry, RF held off\r\n", gnss_config_rc);
      debug_send(line);
      uint32_t retry_start = HAL_GetTick();
      while (HAL_GetTick() - retry_start < 5000U) { gnss_poll(); HAL_Delay(1U); }
    }
    debug_send("GNSS M10 airborne<1g dynModel=6 ACK + RAM readback OK\r\n");
    gnss_wait_for_3d_fix();
  } else {
    debug_send("TEST GNSS bypass: no config, wait or forwarding\r\n");
  }
  MX_SPI2_Init();
  uint8_t radio_version;
  int radio_result = SX1278_APRS_Init(&radio_version);
  char radio_log[88];
  (void)snprintf(radio_log, sizeof(radio_log), "SX1278 version=0x%02X init=%d; APRS %s\r\n",
                 radio_version, radio_result, radio_result == 0 ? "ready" : "disabled");
  debug_send(radio_log);
  if (radio_result != 0) {
    char diagnostic[180];
    SX1278_APRS_ConfigError(diagnostic, sizeof(diagnostic));
    if (diagnostic[0] != 0) debug_send(diagnostic);
    SX1278_APRS_Diagnose(diagnostic, sizeof(diagnostic));
    debug_send(diagnostic);
    if (APP_INDOOR_TEST_MODE && APP_SX1278_NSS_TEST_ON_FAILURE &&
        (radio_result == -1 || radio_result == -2)) {
      SX1278_APRS_TestNss(debug_send);
    }
  }
  MX_I2C2_Init();
  debug_send("I2C2 up\r\n");
  MX_I2C3_Init();
  debug_send("I2C3 up\r\nWSPR 20m/73 ID13=03 freq=14097140 regular=:02 telemetry=:04\r\n");
  debug_send(APP_INDOOR_TEST_MODE != 0U ? "mode indoor test\r\n" : "mode flight\r\n");
  Sensors_Init();
  SensorSample initial_sensors;
  Sensors_Get(&initial_sensors);
  (void)snprintf(radio_log, sizeof(radio_log), "sensors INA3221@40=%d BME280@76=%d valid=0x%02X\r\n",
                 initial_sensors.ina_status, initial_sensors.bme_status, initial_sensors.valid);
  debug_send(radio_log);

  if (HAL_I2C_IsDeviceReady(&hi2c3, (uint16_t)(0x60U << 1), 3U, 100U) != HAL_OK) {
    debug_send("SI5351 no ACK; WSPR disabled\r\n");
  } else {
    si5351_Init(0);
    g_si5351_ready = 1U;
    debug_send("SI5351 up\r\n");
  }

  /* Optional startup-only carrier diagnostic. Disabled in the normal test sequence. */
  if ((APP_INDOOR_TEST_MODE != 0U) && (APP_SX1278_CARRIER_TEST_MS != 0U) && SX1278_APRS_Ready()) {
    char diagnostic[300];
    (void)snprintf(diagnostic, sizeof(diagnostic),
      "SX carrier test %luHz %udBm setting, %ums; observe spectrum now\r\n",
      (unsigned long)APP_APRS_DEFAULT_FREQ_HZ, APP_APRS_POWER_DBM, APP_SX1278_CARRIER_TEST_MS);
    debug_send(diagnostic);
    int carrier_rc = SX1278_APRS_CarrierTest(APP_APRS_DEFAULT_FREQ_HZ,
                                           APP_SX1278_CARRIER_TEST_MS, gnss_poll);
    SX1278_APRS_TxReport(diagnostic, sizeof(diagnostic));
    debug_send(diagnostic);
    (void)snprintf(diagnostic, sizeof(diagnostic), "SX carrier test ended rc=%d; RF off\r\n", carrier_rc);
    debug_send(diagnostic);
    Sensors_Poll();
  }
  if (APP_INDOOR_TEST_MODE != 0U) {
    /* Pure test startup: beacon, CAM_A, CAM_B, reference image, WSPR. */
    debug_send("TEST APRS beacon\r\n");
    (void)aprs_send_burst(15000U, 1U);
    indoor_camera_ssdv_sequence();
    debug_send("TEST WSPR regular\r\n");
    g_indoor_next_wspr_tick = HAL_GetTick() + APP_INDOOR_WSPR_PERIOD_MS;
    if (g_si5351_ready != 0U) wspr_send_regular();
    else {
      debug_send("TEST WSPR skipped: SI5351 unavailable\r\n");
    }
    /* The next indoor cycle sends telemetry after the first startup regular frame. */
    g_indoor_next_telemetry = 1U;
  } else {
    /* Flight mode must wait for its GNSS/UTC slot, including at startup. */
    (void)aprs_send_burst(15000U, 1U);
    g_indoor_next_telemetry = 0U;
    g_indoor_next_wspr_tick = HAL_GetTick();
  }
  g_indoor_next_aprs_tick = HAL_GetTick() + APP_INDOOR_APRS_PERIOD_MS;

  for (;;) {
    gnss_poll();
    /* Leave a guard before an imminent indoor WSPR start. */
    if (APP_INDOOR_TEST_MODE == 0U || g_si5351_ready == 0U ||
        (int32_t)(g_indoor_next_wspr_tick - HAL_GetTick()) > 200) Sensors_Poll();
    if (APP_INDOOR_TEST_MODE != 0U) {
      uint32_t now = HAL_GetTick();
      if (g_si5351_ready != 0U && (int32_t)(now - g_indoor_next_wspr_tick) >= 0) {
        /* Fallback deadline if encoding fails before wspr_transmit is called. */
        g_indoor_next_wspr_tick = now + APP_INDOOR_WSPR_PERIOD_MS;
        if (g_indoor_next_telemetry == 0U) wspr_send_regular();
        else wspr_send_telemetry();
        g_indoor_next_telemetry ^= 1U;
      } else if ((int32_t)(now - g_indoor_next_aprs_tick) >= 0) {
        /* Coalesce overdue packets; reserve time using the encoded bit count. */
        uint32_t budget = g_si5351_ready ? g_indoor_next_wspr_tick - now : 15000U;
        if (aprs_send_burst(budget, 0U)) {
          g_indoor_next_aprs_tick = HAL_GetTick() + APP_INDOOR_APRS_PERIOD_MS;
        }
      }
    } else {
      uint8_t telemetry;
      if (aprs_slot_due()) (void)aprs_send_burst(15000U, 0U);
      if (g_si5351_ready != 0U && slot_due(&telemetry)) {
        if (telemetry == 0U) wspr_send_regular();
        else wspr_send_telemetry();
      }
    }
    HAL_Delay(20U);
  }
}

void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK) Error_Handler();
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMBOOST = RCC_PLLMBOOST_DIV1;
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 20;
  RCC_OscInitStruct.PLL.PLLP = 2;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  RCC_OscInitStruct.PLL.PLLR = 1;
  RCC_OscInitStruct.PLL.PLLRGE = RCC_PLLVCIRANGE_1;
  RCC_OscInitStruct.PLL.PLLFRACN = 0;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) Error_Handler();
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK|RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2|RCC_CLOCKTYPE_PCLK3;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB3CLKDivider = RCC_HCLK_DIV1;
  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK) Error_Handler();
}

void Error_Handler(void)
{
  __disable_irq();
  while (1) { }
}
