#include "gnss_service.h"

#include <string.h>

static GnssService *g_active_service;
#define GNSS_FRESH_MS 3000U

static uint8_t month_days(uint16_t year, uint8_t month)
{
  static const uint8_t days[] = {31,28,31,30,31,30,31,31,30,31,30,31};
  if (month == 2U && year % 4U == 0U &&
      (year % 100U != 0U || year % 400U == 0U)) return 29U;
  return days[month - 1U];
}

static void advance_clock(GnssService *service)
{
  GnssSolution *s = &service->solution;
  uint32_t now = HAL_GetTick();
  if (s->utc_valid != 0U) {
    uint32_t seconds = (uint32_t)(now - service->clock_tick) / 1000U;
    service->clock_tick += seconds * 1000U;
    uint32_t day_seconds = s->hour * 3600U + s->minute * 60U + s->second + seconds;
    uint32_t days = day_seconds / 86400U;
    day_seconds %= 86400U;
    s->hour = (uint8_t)(day_seconds / 3600U);
    s->minute = (uint8_t)(day_seconds / 60U % 60U);
    s->second = (uint8_t)(day_seconds % 60U);
    while (days-- != 0U) {
      if (++s->day > month_days(s->year, s->month)) {
        s->day = 1U;
        if (++s->month > 12U) { s->month = 1U; ++s->year; }
      }
    }
  }
  if (service->nav_pvt_count != 0U && now - s->update_tick > GNSS_FRESH_MS) {
    s->position_valid = 0U;
    s->utc_holdover = s->utc_valid;
  }
}

static uint16_t read_u16(const uint8_t *p)
{
  return (uint16_t)p[0] | ((uint16_t)p[1] << 8U);
}

static uint32_t read_u32(const uint8_t *p)
{
  return (uint32_t)p[0] | ((uint32_t)p[1] << 8U) |
         ((uint32_t)p[2] << 16U) | ((uint32_t)p[3] << 24U);
}

static int32_t read_i32(const uint8_t *p)
{
  return (int32_t)read_u32(p);
}

static void gnss_service_driver_poll(void)
{
  if (g_active_service != 0) gnss_service_poll(g_active_service);
}

int gnss_service_init(GnssService *service, UART_HandleTypeDef *uart)
{
  if (service == 0 || uart == 0) return -1;
  memset(service, 0, sizeof(*service));
  service->uart = uart;
  service->state = GNSS_SERVICE_IDLE;
  g_active_service = service;
  return Ubx_Init(uart);
}

int gnss_service_configure(GnssService *service)
{
  if (service == 0 || service != g_active_service || service->uart == 0) return -1;
  if (Ubx_ConfigureModule(gnss_service_driver_poll) != 0) {
    service->state = GNSS_SERVICE_ERROR;
    return -1;
  }
  service->last_rx_errors = Ubx_RxErrors();
  service->state = GNSS_SERVICE_CONFIGURED;
  return 0;
}

void gnss_service_poll(GnssService *service)
{
  uint8_t byte;
  uint16_t budget = 256U;
  if (service == 0 || service != g_active_service) return;
  while (budget-- != 0U && Ubx_ReadByte(&byte) != 0) Ubx_OnByte(byte);
  service->last_rx_errors = Ubx_RxErrors();
  advance_clock(service);
}

int gnss_service_wait_for_lock(GnssService *service, uint32_t timeout_ms)
{
  uint32_t start;
  if (service == 0 || service != g_active_service ||
      service->state != GNSS_SERVICE_CONFIGURED) return -1;
  service->state = GNSS_SERVICE_WAITING;
  start = HAL_GetTick();
  gnss_service_poll(service);
  while (service->solution.position_valid == 0U ||
         service->solution.utc_valid == 0U ||
         service->solution.fully_resolved == 0U ||
         service->solution.utc_holdover != 0U) {
    gnss_service_poll(service);
    if (HAL_GetTick() - start >= timeout_ms) {
      service->state = GNSS_SERVICE_TIMEOUT;
      return -1;
    }
    HAL_Delay(1U);
  }
  service->state = GNSS_SERVICE_LOCKED;
  return 0;
}

void gnss_service_reset_state(GnssService *service)
{
  if (service == 0) return;
  memset(&service->solution, 0, sizeof(service->solution));
  service->nav_pvt_count = 0U;
  service->last_rx_errors = 0U;
  service->clock_tick = 0U;
  service->state = GNSS_SERVICE_IDLE;
}

GnssServiceState gnss_service_state(const GnssService *service)
{
  return service == 0 ? GNSS_SERVICE_ERROR : service->state;
}

const GnssSolution *gnss_service_solution(const GnssService *service)
{
  return service == 0 ? 0 : &service->solution;
}

uint8_t GNSS_OnUbxNavPvt(const uint8_t *payload, uint16_t length)
{
  GnssSolution *solution;
  uint8_t valid;

  if (g_active_service == 0 || payload == 0 || length != 92U) return 0U;
  solution = &g_active_service->solution;
  advance_clock(g_active_service);
  valid = payload[11];
  solution->fix_type = payload[20];
  solution->position_valid = (payload[20] == 3U && (payload[21] & 1U) != 0U) ? 1U : 0U;
  uint16_t year = read_u16(payload + 4U);
  if ((valid & 7U) == 7U && year >= 2000U && year <= 9999U &&
      payload[6] >= 1U && payload[6] <= 12U && payload[7] >= 1U &&
      payload[7] <= month_days(year, payload[6]) && payload[8] < 24U &&
      payload[9] < 60U && payload[10] < 60U) {
    solution->utc_valid = solution->fully_resolved = 1U;
    solution->utc_holdover = 0U;
    solution->year = year;
    solution->month = payload[6];
    solution->day = payload[7];
    solution->hour = payload[8];
    solution->minute = payload[9];
    solution->second = payload[10];
    g_active_service->clock_tick = HAL_GetTick();
  } else {
    solution->utc_holdover = solution->utc_valid;
  }

  if (solution->position_valid != 0U) {
    solution->position_known = 1U;
    solution->longitude_e7 = read_i32(payload + 24U);
    solution->latitude_e7 = read_i32(payload + 28U);
    solution->height_msl_mm = read_i32(payload + 36U);
    solution->ground_speed_mm_s = read_u32(payload + 60U);
  }
  solution->update_tick = HAL_GetTick();
  ++g_active_service->nav_pvt_count;
  return (solution->position_valid != 0U && solution->utc_valid != 0U &&
          solution->fully_resolved != 0U && solution->utc_holdover == 0U) ? 1U : 0U;
}

