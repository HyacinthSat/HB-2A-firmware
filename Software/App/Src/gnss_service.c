#include "gnss_service.h"

#include <string.h>

static GnssService *g_active_service;

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
}

int gnss_service_wait_for_lock(GnssService *service, uint32_t timeout_ms)
{
  uint32_t start;
  if (service == 0 || service != g_active_service ||
      service->state != GNSS_SERVICE_CONFIGURED) return -1;
  service->state = GNSS_SERVICE_WAITING;
  start = HAL_GetTick();
  while (service->solution.position_valid == 0U ||
         service->solution.utc_valid == 0U ||
         service->solution.fully_resolved == 0U) {
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
  valid = payload[11];
  solution->fix_type = payload[20];
  solution->position_valid = (payload[20] == 3U && (payload[21] & 1U) != 0U) ? 1U : 0U;
  solution->utc_valid = (valid & 0x03U) == 0x03U ? 1U : 0U;
  solution->fully_resolved = (valid & 0x04U) != 0U ? 1U : 0U;
  solution->year = read_u16(payload + 4U);
  solution->month = payload[6];
  solution->day = payload[7];
  solution->hour = payload[8];
  solution->minute = payload[9];
  solution->second = payload[10];

  if (solution->position_valid != 0U) {
    solution->longitude_e7 = read_i32(payload + 24U);
    solution->latitude_e7 = read_i32(payload + 28U);
    solution->height_msl_mm = read_i32(payload + 36U);
    solution->ground_speed_mm_s = read_u32(payload + 60U);
  }
  solution->update_tick = HAL_GetTick();
  ++g_active_service->nav_pvt_count;
  return (solution->position_valid != 0U && solution->utc_valid != 0U &&
          solution->fully_resolved != 0U) ? 1U : 0U;
}

