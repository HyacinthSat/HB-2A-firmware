#include "gnss_service.h"

#include <assert.h>
#include <stdio.h>
#include <string.h>

static UART_HandleTypeDef uart;
static uint8_t *rx_byte;
static uint32_t tick;
static unsigned abort_count;
static int abort_failure;

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *handle);

HAL_StatusTypeDef HAL_UART_AbortReceive(UART_HandleTypeDef *handle)
{
  ++abort_count;
  if (abort_failure) return HAL_ERROR;
  handle->RxState = HAL_UART_STATE_READY;
  rx_byte = NULL;
  return HAL_OK;
}

HAL_StatusTypeDef HAL_UART_Receive_IT(UART_HandleTypeDef *handle, uint8_t *byte, uint16_t length)
{
  assert(handle == &uart && length == 1U);
  if (handle->RxState != HAL_UART_STATE_READY) return HAL_BUSY;
  handle->RxState = HAL_UART_STATE_BUSY_RX;
  rx_byte = byte;
  return HAL_OK;
}

HAL_StatusTypeDef HAL_UART_Transmit(UART_HandleTypeDef *handle, const uint8_t *data,
                                    uint16_t length, uint32_t timeout)
{
  (void)data;
  (void)length;
  (void)timeout;
  assert(handle == &uart);
  return HAL_OK;
}

uint32_t HAL_GetTick(void) { return tick; }
void HAL_Delay(uint32_t milliseconds) { tick += milliseconds; }

static void inject(uint8_t byte)
{
  assert(rx_byte != 0);
  *rx_byte = byte;
  uart.RxState = HAL_UART_STATE_READY;
  HAL_UART_RxCpltCallback(&uart);
}

static void send_nav_pvt(const uint8_t *payload)
{
  uint8_t frame[100] = {0xB5U, 0x62U, 0x01U, 0x07U, 92U, 0U};
  uint8_t ck_a = 0U;
  uint8_t ck_b = 0U;
  memcpy(frame + 6U, payload, 92U);
  for (unsigned i = 2U; i < 98U; ++i) {
    ck_a = (uint8_t)(ck_a + frame[i]);
    ck_b = (uint8_t)(ck_b + ck_a);
  }
  frame[98] = ck_a;
  frame[99] = ck_b;
  for (unsigned i = 0U; i < sizeof(frame); ++i) inject(frame[i]);
}

static void put_u16(uint8_t *p, uint16_t value)
{
  p[0] = (uint8_t)value;
  p[1] = (uint8_t)(value >> 8U);
}

static void put_i32(uint8_t *p, int32_t value)
{
  uint32_t v = (uint32_t)value;
  p[0] = (uint8_t)v;
  p[1] = (uint8_t)(v >> 8U);
  p[2] = (uint8_t)(v >> 16U);
  p[3] = (uint8_t)(v >> 24U);
}

static void put_u32(uint8_t *p, uint32_t value)
{
  put_i32(p, (int32_t)value);
}

int main(void)
{
  GnssService service;
  uint8_t payload[92] = {0};
  const GnssSolution *solution;

  assert(gnss_service_init(&service, &uart) == 0);
  assert(service.state == GNSS_SERVICE_IDLE);
  inject(0xB5U);
  assert(gnss_service_init(&service, &uart) == 0);
  assert(gnss_service_init(&service, &uart) == 0);
  assert(abort_count == 3U && uart.RxState == HAL_UART_STATE_BUSY_RX);
  abort_failure = 1;
  assert(gnss_service_init(&service, &uart) != 0);
  abort_failure = 0;
  assert(gnss_service_init(&service, &uart) == 0);
  assert(gnss_service_configure(&service) != 0);
  /* Configuration needs a real M10 response; parser behavior is tested below. */
  service.state = GNSS_SERVICE_CONFIGURED;

  /* A corrupt oversized length must be rejected immediately so the next
     valid NAV-PVT frame is still found. */
  {
    const uint8_t bad_header[] = {0xB5U, 0x62U, 0x01U, 0x07U, 0xFFU, 0xFFU};
    for (unsigned i = 0U; i < sizeof(bad_header); ++i) inject(bad_header[i]);
  }

  payload[20] = 3U;
  payload[21] = 1U;
  payload[11] = 7U;
  put_u16(payload + 4U, 2026U);
  payload[6] = 10U; payload[7] = 6U;
  payload[8] = 12U; payload[9] = 34U; payload[10] = 56U;
  put_i32(payload + 24U, 1124583333);
  put_i32(payload + 28U, 380208333);
  put_i32(payload + 36U, 850000);
  put_u32(payload + 60U, 18520U);
  send_nav_pvt(payload);
  gnss_service_poll(&service);
  solution = gnss_service_solution(&service);
  assert(service.nav_pvt_count == 1U);
  assert(solution->position_valid == 1U);
  assert(solution->utc_valid == 1U);
  assert(solution->fully_resolved == 1U);
  assert(solution->longitude_e7 == 1124583333);
  assert(solution->latitude_e7 == 380208333);
  assert(solution->height_msl_mm == 850000);
  assert(solution->ground_speed_mm_s == 18520U);
  assert(solution->year == 2026U && solution->hour == 12U);

  tick += 600000U;
  gnss_service_poll(&service);
  assert(solution->position_valid == 0U && solution->utc_holdover == 1U);
  assert(solution->utc_valid && solution->minute == 44U && solution->second == 56U);
  payload[11] = 0U;
  send_nav_pvt(payload);
  gnss_service_poll(&service);
  assert(solution->minute == 44U && solution->utc_holdover);
  payload[11] = 7U;
  put_u16(payload + 4U, 2028U);
  payload[6] = 2U; payload[7] = 28U;
  payload[8] = 23U; payload[9] = 59U; payload[10] = 59U;
  tick = UINT32_MAX - 499U;
  send_nav_pvt(payload); gnss_service_poll(&service);
  tick += 1500U; gnss_service_poll(&service);
  assert(solution->month == 2U && solution->day == 29U && solution->second == 0U);
  tick += 500U; gnss_service_poll(&service);
  assert(solution->second == 1U);
  tick += 86400000U; gnss_service_poll(&service);
  assert(solution->month == 3U && solution->day == 1U && solution->second == 1U);
  payload[6]=12U; payload[7]=31U;
  send_nav_pvt(payload); gnss_service_poll(&service);
  tick += 1000U; gnss_service_poll(&service);
  assert(solution->year == 2029U && solution->month == 1U && solution->day == 1U);
  payload[6]=13U;
  send_nav_pvt(payload); gnss_service_poll(&service);
  assert(solution->month == 1U && solution->utc_holdover);
  payload[6]=10U; payload[7]=6U; payload[8]=12U; payload[9]=34U; payload[10]=56U;
  send_nav_pvt(payload); gnss_service_poll(&service);
  assert(!solution->utc_holdover && solution->position_valid && solution->minute == 34U);

  puts("gnss_service tests passed");
  return 0;
}

