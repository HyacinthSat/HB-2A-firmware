#ifndef HB2A_GNSS_SERVICE_H
#define HB2A_GNSS_SERVICE_H

#include "ubx.h"

#include <stdint.h>

typedef enum {
  GNSS_SERVICE_IDLE = 0,
  GNSS_SERVICE_CONFIGURED,
  GNSS_SERVICE_WAITING,
  GNSS_SERVICE_LOCKED,
  GNSS_SERVICE_TIMEOUT,
  GNSS_SERVICE_ERROR,
} GnssServiceState;

typedef struct {
  uint8_t position_valid;
  uint8_t utc_valid;
  uint8_t fully_resolved;
  uint8_t fix_type;
  int32_t longitude_e7;
  int32_t latitude_e7;
  int32_t height_msl_mm;
  uint32_t ground_speed_mm_s;
  uint16_t year;
  uint8_t month;
  uint8_t day;
  uint8_t hour;
  uint8_t minute;
  uint8_t second;
  uint32_t update_tick;
} GnssSolution;

typedef struct {
  UART_HandleTypeDef *uart;
  GnssServiceState state;
  GnssSolution solution;
  uint32_t nav_pvt_count;
  uint32_t last_rx_errors;
} GnssService;

int gnss_service_init(GnssService *service, UART_HandleTypeDef *uart);
int gnss_service_configure(GnssService *service);
void gnss_service_poll(GnssService *service);
int gnss_service_wait_for_lock(GnssService *service, uint32_t timeout_ms);
void gnss_service_reset_state(GnssService *service);
GnssServiceState gnss_service_state(const GnssService *service);
const GnssSolution *gnss_service_solution(const GnssService *service);

#endif
