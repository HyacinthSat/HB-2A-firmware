#include "app_main.h"

#include "app_flow.h"
#include "gnss_service.h"
#include "main.h"
#include "usart.h"

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

void app_main(void)
{
  AppFlow flow;
  GnssService gnss;

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

  for (;;) {
    gnss_service_poll(&gnss);
    HAL_Delay(10U);
  }
}

