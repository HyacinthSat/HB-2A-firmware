#include "app_flow.h"

#include <assert.h>
#include <stdio.h>

static void test_startup_and_gnss(void)
{
  AppFlow flow;
  app_flow_init(&flow);
  assert(app_flow_mode(&flow) == APP_MODE_BOOT);
  assert(app_flow_dispatch(&flow, APP_EVENT_BOOT_CHECK_DONE) == APP_ACTION_WAIT_GNSS);
  assert(app_flow_mode(&flow) == APP_MODE_GNSS_WAIT);
  assert(app_flow_dispatch(&flow, APP_EVENT_GNSS_TIMEOUT) == APP_ACTION_RESET_GNSS);
  assert(app_flow_dispatch(&flow, APP_EVENT_GNSS_TIMEOUT) == APP_ACTION_RESET_GNSS);
  assert(app_flow_dispatch(&flow, APP_EVENT_GNSS_TIMEOUT) == APP_ACTION_RESTART_MCU);
  assert(app_flow_mode(&flow) == APP_MODE_GNSS_WAIT);
  assert(app_flow_dispatch(&flow, APP_EVENT_GNSS_LOCKED) == APP_ACTION_ENTER_NORMAL);
  assert(app_flow_mode(&flow) == APP_MODE_NORMAL);
}

static void test_experiment_and_transfer(void)
{
  AppFlow flow;
  app_flow_init(&flow);
  (void)app_flow_dispatch(&flow, APP_EVENT_BOOT_CHECK_DONE);
  (void)app_flow_dispatch(&flow, APP_EVENT_GNSS_LOCKED);
  assert(app_flow_dispatch(&flow, APP_EVENT_VALID_EXPR) == APP_ACTION_ENTER_EXPERIMENT);
  assert(app_flow_dispatch(&flow, APP_EVENT_VALID_CAPTURE) == APP_ACTION_START_NEGOTIATION);
  assert(app_flow_mode(&flow) == APP_MODE_HS_NEGOTIATION);
  assert(app_flow_dispatch(&flow, APP_EVENT_HS_PARAMETERS_ACCEPTED) == APP_ACTION_SEND_CONNECTION_INFO);
  assert(app_flow_dispatch(&flow, APP_EVENT_HS_CONFIRM) == APP_ACTION_START_TRANSFER);
  assert(flow.transfer_active == 1U);
  assert(flow.experiment_timer_paused == 1U);
  assert(app_flow_dispatch(&flow, APP_EVENT_HS_COMPLETE) == APP_ACTION_NONE);
  assert(app_flow_mode(&flow) == APP_MODE_EXPERIMENT);
  assert(flow.transfer_active == 0U);
  assert(flow.experiment_timer_paused == 0U);
  assert(app_flow_dispatch(&flow, APP_EVENT_EXPERIMENT_TIMEOUT) == APP_ACTION_EXIT_EXPERIMENT);
  assert(app_flow_mode(&flow) == APP_MODE_NORMAL);
}

int main(void)
{
  test_startup_and_gnss();
  test_experiment_and_transfer();
  puts("app_flow tests passed");
  return 0;
}

