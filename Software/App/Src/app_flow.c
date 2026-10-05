#include "app_flow.h"

static AppAction enter_experiment(AppFlow *flow)
{
  flow->mode = APP_MODE_EXPERIMENT;
  flow->transfer_active = 0U;
  flow->experiment_timer_paused = 0U;
  return APP_ACTION_ENTER_EXPERIMENT;
}

void app_flow_init(AppFlow *flow)
{
  if (flow == 0) return;
  flow->mode = APP_MODE_BOOT;
  flow->gnss_timeout_count = 0U;
  flow->transfer_active = 0U;
  flow->experiment_timer_paused = 0U;
}

AppAction app_flow_dispatch(AppFlow *flow, AppEvent event)
{
  if (flow == 0) return APP_ACTION_NONE;

  switch (flow->mode) {
    case APP_MODE_BOOT:
      if (event == APP_EVENT_BOOT_CHECK_DONE) {
        flow->mode = APP_MODE_GNSS_WAIT;
        return APP_ACTION_WAIT_GNSS;
      }
      break;

    case APP_MODE_GNSS_WAIT:
      if (event == APP_EVENT_GNSS_LOCKED) {
        flow->gnss_timeout_count = 0U;
        flow->mode = APP_MODE_NORMAL;
        return APP_ACTION_ENTER_NORMAL;
      }
      if (event == APP_EVENT_GNSS_TIMEOUT) {
        ++flow->gnss_timeout_count;
        if (flow->gnss_timeout_count >= 3U) {
          return APP_ACTION_RESTART_MCU;
        }
        return APP_ACTION_RESET_GNSS;
      }
      break;

    case APP_MODE_NORMAL:
      if (event == APP_EVENT_VALID_EXPR) return enter_experiment(flow);
      break;

    case APP_MODE_EXPERIMENT:
      if (event == APP_EVENT_VALID_CAPTURE ||
          event == APP_EVENT_VALID_FILE_REQUEST) {
        flow->mode = APP_MODE_HS_NEGOTIATION;
        flow->experiment_timer_paused = 0U;
        return APP_ACTION_START_NEGOTIATION;
      }
      if (event == APP_EVENT_VALID_COMMAND) {
        flow->experiment_timer_paused = 0U;
        return APP_ACTION_NONE;
      }
      if (event == APP_EVENT_TRANSFER_STARTED) {
        flow->experiment_timer_paused = 1U;
        return APP_ACTION_NONE;
      }
      if (event == APP_EVENT_TRANSFER_FINISHED) {
        flow->experiment_timer_paused = 0U;
        return APP_ACTION_NONE;
      }
      if (event == APP_EVENT_EXPERIMENT_TIMEOUT) {
        flow->mode = APP_MODE_NORMAL;
        return APP_ACTION_EXIT_EXPERIMENT;
      }
      break;

    case APP_MODE_HS_NEGOTIATION:
      if (event == APP_EVENT_HS_PARAMETERS_ACCEPTED) {
        return APP_ACTION_SEND_CONNECTION_INFO;
      }
      if (event == APP_EVENT_HS_CONFIRM) {
        flow->mode = APP_MODE_HS_TRANSFER;
        flow->transfer_active = 1U;
        flow->experiment_timer_paused = 1U;
        return APP_ACTION_START_TRANSFER;
      }
      if (event == APP_EVENT_HS_CANCEL) {
        flow->mode = APP_MODE_EXPERIMENT;
        return APP_ACTION_CANCEL_TRANSFER;
      }
      if (event == APP_EVENT_EXPERIMENT_TIMEOUT) {
        flow->mode = APP_MODE_NORMAL;
        return APP_ACTION_EXIT_EXPERIMENT;
      }
      break;

    case APP_MODE_HS_TRANSFER:
      if (event == APP_EVENT_HS_COMPLETE ||
          event == APP_EVENT_TRANSFER_FINISHED) {
        flow->mode = APP_MODE_EXPERIMENT;
        flow->transfer_active = 0U;
        flow->experiment_timer_paused = 0U;
        return APP_ACTION_NONE;
      }
      if (event == APP_EVENT_HS_CANCEL) {
        flow->mode = APP_MODE_EXPERIMENT;
        flow->transfer_active = 0U;
        flow->experiment_timer_paused = 0U;
        return APP_ACTION_CANCEL_TRANSFER;
      }
      break;

    default:
      break;
  }

  return APP_ACTION_NONE;
}

AppMode app_flow_mode(const AppFlow *flow)
{
  return flow == 0 ? APP_MODE_BOOT : flow->mode;
}

