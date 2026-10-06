#include "high_speed_service.h"

#include <stdio.h>
#include <string.h>

#define HS_PARAMETER_TIMEOUT_MS 10000UL
#define HS_CONFIRM_TIMEOUT_MS    5000UL
#define HS_MAX_CONNECTION_ATTEMPTS 3U

static int field(const UplinkCommand *command, uint8_t block,
                 char *out, size_t capacity)
{
  return (command != NULL && out != NULL &&
          uplink_field_text(command, block, out, capacity) != 0U) ? 0 : -1;
}

static int number(const char *text, unsigned *value)
{
  unsigned result = 0U;
  size_t i;
  if (text == NULL || text[0] == 0 || value == NULL) return -1;
  for (i = 0U; text[i] != 0; ++i) {
    if (text[i] < '0' || text[i] > '9') return -1;
    result = result * 10U + (unsigned)(text[i] - '0');
    if (result > 255U) return -1;
  }
  *value = result;
  return 0;
}

static int number16(const char *text, uint16_t *value)
{
  unsigned result = 0U;
  size_t i;
  if (text == NULL || text[0] == 0 || value == NULL) return -1;
  for (i = 0U; text[i] != 0; ++i) {
    if (text[i] < '0' || text[i] > '9') return -1;
    result = result * 10U + (unsigned)(text[i] - '0');
    if (result > 65535U) return -1;
  }
  *value = (uint16_t)result;
  return 0;
}

static int intlv_bit(uint8_t id)
{
  switch (id) {
    case 1U: return 0;
    case 2U: return 1;
    case 3U: return 2;
    case 4U: return 3;
    case 5U: return 4;
    case 8U: return 5;
    default: return -1;
  }
}

static int parameters_valid(const HighSpeedParameters *parameters)
{
  int bit;
  if (parameters == NULL || parameters->rate_id > 5U ||
      parameters->code_id > 3U) return 0;
  bit = intlv_bit(parameters->intlv_id);
  return bit >= 0 &&
         (HS_SUPPORTED_RATE_MASK & (1U << parameters->rate_id)) != 0U &&
         (HS_SUPPORTED_CODE_MASK & (1U << parameters->code_id)) != 0U &&
         (HS_SUPPORTED_INTLV_MASK & (1U << bit)) != 0U;
}

static void event(HighSpeedService *service, HighSpeedEvent value)
{
  if (service->event_count >= (uint8_t)(sizeof(service->events) /
                                         sizeof(service->events[0]))) return;
  service->events[(service->event_head + service->event_count) %
                  (sizeof(service->events) / sizeof(service->events[0]))] = value;
  ++service->event_count;
}

static void reject(HighSpeedService *service, const char *reason)
{
  (void)snprintf(service->reject_reason, sizeof(service->reject_reason),
                 "%s", reason == NULL ? "PARAM" : reason);
  event(service, HS_EVENT_SEND_REJECT);
}

static int parse_parameters(const UplinkCommand *command,
                            HighSpeedParameters *parameters)
{
  char text[8];
  unsigned value;
  if (command == NULL || parameters == NULL || command->block_count != 4U ||
      (memcmp(command->data, "HSPAR", 5U) != 0 &&
       memcmp(command->data, "CHPRAM", 6U) != 0)) return -1;
  if (field(command, 1U, text, sizeof(text)) || number(text, &value)) return -1;
  parameters->rate_id = (uint8_t)value;
  if (field(command, 2U, text, sizeof(text)) || number(text, &value)) return -1;
  parameters->code_id = (uint8_t)value;
  if (field(command, 3U, text, sizeof(text)) || number(text, &value)) return -1;
  parameters->intlv_id = (uint8_t)value;
  return parameters_valid(parameters) ? 0 : -1;
}

static int is_command(const UplinkCommand *command, const char *name,
                      uint8_t blocks)
{
  return command != NULL && command->block_count == blocks &&
         memcmp(command->data, name, strlen(name)) == 0;
}

void high_speed_service_init(HighSpeedService *service)
{
  if (service != NULL) memset(service, 0, sizeof(*service));
}

int high_speed_service_start(HighSpeedService *service,
                             const UplinkCommand *command,
                             uint32_t now_ms)
{
  char text[8];
  unsigned value;
  if (service == NULL || command == NULL || service->state != HS_STATE_IDLE)
    return -1;
  memset(service, 0, sizeof(*service));
  if (is_command(command, "CAPTR", 5U)) {
    if (field(command, 1U, text, sizeof(text)) || text[0] == 0) {
      reject(service, "PARAM");
      return -2;
    }
    service->camera = (uint8_t)text[0];
    if (service->camera != 'A' && service->camera != 'B') {
      reject(service, "PARAM");
      return -2;
    }
    if (field(command, 2U, text, sizeof(text)) ||
        number16(text, &service->width)) {
      reject(service, "PARAM");
      return -2;
    }
    if (field(command, 3U, text, sizeof(text)) ||
        number16(text, &service->height)) {
      reject(service, "PARAM");
      return -2;
    }
    if (field(command, 4U, text, sizeof(text)) || number(text, &value) || value > 7U) {
      reject(service, "PARAM");
      return -2;
    }
    service->quality = (int8_t)value;
    /* SSDV requires MCU-aligned dimensions.  Do not tie the command to the
       current 256x192 test preset; the selected camera driver decides later
       whether a particular aligned mode is available in hardware. */
    if (service->width == 0U || service->height == 0U ||
        (service->width & 15U) != 0U || (service->height & 15U) != 0U) {
      reject(service, "PARAM");
      return -2;
    }
    service->request = HS_REQUEST_CAPTURE;
  } else if (is_command(command, "REQF", 2U)) {
    if (field(command, 1U, text, sizeof(text)) || number(text, &value)) {
      reject(service, "NOFILE");
      return -2;
    }
    service->request = HS_REQUEST_FILE;
    service->file_index = (uint8_t)value;
  } else {
    return -1;
  }
  service->state = HS_STATE_WAIT_PARAMETERS;
  service->deadline_ms = now_ms + HS_PARAMETER_TIMEOUT_MS;
  event(service, HS_EVENT_SEND_ACK);
  event(service, HS_EVENT_SEND_SUPPORT);
  return 0;
}

int high_speed_service_command(HighSpeedService *service,
                               const UplinkCommand *command,
                               uint32_t now_ms)
{
  HighSpeedParameters parameters;
  if (service == NULL || command == NULL) return -1;
  if (is_command(command, "CANCEL", 1U)) {
    if (service->state == HS_STATE_IDLE) return -2;
    service->state = HS_STATE_IDLE;
    service->event_count = 0U;
    service->event_head = 0U;
    event(service, HS_EVENT_CANCELLED);
    return 0;
  }
  if (service->state == HS_STATE_WAIT_PARAMETERS &&
      is_command(command, "HSPAR", 4U)) {
    if (parse_parameters(command, &parameters) != 0) {
      reject(service, "PARAM");
      service->deadline_ms = now_ms + HS_PARAMETER_TIMEOUT_MS;
      return -3;
    }
    service->parameters = parameters;
    service->state = HS_STATE_WAIT_CONFIRM;
    service->connection_attempts = 1U;
    service->deadline_ms = now_ms + HS_CONFIRM_TIMEOUT_MS;
    event(service, HS_EVENT_SEND_CONNECTION);
    return 0;
  }
  if (service->state == HS_STATE_WAIT_CONFIRM &&
      (is_command(command, "CHPRAM", 4U) ||
       is_command(command, "HSPAR", 4U))) {
    if (parse_parameters(command, &parameters) != 0) {
      reject(service, "PARAM");
      return -3;
    }
    service->parameters = parameters;
    service->state = HS_STATE_WAIT_CONFIRM;
    service->connection_attempts = 1U;
    service->deadline_ms = now_ms + HS_CONFIRM_TIMEOUT_MS;
    event(service, HS_EVENT_SEND_CONNECTION);
    return 0;
  }
  if (service->state == HS_STATE_WAIT_CONFIRM && is_command(command, "CFRM", 1U)) {
    service->state = HS_STATE_TRANSFER;
    service->deadline_ms = 0U;
    event(service, HS_EVENT_START_TRANSFER);
    return 0;
  }
  return -2;
}

int high_speed_service_poll(HighSpeedService *service, uint32_t now_ms)
{
  if (service == NULL || service->event_count != 0U ||
      service->state == HS_STATE_IDLE || service->state == HS_STATE_TRANSFER)
    return service != NULL && service->event_count != 0U;
  if ((int32_t)(now_ms - service->deadline_ms) < 0) return 0;
  if (service->state == HS_STATE_WAIT_PARAMETERS) {
    service->state = HS_STATE_IDLE;
    event(service, HS_EVENT_TIMEOUT);
    return 1;
  }
  if (service->connection_attempts < HS_MAX_CONNECTION_ATTEMPTS) {
    ++service->connection_attempts;
    service->deadline_ms = now_ms + HS_CONFIRM_TIMEOUT_MS;
    event(service, HS_EVENT_SEND_CONNECTION);
    return 1;
  }
  service->state = HS_STATE_IDLE;
  event(service, HS_EVENT_TIMEOUT);
  return 1;
}

void high_speed_service_control_sent(HighSpeedService *service,
                                     HighSpeedEvent value, uint32_t now_ms)
{
  if (service == NULL) return;
  if (value == HS_EVENT_SEND_SUPPORT && service->state == HS_STATE_WAIT_PARAMETERS)
    service->deadline_ms = now_ms + HS_PARAMETER_TIMEOUT_MS;
  if (value == HS_EVENT_SEND_CONNECTION && service->state == HS_STATE_WAIT_CONFIRM)
    service->deadline_ms = now_ms + HS_CONFIRM_TIMEOUT_MS;
}

HighSpeedEvent high_speed_service_take_event(HighSpeedService *service)
{
  HighSpeedEvent value;
  if (service == NULL || service->event_count == 0U) return HS_EVENT_NONE;
  value = service->events[service->event_head];
  service->event_head = (uint8_t)((service->event_head + 1U) %
                                  (sizeof(service->events) / sizeof(service->events[0])));
  --service->event_count;
  return value;
}

HighSpeedState high_speed_service_state(const HighSpeedService *service)
{
  return service == NULL ? HS_STATE_IDLE : service->state;
}

void high_speed_service_transfer_done(HighSpeedService *service,
                                      uint8_t cancelled)
{
  if (service == NULL || service->state != HS_STATE_TRANSFER) return;
  service->state = HS_STATE_IDLE;
  event(service, cancelled != 0U ? HS_EVENT_CANCELLED : HS_EVENT_TRANSFER_DONE);
}

int high_speed_service_event_text(const HighSpeedService *service,
                                  HighSpeedEvent value,
                                  char *out, uint32_t capacity)
{
  int written;
  if (service == NULL || out == NULL || capacity == 0U) return -1;
  switch (value) {
    case HS_EVENT_SEND_ACK:
      written = snprintf(out, capacity, "ACK,%s",
                         service->request == HS_REQUEST_CAPTURE ? "CAPTR" : "REQF");
      break;
    case HS_EVENT_SEND_SUPPORT:
      written = snprintf(out, capacity, "HSSUP,%02X,%02X,%02X",
                         HS_SUPPORTED_RATE_MASK, HS_SUPPORTED_CODE_MASK,
                         HS_SUPPORTED_INTLV_MASK);
      break;
    case HS_EVENT_SEND_CONNECTION:
      written = snprintf(out, capacity, "HSCON,%u,%u,%u",
                         service->parameters.rate_id, service->parameters.code_id,
                         service->parameters.intlv_id);
      break;
    case HS_EVENT_SEND_REJECT:
      written = snprintf(out, capacity, "HSREJ,%s", service->reject_reason);
      break;
    case HS_EVENT_CANCELLED: written = snprintf(out, capacity, "ACK,CANCEL"); break;
    default: return 0;
  }
  return written < 0 || (uint32_t)written >= capacity ? -2 : written;
}
