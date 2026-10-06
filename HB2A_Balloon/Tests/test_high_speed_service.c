#include "high_speed_service.h"

#include <assert.h>
#include <stdio.h>
#include <string.h>

static void put_field(UplinkCommand *command, unsigned block, const char *text)
{
  size_t length = strlen(text);
  if (length > UPLINK_BCH_DATA_BYTES) length = UPLINK_BCH_DATA_BYTES;
  memcpy(command->data + block * UPLINK_BCH_DATA_BYTES, text, length);
  memset(command->data + block * UPLINK_BCH_DATA_BYTES + length, 0x20,
         UPLINK_BCH_DATA_BYTES - length);
}

static UplinkCommand command(unsigned blocks, const char *name,
                             const char *a, const char *b, const char *c,
                             const char *d)
{
  UplinkCommand result = {0};
  result.block_count = (uint8_t)blocks;
  put_field(&result, 0U, name);
  if (a != NULL) put_field(&result, 1U, a);
  if (b != NULL) put_field(&result, 2U, b);
  if (c != NULL) put_field(&result, 3U, c);
  if (d != NULL) put_field(&result, 4U, d);
  return result;
}

int main(void)
{
  HighSpeedService service;
  HighSpeedParameters parameters;
  UplinkCommand captr = command(5U, "CAPTR", "A", "256", "192", "5");
  UplinkCommand hspar = command(4U, "HSPAR", "5", "3", "8", NULL);
  UplinkCommand cfrm = command(1U, "CFRM", NULL, NULL, NULL, NULL);
  UplinkCommand bad_captr = command(5U, "CAPTR", "C", "256", "192", "5");
  char text[32];

  high_speed_service_init(&service);
  assert(high_speed_service_start(&service, &captr, 100U) == 0);
  assert(high_speed_service_state(&service) == HS_STATE_WAIT_PARAMETERS);
  assert(high_speed_service_take_event(&service) == HS_EVENT_SEND_ACK);
  assert(high_speed_service_event_text(&service, HS_EVENT_SEND_SUPPORT,
                                       text, sizeof(text)) > 0);
  assert(strcmp(text, "HSSUP,3F,0F,3F") == 0);
  assert(high_speed_service_take_event(&service) == HS_EVENT_SEND_SUPPORT);

  assert(high_speed_service_command(&service, &hspar, 200U) == 0);
  assert(high_speed_service_state(&service) == HS_STATE_WAIT_CONFIRM);
  assert(high_speed_service_take_event(&service) == HS_EVENT_SEND_CONNECTION);
  assert(high_speed_service_event_text(&service, HS_EVENT_SEND_CONNECTION,
                                       text, sizeof(text)) > 0);
  assert(strcmp(text, "HSCON,5,3,8") == 0);
  assert(high_speed_service_poll(&service, 5199U) == 0);
  assert(high_speed_service_poll(&service, 5200U) == 1);
  assert(high_speed_service_take_event(&service) == HS_EVENT_SEND_CONNECTION);
  assert(high_speed_service_command(&service, &cfrm, 5300U) == 0);
  assert(high_speed_service_state(&service) == HS_STATE_TRANSFER);
  assert(high_speed_service_take_event(&service) == HS_EVENT_START_TRANSFER);
  high_speed_service_transfer_done(&service, 0U);
  assert(high_speed_service_state(&service) == HS_STATE_IDLE);
  assert(high_speed_service_take_event(&service) == HS_EVENT_TRANSFER_DONE);

  high_speed_service_init(&service);
  assert(high_speed_service_start(&service, &bad_captr, 0U) == -2);
  assert(high_speed_service_take_event(&service) == HS_EVENT_SEND_REJECT);

  high_speed_service_init(&service);
  assert(high_speed_service_start(&service, &captr, 0U) == 0);
  (void)high_speed_service_take_event(&service);
  (void)high_speed_service_take_event(&service);
  assert(high_speed_service_poll(&service, 10000U) == 1);
  assert(high_speed_service_state(&service) == HS_STATE_IDLE);
  assert(high_speed_service_take_event(&service) == HS_EVENT_TIMEOUT);

  high_speed_service_init(&service);
  assert(high_speed_service_start(&service, &captr, 0U) == 0);
  (void)high_speed_service_take_event(&service);
  (void)high_speed_service_take_event(&service);
  assert(high_speed_service_command(&service, &hspar, 1U) == 0);
  (void)high_speed_service_take_event(&service);
  assert(high_speed_service_poll(&service, 5001U) == 1);
  (void)high_speed_service_take_event(&service);
  assert(high_speed_service_poll(&service, 10001U) == 1);
  (void)high_speed_service_take_event(&service);
  assert(high_speed_service_poll(&service, 15001U) == 1);
  assert(high_speed_service_state(&service) == HS_STATE_IDLE);
  assert(high_speed_service_take_event(&service) == HS_EVENT_TIMEOUT);
  puts("high speed service tests passed");
  (void)parameters;
  return 0;
}
