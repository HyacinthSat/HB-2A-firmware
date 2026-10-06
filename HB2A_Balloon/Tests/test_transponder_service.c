#include "transponder_service.h"

#include <assert.h>
#include <stdio.h>
#include <string.h>

static TransponderPacket sent;
static unsigned sends;

int aprs_service_send_transponder(AprsService *service,
                                  const char *source, const char *destination,
                                  const char *path, const uint8_t *info,
                                  size_t length)
{
  (void)service;
  strcpy(sent.source, source);
  strcpy(sent.destination, destination);
  strcpy(sent.path, path);
  sent.text_length = (uint8_t)length;
  memcpy(sent.text, info, length);
  ++sends;
  return 0;
}

int main(void)
{
  TransponderService service;
  UplinkCommand command = {0};
  AprsService radio = {.initialized = 1U};
  memcpy(command.data, "TRANPD", 6U);
  memcpy(command.data + 7U, "CALL1", 5U);
  memcpy(command.data + 14U, "APZDQ7", 6U);
  memcpy(command.data + 21U, "APZDQ7", 6U);
  command.block_count = 5U;
  memset(command.data + 28U, 0x20, UPLINK_BCH_DATA_BYTES);
  transponder_service_init(&service);
  assert(transponder_service_enqueue(&service, &command) == 0);
  transponder_service_poll(&service, &radio);
  assert(sends == 1U);
  assert(strcmp(sent.source, "CALL1") == 0);
  assert(strcmp(sent.destination, "APZDQ7") == 0);
  assert(strcmp(sent.path, TRANSPONDER_AX25_PATH) == 0);
  assert(sent.text_length == 0U);
  memcpy(command.data + 7U, "TOOLONG", 7U);
  assert(transponder_service_enqueue(&service, &command) == -4);
  puts("transponder service tests passed");
  return 0;
}
