#include "transponder_service.h"

#include <string.h>

static void field(const UplinkCommand *command, uint8_t block,
                  char out[8])
{
  (void)uplink_field_text(command, block, out, 8U);
}

static void call_field(const UplinkCommand *command, uint8_t block,
                       char out[8])
{
  field(command, block, out);
  /* AX.25 allows six base-call characters plus an optional -SSID suffix.
     The uplink field itself is seven bytes, so an overlong base call is
     truncated before the common AX.25 address builder validates it. */
  {
    char *dash = strchr(out, '-');
    if (dash == NULL && strlen(out) > 6U) out[6] = 0;
    else if (dash != NULL && (size_t)(dash - out) > 6U) out[6] = 0;
  }
}

void transponder_service_init(TransponderService *service)
{
  if (service != NULL) memset(service, 0, sizeof(*service));
}

int transponder_service_enqueue(TransponderService *service,
                                const UplinkCommand *command)
{
  TransponderPacket *packet;
  char path[8];
  if (service == NULL || command == NULL || command->block_count != 5U ||
      memcmp(command->data, "TRANPD", 6U) != 0) return -3;
  field(command, 3U, path);
  if (strcmp(path, "HB-2A") != 0) return -4;
  if (service->count >= TRANSPONDER_QUEUE_CAPACITY) {
    ++service->dropped_count; /* The newest packet is discarded. */
    return -2;
  }
  packet = &service->queue[(service->head + service->count) %
                           TRANSPONDER_QUEUE_CAPACITY];
  call_field(command, 1U, packet->source);
  call_field(command, 2U, packet->destination);
  memcpy(packet->path, path, sizeof(packet->path));
  packet->text_length = UPLINK_BCH_DATA_BYTES;
  memcpy(packet->text, command->data + 4U * UPLINK_BCH_DATA_BYTES,
         packet->text_length);
  while (packet->text_length > 0U && packet->text[packet->text_length - 1U] == 0x20U)
    --packet->text_length;
  ++service->count;
  ++service->accepted_count;
  return 0;
}

void transponder_service_poll(TransponderService *service, AprsService *radio)
{
  TransponderPacket *packet;
  int result;
  if (service == NULL || radio == NULL || radio->initialized == 0U ||
      service->count == 0U) return;
  packet = &service->queue[service->head];
  result = aprs_service_send_transponder(radio, packet->source,
                                          packet->destination, packet->path,
                                          packet->text, packet->text_length);
  if (result != 0) return;
  ++service->forwarded_count;
  service->head = (uint8_t)((service->head + 1U) % TRANSPONDER_QUEUE_CAPACITY);
  --service->count;
}
