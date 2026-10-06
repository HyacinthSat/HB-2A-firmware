#include "transponder_service.h"

#include <string.h>

static void field(const UplinkCommand *command, uint8_t block,
                  char out[8])
{
  (void)uplink_field_text(command, block, out, 8U);
}

static int valid_call(const char *call)
{
  size_t i = 0U;
  size_t base = 0U;
  unsigned ssid = 0U;
  unsigned digits = 0U;
  if (call == NULL) return 0;
  while (call[i] != 0 && call[i] != '-') {
    if (!((call[i] >= 'A' && call[i] <= 'Z') ||
          (call[i] >= '0' && call[i] <= '9')) || ++base > 6U) return 0;
    ++i;
  }
  if (base == 0U) return 0;
  if (call[i] == '-') {
    ++i;
    while (call[i] != 0) {
      if (call[i] < '0' || call[i] > '9' || ++digits > 2U) return 0;
      ssid = ssid * 10U + (unsigned)(call[i++] - '0');
    }
    if (digits == 0U || ssid > 15U) return 0;
  }
  return 1;
}

void transponder_service_init(TransponderService *service)
{
  if (service != NULL) memset(service, 0, sizeof(*service));
}

int transponder_service_enqueue(TransponderService *service,
                                const UplinkCommand *command)
{
  TransponderPacket *packet;
  char source[8];
  char destination[8];
  char path[8];
  if (service == NULL || command == NULL || command->block_count != 5U ||
      memcmp(command->data, "TRANPD", 6U) != 0) return -3;
  field(command, 3U, path);
  if (strcmp(path, TRANSPONDER_AX25_PATH) != 0) return -4;
  field(command, 1U, source);
  field(command, 2U, destination);
  if (!valid_call(source) || !valid_call(destination)) return -4;
  if (service->count >= TRANSPONDER_QUEUE_CAPACITY) {
    ++service->dropped_count; /* The newest packet is discarded. */
    return -2;
  }
  packet = &service->queue[(service->head + service->count) %
                           TRANSPONDER_QUEUE_CAPACITY];
  memcpy(packet->source, source, sizeof(packet->source));
  memcpy(packet->destination, destination, sizeof(packet->destination));
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
