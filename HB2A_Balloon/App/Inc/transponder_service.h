#ifndef HB2A_TRANSPONDER_SERVICE_H
#define HB2A_TRANSPONDER_SERVICE_H

#include "aprs_service.h"
#include "uplink_protocol.h"

#include <stdint.h>

#define TRANSPONDER_QUEUE_CAPACITY 4U
/* TRANPD uses APZDQ7 as both its validated path field and downlink alias. */
#define TRANSPONDER_AX25_PATH "APZDQ7"

typedef struct {
  char source[8];
  char destination[8];
  char path[8];
  uint8_t text[UPLINK_BCH_DATA_BYTES];
  uint8_t text_length;
} TransponderPacket;

typedef struct {
  TransponderPacket queue[TRANSPONDER_QUEUE_CAPACITY];
  uint8_t head;
  uint8_t count;
  uint32_t accepted_count;
  uint32_t dropped_count;
  uint32_t forwarded_count;
} TransponderService;

void transponder_service_init(TransponderService *service);
/* Returns 0 when accepted, -2 for a full queue, -3 for a non-TRANPD frame,
   and -4 when the path or source/destination callsign is invalid. */
int transponder_service_enqueue(TransponderService *service,
                                const UplinkCommand *command);
/* Send at most one queued packet when the APRS radio is available. */
void transponder_service_poll(TransponderService *service, AprsService *radio);

#endif
