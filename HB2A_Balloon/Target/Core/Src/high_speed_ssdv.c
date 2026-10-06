#include "high_speed_ssdv.h"

#include <string.h>

#define SSDV_IMAGE_OFFSET      6U
#define SSDV_PACKET_ID_OFFSET  7U
#define SSDV_WIDTH_OFFSET      9U
#define SSDV_HEIGHT_OFFSET     10U
#define SSDV_FLAGS_OFFSET      11U
#define SSDV_MCU_OFFSET        12U
#define SSDV_PAYLOAD_OFFSET    15U
#define SSDV_PAYLOAD_BYTES     205U

static uint8_t master_count;
static uint8_t virtual_count;

static uint32_t crc32_update(uint32_t crc, const uint8_t *data, size_t length)
{
  while (length-- != 0U) {
    crc ^= *data++;
    for (unsigned bit = 0U; bit < 8U; ++bit)
      crc = (crc & 1U) != 0U ? (crc >> 1U) ^ 0xEDB88320UL : crc >> 1U;
  }
  return crc;
}

void high_speed_ssdv_reset_counters(void)
{
  master_count = 0U;
  virtual_count = 0U;
}

int high_speed_ssdv_build_frame(const uint8_t *ssdv_packet, size_t packet_length,
                                const char *callsign,
                                uint8_t *frame, size_t frame_capacity)
{
  uint32_t header;
  uint32_t dimensions;
  uint32_t crc;
  size_t callsign_length;
  if (ssdv_packet == NULL || callsign == NULL || frame == NULL ||
      packet_length < HIGH_SPEED_SSDV_PACKET_BYTES ||
      frame_capacity < HIGH_SPEED_SSDV_DATA_BYTES) return -1;
  callsign_length = strlen(callsign);
  if (callsign_length == 0U || callsign_length > 6U) return -1;

  /* Version 00, spacecraft 0, VCID 2, no secondary header.  The final two
     bytes are the CCSDS master and virtual-channel frame counters. */
  header = (2UL << 17U) | ((uint32_t)master_count << 8U) | virtual_count;
  frame[0] = (uint8_t)(header >> 24U);
  frame[1] = (uint8_t)(header >> 16U);
  frame[2] = (uint8_t)(header >> 8U);
  frame[3] = (uint8_t)header;
  frame[4] = ssdv_packet[SSDV_IMAGE_OFFSET];
  frame[5] = ssdv_packet[SSDV_PACKET_ID_OFFSET];
  frame[6] = ssdv_packet[SSDV_PACKET_ID_OFFSET + 1U];
  dimensions = ((uint32_t)ssdv_packet[SSDV_WIDTH_OFFSET] << 12U) |
               (uint32_t)ssdv_packet[SSDV_HEIGHT_OFFSET];
  frame[7] = (uint8_t)(dimensions >> 16U);
  frame[8] = (uint8_t)(dimensions >> 8U);
  frame[9] = (uint8_t)dimensions;
  /* SSDV already stores quality-4 in bits 7..5.  Copy that wire field and
     retain EOI plus the 2-bit MCU sampling mode. */
  frame[10] = ssdv_packet[SSDV_FLAGS_OFFSET];
  frame[11] = ssdv_packet[SSDV_MCU_OFFSET];
  frame[12] = ssdv_packet[SSDV_MCU_OFFSET + 1U];
  frame[13] = ssdv_packet[SSDV_MCU_OFFSET + 2U];
  memcpy(frame + 14U, ssdv_packet + SSDV_PAYLOAD_OFFSET,
         SSDV_PAYLOAD_BYTES);

  /* The callsign is deliberately outside the transmitted bytes. */
  crc = crc32_update(0xFFFFFFFFUL, (const uint8_t *)callsign,
                     callsign_length);
  crc = crc32_update(crc, frame + 4U, 215U);
  crc = ~crc;
  frame[219] = (uint8_t)(crc >> 24U);
  frame[220] = (uint8_t)(crc >> 16U);
  frame[221] = (uint8_t)(crc >> 8U);
  frame[222] = (uint8_t)crc;
  ++master_count;
  ++virtual_count;
  return 0;
}
