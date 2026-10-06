#ifndef HB2A_HIGH_SPEED_SSDV_H
#define HB2A_HIGH_SPEED_SSDV_H

#include <stddef.h>
#include <stdint.h>

#define HIGH_SPEED_SSDV_PACKET_BYTES 256U
#define HIGH_SPEED_SSDV_DATA_BYTES   223U

/* Build the 223-byte high-speed transfer-frame data area from one raw SSDV
   packet.  The SSDV encoder callsign is used for CRC32 only. */
int high_speed_ssdv_build_frame(const uint8_t *ssdv_packet, size_t packet_length,
                                const char *callsign,
                                uint8_t *frame, size_t frame_capacity);

void high_speed_ssdv_reset_counters(void);

#endif
