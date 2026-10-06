#ifndef HB2A_GMSK_DOWNLINK_H
#define HB2A_GMSK_DOWNLINK_H

#include <stddef.h>
#include <stdint.h>

/* Conventional balloon GMSK downlink: one 255-byte TM RS codeword. */
#define GMSK_DOWNLINK_TEXT_BYTES 219U
#define GMSK_DOWNLINK_RS_DATA_BYTES 223U
#define GMSK_DOWNLINK_RS_CODE_BYTES 255U
#define GMSK_DOWNLINK_PREAMBLE_BYTES 16U
#define GMSK_DOWNLINK_MAX_BITS \
  ((GMSK_DOWNLINK_PREAMBLE_BYTES * 8U) + \
   ((4U + GMSK_DOWNLINK_RS_CODE_BYTES) * 8U + 6U) * 2U)
#define GMSK_DOWNLINK_MAX_PACKED_BYTES \
  ((GMSK_DOWNLINK_MAX_BITS + 7U) / 8U)

/* Reset the CCSDS master and virtual-channel counters. Intended for tests. */
void gmsk_downlink_reset_counters(void);

/*
 * Encode one conventional GMSK downlink message.
 *
 * The input is ASCII text. It is truncated or space padded to 219 bytes,
 * then preceded by the four-byte fixed primary header (VCID 1), RS(255,223)
 * encoded, PN17 scrambled, prefixed with 0x1ACFFC1D, and convolutionally
 * encoded using CCSDS CodeID 1 (K=7, rate 1/2, C2 inverted, terminated).
 * The first 16 bytes of the returned MSB-first bitstream are raw 0x55.
 *
 * Returns 0 on success. bit_count reports the number of valid bits in out.
 */
int gmsk_downlink_encode(const uint8_t *text, size_t text_length,
                         uint8_t *out, size_t out_capacity,
                         size_t *bit_count);

#endif
