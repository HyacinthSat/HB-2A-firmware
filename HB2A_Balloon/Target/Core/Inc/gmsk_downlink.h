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

/* A high-speed group contains at most eight RS(255,223) codewords. */
#define GMSK_HIGH_SPEED_DATA_BYTES       223U
#define GMSK_HIGH_SPEED_CODEWORD_BYTES   255U
#define GMSK_HIGH_SPEED_MAX_CODEWORDS    8U
#define GMSK_HIGH_SPEED_MAX_INPUT_BYTES \
  (GMSK_HIGH_SPEED_CODEWORD_BYTES * GMSK_HIGH_SPEED_MAX_CODEWORDS)
#define GMSK_HIGH_SPEED_MAX_PACKED_BYTES 8192U

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

/*
 * Encode one to eight high-speed transfer-frame data areas.  Each input area
 * is 223 bytes (the four-byte TM primary header followed by SSDV data and
 * its CRC).  RS encoding is performed here, followed by symbol interleaving
 * at the negotiated depth, PN17, CSM and the selected CCSDS convolutional
 * code.  frames are contiguous, frame_count is the number of codewords in
 * this group, and intlv_id is one of 1, 2, 3, 4, 5 or 8.
 */
int gmsk_high_speed_encode(const uint8_t *frames, size_t frame_count,
                           uint8_t code_id, uint8_t intlv_id,
                           uint8_t *out, size_t out_capacity,
                           size_t *bit_count);

#endif
