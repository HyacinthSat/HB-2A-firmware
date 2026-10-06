#ifndef HB2A_UPLINK_PROTOCOL_H
#define HB2A_UPLINK_PROTOCOL_H

#include <stddef.h>
#include <stdint.h>

#define UPLINK_BCH_DATA_BYTES 7U
#define UPLINK_BCH_CODE_BYTES 8U
#define UPLINK_MAX_BLOCKS 8U
#define UPLINK_MAX_DATA_BYTES (UPLINK_MAX_BLOCKS * UPLINK_BCH_DATA_BYTES)
#define UPLINK_MAX_RAW_BITS ((UPLINK_MAX_BLOCKS * UPLINK_BCH_CODE_BYTES + 2U) * 8U)

typedef struct {
  uint8_t block_count;
  uint8_t data[UPLINK_MAX_DATA_BYTES];
  uint16_t crc;
} UplinkCommand;

int uplink_bch_encode(const uint8_t data[UPLINK_BCH_DATA_BYTES],
                      uint8_t codeword[UPLINK_BCH_CODE_BYTES]);
int uplink_bch_decode(const uint8_t codeword[UPLINK_BCH_CODE_BYTES],
                      uint8_t data[UPLINK_BCH_DATA_BYTES]);
uint16_t uplink_crc16(const uint8_t *data, size_t length);

/* Build a complete ground-to-balloon frame. data_length is padded to 7-byte
   BCH fields with 0x20; callers normally pass complete command fields. The
   CRC is the APRS/X.25 value, sent low byte first after the NRZI data. */
int uplink_frame_encode(const uint8_t *data, size_t data_length,
                        uint8_t *out, size_t capacity);

typedef struct {
  uint32_t sync_shift;
  uint8_t preamble_level;
  uint8_t preamble_seen;
  uint8_t preamble_bits;
  uint8_t syncing;
  uint8_t collecting;
  uint8_t expected_blocks;
  uint16_t raw_count;
  uint8_t raw_bits[UPLINK_MAX_RAW_BITS];
  uint8_t ready;
  UplinkCommand command;
} UplinkDecoder;

void uplink_decoder_init(UplinkDecoder *decoder);
/* Start at the NRZI BCH payload after the radio has matched the wire ASM.
   Does not synthesize received preamble bits or bypass BCH/CRC validation. */
void uplink_decoder_begin_payload(UplinkDecoder *decoder);
void uplink_decoder_push_bit(UplinkDecoder *decoder, uint8_t bit);
int uplink_decoder_take(UplinkDecoder *decoder, UplinkCommand *command);

/* Copy one padded BCH field to a NUL-terminated text field. */
size_t uplink_field_text(const UplinkCommand *command, uint8_t block,
                         char *out, size_t capacity);

#endif
