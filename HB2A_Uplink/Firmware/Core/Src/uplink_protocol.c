#include "uplink_protocol.h"

#include <string.h>

#define UPLINK_ASM 0x1ACFFC1DU
#define BCH_POLY 0x4BU /* x^6+x^3+x+1, for g(x)=x^7+x^6+x^3+x+1 */

static uint8_t bch_step(uint8_t state, uint8_t bit)
{
  uint8_t feedback = (uint8_t)((bit ^ (state >> 6U)) & 1U);
  state = (uint8_t)((state << 1U) & 0x7FU);
  if (feedback != 0U) state ^= BCH_POLY;
  return state;
}

static uint8_t bch_parity(const uint8_t data[UPLINK_BCH_DATA_BYTES])
{
  uint8_t state = 0U;
  for (unsigned i = 0U; i < UPLINK_BCH_DATA_BYTES; ++i)
    for (unsigned bit = 0U; bit < 8U; ++bit)
      state = bch_step(state, (uint8_t)((data[i] >> (7U - bit)) & 1U));
  return state;
}

static uint8_t codeword_syndrome(const uint8_t codeword[UPLINK_BCH_CODE_BYTES])
{
  uint8_t state = 0U;
  for (unsigned i = 0U; i < 7U; ++i)
    for (unsigned bit = 0U; bit < 8U; ++bit)
      state = bch_step(state, (uint8_t)((codeword[i] >> (7U - bit)) & 1U));
  for (unsigned bit = 0U; bit < 7U; ++bit)
    state = bch_step(state, (uint8_t)((codeword[7] >> (6U - bit)) & 1U));
  return state;
}

int uplink_bch_encode(const uint8_t data[UPLINK_BCH_DATA_BYTES],
                      uint8_t codeword[UPLINK_BCH_CODE_BYTES])
{
  if (data == NULL || codeword == NULL) return -1;
  memcpy(codeword, data, UPLINK_BCH_DATA_BYTES);
  codeword[7] = (uint8_t)(bch_parity(data) & 0x7FU);
  return 0;
}

int uplink_bch_decode(const uint8_t codeword[UPLINK_BCH_CODE_BYTES],
                      uint8_t data[UPLINK_BCH_DATA_BYTES])
{
  uint8_t corrected[UPLINK_BCH_CODE_BYTES];
  if (codeword == NULL || data == NULL) return -1;
  memcpy(corrected, codeword, sizeof(corrected));
  if ((corrected[7] & 0x80U) != 0U) corrected[7] &= 0x7FU;
  if (codeword_syndrome(corrected) != 0U) {
    uint8_t found = 0U;
    for (unsigned byte = 0U; byte < 8U && found == 0U; ++byte) {
      unsigned limit = byte == 7U ? 7U : 8U;
      for (unsigned bit = 0U; bit < limit; ++bit) {
        uint8_t mask = (uint8_t)(1U << (7U - bit));
        corrected[byte] ^= mask;
        if (codeword_syndrome(corrected) == 0U) {
          found = 1U;
          break;
        }
        corrected[byte] ^= mask;
      }
    }
    if (found == 0U) return -2;
  }
  memcpy(data, corrected, UPLINK_BCH_DATA_BYTES);
  return 0;
}

uint16_t uplink_crc16(const uint8_t *data, size_t length)
{
  uint16_t crc = 0xFFFFU;
  if (data == NULL && length != 0U) return 0U;
  for (size_t i = 0U; i < length; ++i) {
    crc ^= data[i];
    for (unsigned bit = 0U; bit < 8U; ++bit)
      crc = (crc & 1U) != 0U ? (uint16_t)((crc >> 1U) ^ 0x8408U)
                             : (uint16_t)(crc >> 1U);
  }
  return (uint16_t)~crc;
}

static unsigned command_blocks(const uint8_t *data)
{
  if (memcmp(data, "TRANPD", 6U) == 0) return 5U;
  if (memcmp(data, "EXPR", 4U) == 0) return 1U;
  if (memcmp(data, "CAPTR", 5U) == 0) return 5U;
  if (memcmp(data, "REQF", 4U) == 0) return 2U;
  if (memcmp(data, "HSPAR", 5U) == 0) return 4U;
  if (memcmp(data, "CHPRAM", 6U) == 0) return 4U;
  if (memcmp(data, "CFRM", 4U) == 0) return 1U;
  if (memcmp(data, "CANCEL", 6U) == 0) return 1U;
  return 0U;
}

static void put_bit(uint8_t *out, size_t *index, uint8_t bit)
{
  out[*index / 8U] = (uint8_t)((out[*index / 8U] << 1U) | (bit & 1U));
  ++*index;
}

int uplink_frame_encode(const uint8_t *data, size_t data_length,
                        uint8_t *out, size_t capacity)
{
  size_t blocks;
  size_t bytes;
  size_t pos = 0U;
  uint8_t level = 1U;
  uint8_t padded[UPLINK_MAX_DATA_BYTES];
  if (data == NULL || out == NULL || data_length == 0U ||
      data_length > UPLINK_MAX_DATA_BYTES) return -1;
  blocks = (data_length + UPLINK_BCH_DATA_BYTES - 1U) /
           UPLINK_BCH_DATA_BYTES;
  bytes = 16U + 4U + blocks * 8U + 2U;
  if (blocks > UPLINK_MAX_BLOCKS || capacity < bytes) return -2;
  memset(padded, 0x20, sizeof(padded));
  memcpy(padded, data, data_length);
  memset(out, 0, bytes);
  for (size_t i = 0U; i < 16U; ++i) out[i] = 0x55U;
  out[16] = 0x1AU; out[17] = 0xCFU; out[18] = 0xFCU; out[19] = 0x1DU;
  pos = 160U;
  for (size_t block = 0U; block < blocks; ++block) {
    uint8_t plain[UPLINK_BCH_DATA_BYTES];
    uint8_t codeword[UPLINK_BCH_CODE_BYTES];
    for (size_t i = 0U; i < sizeof(plain); ++i) {
      size_t index = block * sizeof(plain) + i;
      plain[i] = index < data_length ? data[index] : 0x20U;
    }
    (void)uplink_bch_encode(plain, codeword);
    for (size_t i = 0U; i < sizeof(codeword); ++i) {
      for (unsigned bit = 0U; bit < 8U; ++bit) {
        uint8_t decoded = (uint8_t)((codeword[i] >> (7U - bit)) & 1U);
        uint8_t raw = decoded != 0U ? level : (uint8_t)!level;
        level = raw;
        put_bit(out, &pos, raw);
      }
    }
  }
  {
    uint16_t crc = uplink_crc16(padded, blocks * UPLINK_BCH_DATA_BYTES);
    for (unsigned bit = 0U; bit < 16U; ++bit) {
      uint8_t decoded = (uint8_t)((((bit < 8U) ? crc : (uint16_t)(crc >> 8U)) >>
                                   (7U - (bit & 7U))) & 1U);
      uint8_t raw = decoded != 0U ? level : (uint8_t)!level;
      level = raw;
      put_bit(out, &pos, raw);
    }
  }
  return (int)bytes;
}

static uint8_t raw_byte(const uint8_t *bits, size_t start, uint8_t initial,
                        size_t byte_index)
{
  uint8_t previous = initial;
  uint8_t value = 0U;
  for (size_t i = 0U; i < byte_index * 8U; ++i) previous = bits[start + i];
  for (unsigned bit = 0U; bit < 8U; ++bit) {
    uint8_t current = bits[start + byte_index * 8U + bit];
    value = (uint8_t)((value << 1U) | (current == previous ? 1U : 0U));
    previous = current;
  }
  return value;
}

static int decode_raw(const UplinkDecoder *decoder, uint8_t initial,
                      UplinkCommand *command)
{
  uint8_t decoded[UPLINK_MAX_DATA_BYTES];
  size_t code_bytes;
  uint16_t received_crc;
  if (decoder == NULL || command == NULL || decoder->expected_blocks == 0U) return -1;
  code_bytes = (size_t)decoder->expected_blocks * UPLINK_BCH_CODE_BYTES;
  for (size_t block = 0U; block < decoder->expected_blocks; ++block) {
    uint8_t codeword[UPLINK_BCH_CODE_BYTES];
    for (size_t i = 0U; i < sizeof(codeword); ++i)
      codeword[i] = raw_byte(decoder->raw_bits, 0U, initial,
                             block * sizeof(codeword) + i);
    if (uplink_bch_decode(codeword, decoded + block * UPLINK_BCH_DATA_BYTES)) return -2;
  }
  received_crc = (uint16_t)raw_byte(decoder->raw_bits, 0U, initial, code_bytes) |
                 (uint16_t)((uint16_t)raw_byte(decoder->raw_bits, 0U, initial,
                                                code_bytes + 1U) << 8U);
  command->block_count = decoder->expected_blocks;
  memcpy(command->data, decoded,
         (size_t)decoder->expected_blocks * UPLINK_BCH_DATA_BYTES);
  command->crc = received_crc;
  if (uplink_crc16(command->data,
                   (size_t)command->block_count * UPLINK_BCH_DATA_BYTES) != received_crc)
    return -3;
  return 0;
}

void uplink_decoder_init(UplinkDecoder *decoder)
{
  if (decoder != NULL) memset(decoder, 0, sizeof(*decoder));
}

void uplink_decoder_push_bit(UplinkDecoder *decoder, uint8_t bit)
{
  uint8_t first[UPLINK_BCH_DATA_BYTES];
  uint8_t codeword[UPLINK_BCH_CODE_BYTES];
  if (decoder == NULL || decoder->ready != 0U) return;
  bit &= 1U;
  if (decoder->collecting == 0U) {
    if (decoder->preamble_seen == 0U) {
      if (decoder->preamble_bits == 0U) {
        decoder->preamble_level = bit;
        decoder->preamble_bits = 1U;
      } else if (bit == (uint8_t)(decoder->preamble_level ^
                                  (decoder->preamble_bits & 1U))) {
        ++decoder->preamble_bits;
        if (decoder->preamble_bits >= 128U) decoder->preamble_seen = 1U;
      } else {
        decoder->preamble_level = bit;
        decoder->preamble_bits = 1U;
      }
    }
    decoder->sync_shift = (decoder->sync_shift << 1U) | bit;
    if (decoder->preamble_seen != 0U && decoder->sync_shift == UPLINK_ASM) {
      decoder->collecting = 1U;
      decoder->raw_count = 0U;
      decoder->expected_blocks = 0U;
    }
    return;
  }
  if (decoder->raw_count >= UPLINK_MAX_RAW_BITS) {
    uplink_decoder_init(decoder);
    return;
  }
  decoder->raw_bits[decoder->raw_count++] = bit;
  if (decoder->expected_blocks == 0U && decoder->raw_count >= 64U) {
    for (uint8_t initial = 0U; initial < 2U && decoder->expected_blocks == 0U; ++initial) {
      uint8_t previous = initial;
      memset(codeword, 0, sizeof(codeword));
      for (unsigned i = 0U; i < 64U; ++i) {
        uint8_t current = decoder->raw_bits[i];
        uint8_t value = (uint8_t)(current == previous ? 1U : 0U);
        codeword[i / 8U] = (uint8_t)((codeword[i / 8U] << 1U) | value);
        previous = current;
      }
      if (uplink_bch_decode(codeword, first) == 0) {
        unsigned blocks = command_blocks(first);
        if (blocks != 0U) decoder->expected_blocks = (uint8_t)blocks;
      }
      memset(codeword, 0, sizeof(codeword));
    }
  }
  if (decoder->expected_blocks != 0U &&
      decoder->raw_count >= (uint16_t)((decoder->expected_blocks * 8U + 2U) * 8U)) {
    /* Mark complete; BCH correction and CRC are deliberately deferred to the
       foreground poll so the 4.8 kHz DIO clock ISR remains bounded. */
    decoder->ready = 2U;
  }
}

int uplink_decoder_take(UplinkDecoder *decoder, UplinkCommand *command)
{
  if (decoder == NULL || command == NULL || decoder->ready == 0U) return 0;
  if (decoder->ready == 2U) {
    UplinkCommand decoded;
    memset(&decoded, 0, sizeof(decoded));
    if (decode_raw(decoder, 0U, &decoded) != 0 &&
        decode_raw(decoder, 1U, &decoded) != 0) {
      uplink_decoder_init(decoder);
      return 0;
    }
    decoder->command = decoded;
    decoder->ready = 1U;
  }
  *command = decoder->command;
  uplink_decoder_init(decoder);
  return 1;
}

size_t uplink_field_text(const UplinkCommand *command, uint8_t block,
                         char *out, size_t capacity)
{
  size_t length = UPLINK_BCH_DATA_BYTES;
  if (command == NULL || out == NULL || capacity == 0U ||
      block >= command->block_count) return 0U;
  while (length > 0U && command->data[(size_t)block * 7U + length - 1U] == 0x20U) --length;
  if (length >= capacity) length = capacity - 1U;
  memcpy(out, command->data + (size_t)block * 7U, length);
  out[length] = 0;
  return length;
}
