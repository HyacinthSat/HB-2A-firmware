#include "gmsk_downlink.h"

#include "rs8.h"

#include <string.h>

#define PN17_INITIAL 0x18E38U /* 11000111000111000, as specified by TM. */

static uint8_t master_count;
static uint8_t virtual_count;
/* Single foreground encoder; avoid nested buffers on the 1 KiB main stack. */
static uint8_t codeword[GMSK_DOWNLINK_RS_CODE_BYTES];

typedef struct {
  uint8_t *data;
  size_t capacity;
  size_t bits;
} BitWriter;

static int put_bit(BitWriter *writer, uint8_t value)
{
  size_t byte;
  uint8_t mask;
  if (writer == NULL || writer->data == NULL ||
      writer->bits >= writer->capacity * 8U) return -1;
  byte = writer->bits >> 3U;
  mask = (uint8_t)(0x80U >> (writer->bits & 7U));
  if (value != 0U) writer->data[byte] |= mask;
  ++writer->bits;
  return 0;
}

static int put_byte(BitWriter *writer, uint8_t value)
{
  for (int bit = 7; bit >= 0; --bit)
    if (put_bit(writer, (uint8_t)((value >> bit) & 1U))) return -1;
  return 0;
}

static uint8_t parity(uint8_t value)
{
  value ^= (uint8_t)(value >> 4U);
  value ^= (uint8_t)(value >> 2U);
  value ^= (uint8_t)(value >> 1U);
  return (uint8_t)(value & 1U);
}

static uint8_t pn17_next(uint32_t *state)
{
  uint8_t output;
  /* CCSDS Figure 10-2: x16..x0 shift toward x0 (output); feedback
     enters x16 and is XORed into x13. This is the Galois arrangement. */
  output = (uint8_t)(*state & 1U);
  *state = (*state >> 1U) ^ (output ? 0x12000U : 0U);
  return output;
}

static void pn17_scramble(uint8_t *codeword)
{
  uint32_t state = PN17_INITIAL;
  for (size_t i = 0U; i < GMSK_DOWNLINK_RS_CODE_BYTES; ++i) {
    uint8_t value = 0U;
    for (int bit = 7; bit >= 0; --bit) {
      uint8_t input = (uint8_t)((codeword[i] >> bit) & 1U);
      value = (uint8_t)((value << 1U) | (input ^ pn17_next(&state)));
    }
    codeword[i] = value;
  }
}

static int put_convolutional_bit(BitWriter *writer, uint8_t input,
                                 uint8_t *state)
{
  /* Newest bit at bit6: G1=171(octal), G2=133(octal), C1 first.
     Reversing register direction without reversing masks changes the code. */
  uint8_t reg = (uint8_t)((*state >> 1U) | ((input & 1U) << 6U));
  if (put_bit(writer, parity((uint8_t)(reg & 0x79U))) ||
      put_bit(writer, (uint8_t)!parity((uint8_t)(reg & 0x5BU)))) return -1;
  *state = reg;
  return 0;
}

static int put_convolutional_byte(BitWriter *writer, uint8_t value,
                                  uint8_t *state)
{
  for (int bit = 7; bit >= 0; --bit)
    if (put_convolutional_bit(writer, (uint8_t)((value >> bit) & 1U), state))
      return -1;
  return 0;
}

void gmsk_downlink_reset_counters(void)
{
  master_count = 0U;
  virtual_count = 0U;
}

int gmsk_downlink_encode(const uint8_t *text, size_t text_length,
                         uint8_t *out, size_t out_capacity,
                         size_t *bit_count)
{
  BitWriter writer;
  uint32_t header;
  uint8_t state = 0U;

  if (bit_count != NULL) *bit_count = 0U;
  if (out == NULL || bit_count == NULL ||
      out_capacity < GMSK_DOWNLINK_MAX_PACKED_BYTES ||
      (text == NULL && text_length != 0U)) return -1;
  memset(out, 0, GMSK_DOWNLINK_MAX_PACKED_BYTES);
  memset(codeword, 0x20, GMSK_DOWNLINK_RS_DATA_BYTES);
  if (text_length > GMSK_DOWNLINK_TEXT_BYTES) text_length = GMSK_DOWNLINK_TEXT_BYTES;
  if (text_length != 0U) memcpy(codeword + 4U, text, text_length);

  /* 2-bit version, 10-bit spacecraft ID, 3-bit VCID, no secondary header,
     followed by one-byte master and virtual-channel frame counters. */
  header = ((uint32_t)1U << 17U) |
           ((uint32_t)master_count << 8U) | virtual_count;
  codeword[0] = (uint8_t)(header >> 24U);
  codeword[1] = (uint8_t)(header >> 16U);
  codeword[2] = (uint8_t)(header >> 8U);
  codeword[3] = (uint8_t)header;
  encode_rs_8(codeword, codeword + GMSK_DOWNLINK_RS_DATA_BYTES, 0);
  pn17_scramble(codeword);

  writer.data = out;
  writer.capacity = out_capacity;
  writer.bits = 0U;
  for (size_t i = 0U; i < GMSK_DOWNLINK_PREAMBLE_BYTES; ++i)
    if (put_byte(&writer, 0x55U)) return -2;

  /* CSM is part of the convolutional input; it is deliberately not PN17
     scrambled. The preamble is the only raw, uncoded portion. */
  if (put_convolutional_byte(&writer, 0x1AU, &state) ||
      put_convolutional_byte(&writer, 0xCFU, &state) ||
      put_convolutional_byte(&writer, 0xFCU, &state) ||
      put_convolutional_byte(&writer, 0x1DU, &state)) return -2;
  for (size_t i = 0U; i < sizeof(codeword); ++i)
    if (put_convolutional_byte(&writer, codeword[i], &state)) return -2;
  /* K=7 terminated convolutional code: six zero input bits. */
  for (unsigned i = 0U; i < 6U; ++i)
    if (put_convolutional_bit(&writer, 0U, &state)) return -2;

  *bit_count = writer.bits;
  ++master_count;
  ++virtual_count;
  return 0;
}
