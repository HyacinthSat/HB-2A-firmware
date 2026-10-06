#include "gmsk_downlink.h"

#include "rs8.h"

#include <string.h>

#define PN17_INITIAL 0x18E38U /* 11000111000111000, as specified by TM. */

static uint8_t master_count;
static uint8_t virtual_count;
/* Single foreground encoder; avoid nested buffers on the 1 KiB main stack. */
static uint8_t codeword[GMSK_DOWNLINK_RS_CODE_BYTES];
static uint8_t hs_codewords[GMSK_HIGH_SPEED_MAX_CODEWORDS]
                            [GMSK_HIGH_SPEED_CODEWORD_BYTES];
static uint8_t hs_interleaved[GMSK_HIGH_SPEED_MAX_INPUT_BYTES];

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

static void pn17_scramble_bytes(uint8_t *data, size_t length)
{
  uint32_t state = PN17_INITIAL;
  if (data == NULL) return;
  for (size_t i = 0U; i < length; ++i) {
    uint8_t value = 0U;
    for (int bit = 7; bit >= 0; --bit) {
      uint8_t input = (uint8_t)((data[i] >> bit) & 1U);
      value = (uint8_t)((value << 1U) | (input ^ pn17_next(&state)));
    }
    data[i] = value;
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

static int put_high_speed_bit(BitWriter *writer, uint8_t input,
                              uint8_t *state, uint8_t code_id,
                              unsigned *puncture_phase)
{
  uint8_t reg;
  uint8_t c1;
  uint8_t c2;
  uint8_t keep;
  unsigned phase;
  if (writer == NULL || state == NULL || puncture_phase == NULL ||
      code_id > 3U) return -1;
  reg = (uint8_t)((*state >> 1U) | ((input & 1U) << 6U));
  c1 = parity((uint8_t)(reg & 0x79U));
  c2 = parity((uint8_t)(reg & 0x5BU));
  if (code_id == 0U) {
    if (put_bit(writer, input)) return -1;
  } else {
    if (code_id == 1U) c2 = (uint8_t)!c2;
    for (unsigned n = 0U; n < 2U; ++n) {
      phase = *puncture_phase;
      if (code_id == 1U) keep = 1U;
      else if (code_id == 2U) keep = (uint8_t)((phase % 4U) != 2U);
      else keep = (uint8_t)((phase % 6U) == 0U ||
                             (phase % 6U) == 1U ||
                             (phase % 6U) == 3U ||
                             (phase % 6U) == 4U);
      if (keep != 0U && put_bit(writer, n == 0U ? c1 : c2)) return -1;
      *puncture_phase = phase + 1U;
    }
  }
  *state = reg;
  return 0;
}

static int put_high_speed_byte(BitWriter *writer, uint8_t value,
                               uint8_t *state, uint8_t code_id,
                               unsigned *puncture_phase)
{
  for (int bit = 7; bit >= 0; --bit)
    if (put_high_speed_bit(writer, (uint8_t)((value >> bit) & 1U), state,
                           code_id, puncture_phase) != 0) return -1;
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

int gmsk_high_speed_encode(const uint8_t *frames, size_t frame_count,
                           uint8_t code_id, uint8_t intlv_id,
                           uint8_t *out, size_t out_capacity,
                           size_t *bit_count)
{
  BitWriter writer;
  uint8_t state = 0U;
  unsigned puncture_phase = 0U;
  size_t depth;
  size_t coded_bytes;

  if (bit_count != NULL) *bit_count = 0U;
  if (frames == NULL || out == NULL || bit_count == NULL ||
      frame_count == 0U || frame_count > GMSK_HIGH_SPEED_MAX_CODEWORDS ||
      code_id > 3U || out_capacity < GMSK_HIGH_SPEED_MAX_PACKED_BYTES)
    return -1;
  switch (intlv_id) {
    case 1U: depth = 1U; break;
    case 2U: depth = 2U; break;
    case 3U: depth = 3U; break;
    case 4U: depth = 4U; break;
    case 5U: depth = 5U; break;
    case 8U: depth = 8U; break;
    default: return -1;
  }
  if (frame_count > depth) return -1;
  memset(out, 0, out_capacity);
  memset(hs_codewords, 0, sizeof(hs_codewords));
  for (size_t frame = 0U; frame < frame_count; ++frame) {
    memcpy(hs_codewords[frame], frames + frame * GMSK_HIGH_SPEED_DATA_BYTES,
           GMSK_HIGH_SPEED_DATA_BYTES);
    encode_rs_8(hs_codewords[frame],
                hs_codewords[frame] + GMSK_HIGH_SPEED_DATA_BYTES, 0);
    /* The TM PN sequence is restarted for every RS codeword. */
    pn17_scramble_bytes(hs_codewords[frame], GMSK_HIGH_SPEED_CODEWORD_BYTES);
  }
  coded_bytes = frame_count * GMSK_HIGH_SPEED_CODEWORD_BYTES;
  if (frame_count == 1U) {
    memcpy(hs_interleaved, hs_codewords[0], coded_bytes);
  } else {
    size_t offset = 0U;
    for (size_t symbol = 0U; symbol < GMSK_HIGH_SPEED_CODEWORD_BYTES;
         ++symbol) {
      for (size_t frame = 0U; frame < frame_count; ++frame)
        hs_interleaved[offset++] = hs_codewords[frame][symbol];
    }
  }
  writer.data = out;
  writer.capacity = out_capacity;
  writer.bits = 0U;
  for (size_t i = 0U; i < GMSK_DOWNLINK_PREAMBLE_BYTES; ++i)
    if (put_byte(&writer, 0x55U)) return -2;
  if (put_high_speed_byte(&writer, 0x1AU, &state, code_id,
                          &puncture_phase) ||
      put_high_speed_byte(&writer, 0xCFU, &state, code_id,
                          &puncture_phase) ||
      put_high_speed_byte(&writer, 0xFCU, &state, code_id,
                          &puncture_phase) ||
      put_high_speed_byte(&writer, 0x1DU, &state, code_id,
                          &puncture_phase)) return -2;
  for (size_t i = 0U; i < coded_bytes; ++i)
    if (put_high_speed_byte(&writer, hs_interleaved[i], &state, code_id,
                            &puncture_phase)) return -2;
  if (code_id != 0U) {
    for (unsigned i = 0U; i < 6U; ++i)
      if (put_high_speed_bit(&writer, 0U, &state, code_id,
                             &puncture_phase)) return -2;
  }
  *bit_count = writer.bits;
  return 0;
}
