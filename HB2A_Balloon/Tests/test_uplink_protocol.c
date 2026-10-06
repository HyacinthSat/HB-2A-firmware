#include "uplink_protocol.h"

#include <assert.h>
#include <stdio.h>
#include <string.h>

static void push_frame(UplinkDecoder *decoder, const uint8_t *frame, size_t length)
{
  for (size_t i = 0U; i < length; ++i)
    for (unsigned bit = 0U; bit < 8U; ++bit)
      uplink_decoder_push_bit(decoder, (uint8_t)((frame[i] >> (7U - bit)) & 1U));
}

int main(void)
{
  uint8_t data[35] = {0};
  uint8_t frame[128] = {0};
  uint8_t codeword[UPLINK_BCH_CODE_BYTES];
  uint8_t decoded[UPLINK_BCH_DATA_BYTES];
  UplinkDecoder decoder;
  UplinkCommand command;
  char field[8];
  int length;

  memcpy(data, "TRANPD", 6U);
  memcpy(data + 7U, "CALL1", 5U);
  memcpy(data + 14U, "APZDQ7", 6U);
  memcpy(data + 21U, "APZDQ7", 6U);
  memcpy(data + 28U, "HELLO", 5U);
  for (size_t i = 0U; i < sizeof(data); ++i)
    if (data[i] == 0U) data[i] = 0x20U;

  assert(uplink_bch_encode(data, codeword) == 0);
  codeword[2] ^= 0x04U;
  assert(uplink_bch_decode(codeword, decoded) == 0);
  assert(memcmp(decoded, data, sizeof(decoded)) == 0);
  for (unsigned bit = 0U; bit < 7U; ++bit) {
    assert(uplink_bch_encode(data, codeword) == 0);
    codeword[7] ^= (uint8_t)(1U << bit);
    assert(uplink_bch_decode(codeword, decoded) == 0);
    assert(memcmp(decoded, data, sizeof(decoded)) == 0);
  }

  length = uplink_frame_encode(data, sizeof(data), frame, sizeof(frame));
  assert(length > 0);
  uplink_decoder_init(&decoder);
  push_frame(&decoder, frame, (size_t)length);
  assert(uplink_decoder_take(&decoder, &command) == 1);
  assert(command.block_count == 5U);
  assert(memcmp(command.data, data, sizeof(data)) == 0);
  assert(uplink_field_text(&command, 3U, field, sizeof(field)) == 6U);
  assert(strcmp(field, "APZDQ7") == 0);
  assert(uplink_decoder_take(&decoder, &command) == 0);

  {
    uint8_t short_data[4] = {'E','X','P','R'};
    length = uplink_frame_encode(short_data, sizeof(short_data), frame, sizeof(frame));
    assert(length > 0);
    uplink_decoder_init(&decoder);
    push_frame(&decoder, frame, (size_t)length);
    assert(uplink_decoder_take(&decoder, &command) == 1);
    assert(command.block_count == 1U);
    assert(memcmp(command.data, "EXPR   ", 7U) == 0);
  }

  frame[0] ^= 0x01U;
  uplink_decoder_init(&decoder);
  push_frame(&decoder, frame, (size_t)length);
  assert(uplink_decoder_take(&decoder, &command) == 0);
  frame[0] ^= 0x01U;
  frame[length - 1U] ^= 0x01U;
  uplink_decoder_init(&decoder);
  push_frame(&decoder, frame, (size_t)length);
  assert(uplink_decoder_take(&decoder, &command) == 0);
  puts("uplink protocol tests passed");
  return 0;
}
