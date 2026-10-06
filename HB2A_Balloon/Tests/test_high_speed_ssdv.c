#include "gmsk_downlink.h"
#include "high_speed_ssdv.h"

#include <assert.h>
#include <stdio.h>
#include <string.h>

static uint32_t crc32_update(uint32_t crc, const uint8_t *data, size_t length)
{
  while (length-- != 0U) {
    crc ^= *data++;
    for (unsigned bit = 0U; bit < 8U; ++bit)
      crc = (crc & 1U) != 0U ? (crc >> 1U) ^ 0xEDB88320UL : crc >> 1U;
  }
  return crc;
}

int main(void)
{
  uint8_t packet[256] = {0};
  uint8_t frame[223];
  uint8_t frames[8U * 223U];
  uint8_t bits[8192];
  size_t bit_count;
  const unsigned depths[] = {1U, 2U, 3U, 4U, 5U, 8U};

  packet[0] = 0x55U;
  packet[1] = 0x66U;
  packet[6] = 0xA5U;
  packet[7] = 0x12U;
  packet[8] = 0x34U;
  packet[9] = 8U;
  packet[10] = 6U;
  packet[11] = 0x1AU;
  packet[12] = 0x22U;
  packet[13] = 0x01U;
  packet[14] = 0x02U;
  for (unsigned i = 0U; i < 205U; ++i) packet[15U + i] = (uint8_t)i;
  assert(high_speed_ssdv_build_frame(packet, sizeof(packet), "BG7ZDQ",
                                     frame, sizeof(frame)) == 0);
  assert(frame[0] == 0x00U && frame[1] == 0x04U);
  assert(frame[4] == 0xA5U && frame[5] == 0x12U && frame[6] == 0x34U);
  assert(frame[7] == 0x00U && frame[8] == 0x80U && frame[9] == 0x06U);
  assert(frame[10] == packet[11] && frame[11] == 0x22U && frame[12] == 0x01U &&
         frame[13] == 0x02U);
  assert(memcmp(frame + 14U, packet + 15U, 205U) == 0);
  {
    uint32_t crc = crc32_update(0xFFFFFFFFUL, (const uint8_t *)"BG7ZDQ", 6U);
    crc = ~crc32_update(crc, frame + 4U, 215U);
    assert(frame[219] == (uint8_t)(crc >> 24U));
    assert(frame[220] == (uint8_t)(crc >> 16U));
    assert(frame[221] == (uint8_t)(crc >> 8U));
    assert(frame[222] == (uint8_t)crc);
  }
  for (unsigned i = 0U; i < 8U; ++i) memcpy(frames + i * 223U, frame, 223U);
  high_speed_ssdv_reset_counters();
  for (unsigned code = 0U; code <= 3U; ++code) {
    for (unsigned i = 0U; i < sizeof(depths) / sizeof(depths[0]); ++i) {
      unsigned count = depths[i];
      assert(gmsk_high_speed_encode(frames, count, (uint8_t)code,
                                    (uint8_t)depths[i], bits, sizeof(bits),
                                    &bit_count) == 0);
      assert(bit_count > 128U && bit_count <= sizeof(bits) * 8U);
      {
        size_t input_bits = (4U + count * 255U) * 8U;
        size_t expected;
        if (code == 0U) expected = input_bits;
        else if (code == 1U) expected = (input_bits + 6U) * 2U;
        else if (code == 2U) expected = (input_bits + 6U) * 3U / 2U;
        else {
          static const unsigned keep_prefix[] = {0U, 1U, 2U, 2U, 3U, 4U};
          size_t mother = (input_bits + 6U) * 2U;
          expected = mother / 6U * 4U + keep_prefix[mother % 6U];
        }
        assert(bit_count == 128U + expected);
      }
      for (unsigned pre = 0U; pre < 16U; ++pre) assert(bits[pre] == 0x55U);
    }
  }
  puts("high-speed SSDV frame, RS, PN17, interleave and convolution vectors: passed");
  return 0;
}
