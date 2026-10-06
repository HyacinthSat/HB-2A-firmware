#include "gmsk_downlink.h"
#include "rs8.h"
#include <assert.h>
#include <stdio.h>
#include <string.h>

static uint8_t packed[GMSK_DOWNLINK_MAX_PACKED_BYTES + 1U];
static uint8_t decoded[259];

static unsigned bit(size_t i) { return (packed[i / 8U] >> (7U - i % 8U)) & 1U; }

/* Independent implementation of the six delay elements in the protocol
   diagram, decoding noiseless symbols. Checks both C1 and inverted C2. */
static void decode_symbols(void)
{
  unsigned d[6] = {0};
  memset(decoded, 0, sizeof(decoded));
  for (size_t i = 0; i < sizeof(decoded) * 8U + 6U; ++i) {
    size_t pos = 128U + i * 2U;
    unsigned input = bit(pos) ^ d[0] ^ d[1] ^ d[2] ^ d[5];
    assert(bit(pos + 1U) == (1U ^ input ^ d[1] ^ d[2] ^ d[4] ^ d[5]));
    if (i < sizeof(decoded) * 8U)
      decoded[i / 8U] |= (uint8_t)(input << (7U - i % 8U));
    else assert(input == 0U);
    for (unsigned j = 5U; j > 0U; --j) d[j] = d[j - 1U];
    d[0] = input;
  }
  for (unsigned j = 0; j < 6U; ++j) assert(d[j] == 0U);
  assert(memcmp(decoded, "\x1a\xcf\xfc\x1d", 4) == 0);
}

/* Galois diagram, with x16 at d[0] and x0 at d[16]. */
static void descramble(void)
{
  const char *initial = "11000111000111000";
  unsigned d[17];
  uint8_t prefix[8] = {0};
  const uint8_t expected[] = {0x1c,0x71,0xff,0xff,0x00,0x03,0x80,0x0f};
  for (unsigned i = 0; i < 17; ++i) d[i] = (unsigned)(initial[i] - '0');
  for (size_t i = 0; i < 255U * 8U; ++i) {
    unsigned output = d[16];
    decoded[4U + i / 8U] ^= (uint8_t)(output << (7U - i % 8U));
    if (i < 64U) prefix[i / 8U] |= (uint8_t)(output << (7U - i % 8U));
    for (unsigned j = 16U; j > 0U; --j) d[j] = d[j - 1U];
    d[0] = output;
    d[3] ^= output;
  }
  assert(memcmp(prefix, expected, sizeof(prefix)) == 0);
}

static uint8_t multiply(unsigned a, unsigned b)
{
  unsigned result = 0;
  while (b) {
    if (b & 1U) result ^= a;
    b >>= 1U; a <<= 1U;
    if (a & 256U) a ^= 0x187U;
  }
  return (uint8_t)result;
}

/* Independent RS check at all 32 CCSDS generator roots (no encoder table). */
static void check_rs(void)
{
  for (unsigned root = 0; root < 32; ++root) {
    uint8_t x = 1, syndrome = 0;
    for (unsigned j = 0; j < ((112U + root) * 11U) % 255U; ++j)
      x = multiply(x, 2U);
    for (unsigned j = 4; j < sizeof(decoded); ++j)
      syndrome = multiply(syndrome, x) ^ decoded[j];
    assert(syndrome == 0U);
  }
}

static void verify(const uint8_t *text, size_t length, unsigned counter)
{
  size_t bits = 0;
  const uint8_t csm[] = {0x56,0x08,0x1c,0x97,0x1a,0xa7,0x3d,0x3e};
  packed[sizeof(packed) - 1U] = 0xa5;
  assert(gmsk_downlink_encode(text, length, packed, sizeof(packed)-1U, &bits) == 0);
  assert(bits == 4284U && GMSK_DOWNLINK_MAX_PACKED_BYTES == 536U);
  assert(packed[536] == 0xa5 && (packed[535] & 15U) == 0U);
  for (unsigned i = 0; i < 16; ++i) assert(packed[i] == 0x55);
  assert(memcmp(packed+16, csm, sizeof(csm)) == 0);
  decode_symbols(); descramble(); check_rs();
  assert(decoded[4] == 0 && decoded[5] == 2);
  assert(decoded[6] == (uint8_t)counter && decoded[7] == (uint8_t)counter);
  if (length > 219U) length = 219U;
  if (length) assert(memcmp(decoded+8, text, length) == 0);
  for (size_t i = length; i < 219; ++i) assert(decoded[8+i] == 0x20);
}

int main(void)
{
  uint8_t original[255], long_text[300];
  size_t bits = 123;
  const char *messages[] = {"ACK,EXPR", "ACK,CAPTR", "HSSUP,3F,0F,3F",
                            "HSCON,5,3,8", "HSREJ,PARAM", "ACK,CANCEL"};
  gmsk_downlink_reset_counters();
  assert(gmsk_downlink_encode(NULL, 1, packed, sizeof(packed), &bits) == -1);
  assert(bits == 0);
  assert(gmsk_downlink_encode(NULL, 0, packed, 535, &bits) == -1);
  for (unsigned i = 0; i < 6; ++i) verify((const uint8_t *)messages[i], strlen(messages[i]), i);
  memcpy(original, decoded+4, sizeof(original));
  for (unsigned i = 0; i < 16; ++i) decoded[4+i*15] ^= (uint8_t)(i+1);
  assert(decode_rs_8(decoded+4, NULL, 0, 0) == 16);
  assert(memcmp(original, decoded+4, sizeof(original)) == 0);
  memset(long_text, 'X', sizeof(long_text));
  verify(long_text, sizeof(long_text), 6);
  for (unsigned i = 7; i < 258; ++i) verify(NULL, 0, i);
  puts("GMSK full-frame decode, RS syndromes/correction, PN17, convolution, counters: passed");
  return 0;
}
