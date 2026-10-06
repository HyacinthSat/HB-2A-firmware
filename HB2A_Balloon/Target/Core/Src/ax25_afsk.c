#include "ax25_afsk.h"
#include <string.h>

uint16_t AX25_Crc(const uint8_t *data, size_t length)
{
  uint16_t crc = 0xFFFFU;
  for (size_t i = 0U; i < length; ++i) {
    crc ^= data[i];
    for (unsigned b = 0U; b < 8U; ++b)
      crc = (crc & 1U) ? (uint16_t)((crc >> 1) ^ 0x8408U) : (uint16_t)(crc >> 1);
  }
  return (uint16_t)~crc; /* CRC-16/X.25; check("123456789") = 0x906E */
}

static int address(const char *call, uint8_t *out, uint8_t control)
{
  unsigned n = 0U, ssid = 0U;
  if (call == NULL) return -1;
  while (call[n] != 0 && call[n] != '-') {
    char c = call[n];
    if (n == 6U || !((c >= 'A' && c <= 'Z') || (c >= '0' && c <= '9'))) return -1;
    out[n++] = (uint8_t)((unsigned)c << 1);
  }
  const char *suffix = call + n;
  for (; n < 6U; ++n) out[n] = (uint8_t)(' ' << 1);
  if (*suffix == '-') {
    ++suffix;
    unsigned digits = 0U;
    while (*suffix != 0) {
      if (*suffix < '0' || *suffix > '9' || ++digits > 2U) return -1;
      ssid = ssid * 10U + (unsigned)(*suffix++ - '0');
    }
    if (digits == 0U || ssid > 15U) return -1;
  }
  out[6] = (uint8_t)(0x60U | (ssid << 1) | control);
  return 0;
}

static int path_address(const char *path, uint8_t *out)
{
  size_t n;
  if (path == NULL || out == NULL) return -1;
  n = strlen(path);
  if (n == 0U || n > 6U) return -1;
  memset(out, (int)(' ' << 1), 6U);
  for (size_t i = 0U; i < n; ++i) out[i] = (uint8_t)((unsigned char)path[i] << 1);
  out[6] = 0xE1U; /* SSID 0, reserved bits set, H bit set -> decoder '*'. */
  return 0;
}

int AX25_Build(const char *source, const char *destination,
               const uint8_t *info, size_t length, uint8_t *out, size_t capacity)
{
  if (info == NULL || out == NULL || length > AX25_MAX_INFO || capacity < length + 18U) return -1;
  /* APZDQ7 is the destination, not a digipeater. UI command: dest C=1, src C=0. */
  if (address(destination, out, 0x80U) || address(source, out + 7, 0x01U)) return -2;
  out[14] = 0x03U; /* UI */
  out[15] = 0xF0U; /* no layer 3 */
  memcpy(out + 16, info, length);
  uint16_t crc = AX25_Crc(out, length + 16U);
  out[length + 16U] = (uint8_t)crc;
  out[length + 17U] = (uint8_t)(crc >> 8);
  return (int)(length + 18U);
}

int AX25_BuildPath(const char *source, const char *destination,
                   const char *path, const uint8_t *info, size_t length,
                   uint8_t *out, size_t capacity)
{
  if (info == NULL || out == NULL || path == NULL ||
      length > AX25_MAX_INFO || capacity < length + 25U) return -1;
  if (address(destination, out, 0x80U) ||
      address(source, out + 7U, 0x00U) ||
      path_address(path, out + 14U)) return -2;
  out[21] = 0x03U;
  out[22] = 0xF0U;
  memcpy(out + 23U, info, length);
  {
    uint16_t crc = AX25_Crc(out, length + 23U);
    out[length + 23U] = (uint8_t)crc;
    out[length + 24U] = (uint8_t)(crc >> 8);
  }
  /* Destination and source are terminated by the following path address;
     mark only the last address in the address field with the extension bit. */
  out[6] &= (uint8_t)~0x01U;
  out[13] &= (uint8_t)~0x01U;
  return (int)(length + 25U);
}

static int bit(AFSK_Stream *s, unsigned data, uint8_t *tone)
{
  if (s->count == AFSK_MAX_BITS) return -1;
  if (data == 0U) *tone ^= 1U; /* NRZI: zero changes tone, one retains it */
  s->tones[s->count++] = *tone;
  return 0;
}

static int flags(AFSK_Stream *s, unsigned count, uint8_t *tone)
{
  while (count-- != 0U)
    for (unsigned b = 0U; b < 8U; ++b)
      if (bit(s, (0x7EU >> b) & 1U, tone)) return -1;
  return 0;
}

int AFSK_Prepare(AFSK_Stream *s, const uint8_t *frame, size_t length)
{
  uint8_t tone = 0U;
  unsigned ones = 0U;
  if (s == NULL) return -1;
  s->count = s->index = 0U;
  s->sample = s->phase = 0U;
  if (frame == NULL || length < 18U || length > AX25_MAX_FRAME) return -1;
  if (flags(s, AFSK_PREAMBLE_FLAGS, &tone)) goto fail;
  for (size_t i = 0U; i < length; ++i) {
    for (unsigned b = 0U; b < 8U; ++b) {
      unsigned v = (frame[i] >> b) & 1U;
      if (bit(s, v, &tone)) goto fail;
      ones = v ? ones + 1U : 0U;
      if (ones == 5U) {
        if (bit(s, 0U, &tone)) goto fail;
        ones = 0U;
      }
    }
  }
  if (flags(s, AFSK_TAIL_FLAGS, &tone)) goto fail;
  return 0;
fail:
  s->count = 0U;
  return -1;
}

int AFSK_NextSample(AFSK_Stream *s)
{
  if (s->index >= s->count) return -1;
  /* 26400 Hz / 132 * {6,11} = {1200,2200} Hz. Preserve phase across bits. */
  int level = s->phase < 66U;
  unsigned next = (unsigned)s->phase + (s->tones[s->index] ? 11U : 6U);
  s->phase = (uint8_t)(next >= 132U ? next - 132U : next);
  if (++s->sample == AFSK_SAMPLES_PER_BIT) {
    s->sample = 0U;
    ++s->index;
  }
  return level;
}

uint32_t AFSK_DurationMs(const AFSK_Stream *s)
{
  return ((uint32_t)s->count * 1000U + 1199U) / 1200U;
}
