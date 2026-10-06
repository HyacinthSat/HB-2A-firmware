#include "ground_command.h"
#include "uplink_protocol.h"
#include <string.h>

static int hex_digit(char c)
{
  if (c >= '0' && c <= '9') return c - '0';
  if (c >= 'a' && c <= 'f') return c - 'a' + 10;
  if (c >= 'A' && c <= 'F') return c - 'A' + 10;
  return -1;
}

/* Validate one space-padded seven-byte AX.25 callsign field.  The base
   callsign is limited to six characters; an optional numeric SSID is kept
   within the seven-byte uplink field and is limited to 0..15. */
static int valid_callsign_field(const uint8_t *field)
{
  size_t i = 0U;
  unsigned base_length = 0U;
  unsigned ssid = 0U;
  unsigned ssid_digits = 0U;
  if (field == NULL) return 0;
  while (i < 7U && field[i] != (uint8_t)' ' && field[i] != (uint8_t)'-') {
    uint8_t c = field[i];
    if (!((c >= (uint8_t)'A' && c <= (uint8_t)'Z') ||
          (c >= (uint8_t)'0' && c <= (uint8_t)'9')) || ++base_length > 6U)
      return 0;
    ++i;
  }
  if (base_length == 0U) return 0;
  if (i < 7U && field[i] == (uint8_t)'-') {
    ++i;
    while (i < 7U && field[i] != (uint8_t)' ') {
      if (field[i] < (uint8_t)'0' || field[i] > (uint8_t)'9' ||
          ++ssid_digits > 2U)
        return 0;
      ssid = ssid * 10U + (unsigned)(field[i] - (uint8_t)'0');
      ++i;
    }
    if (ssid_digits == 0U || ssid > 15U) return 0;
  }
  while (i < 7U) {
    if (field[i++] != (uint8_t)' ') return 0;
  }
  return 1;
}

int GroundCommand_Encode(const char *line, uint8_t *power,
                         uint8_t *frame, size_t capacity)
{
  uint8_t data[35];
  size_t n, expected;
  unsigned p = 0U, digits = 0U;
  if (!line || !power || !frame || strncmp(line, "SEND ", 5U)) return -1;
  line += 5;
  while (*line >= '0' && *line <= '9') {
    if (++digits > 2U) return -1;
    p = p * 10U + (unsigned)(*line++ - '0');
  }
  if (*line++ != ' ' || p < 2U || p > 17U) return -1;
  n = strlen(line);
  if (n == 0U || n > sizeof(data) * 2U || (n % 14U) != 0U) return -2;
  n /= 2U;
  for (size_t i = 0U; i < n; ++i) {
    int a = hex_digit(line[i * 2U]), b = hex_digit(line[i * 2U + 1U]);
    if (a < 0 || b < 0) return -2;
    data[i] = (uint8_t)((a << 4) | b);
    if (data[i] < 0x20U || data[i] > 0x7EU) return -2;
  }
  if (memcmp(data, "TRANPD ", 7U) == 0) expected = 35U;
  else if (memcmp(data, "EXPR   ", 7U) == 0) expected = 7U;
  else if (memcmp(data, "CAPTR  ", 7U) == 0) expected = 35U;
  else if (memcmp(data, "REQF   ", 7U) == 0) expected = 14U;
  else if (memcmp(data, "HSPAR  ", 7U) == 0) expected = 28U;
  else if (memcmp(data, "CHPRAM ", 7U) == 0) expected = 28U;
  else if (memcmp(data, "CFRM   ", 7U) == 0) expected = 7U;
  else if (memcmp(data, "CANCEL ", 7U) == 0) expected = 7U;
  else return -3;
  if (n != expected) return -3;
  if (memcmp(data, "TRANPD ", 7U) == 0 &&
      (!valid_callsign_field(data + 7U) ||
       !valid_callsign_field(data + 14U) ||
       memcmp(data + 21U, "APZDQ7 ", 7U) != 0)) return -4;
  *power = (uint8_t)p;
  return uplink_frame_encode(data, n, frame, capacity);
}
