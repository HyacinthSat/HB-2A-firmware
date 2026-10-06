#include "ax25_afsk.h"

#include <assert.h>
#include <stdio.h>
#include <string.h>

int main(void)
{
  uint8_t frame[AX25_MAX_FRAME];
  AFSK_Stream stream;
  int length = AX25_BuildPath("CALL1", "APZDQ7", "APZDQ7",
                              (const uint8_t *)"X", 1U,
                              frame, sizeof(frame));
  assert(length == 26);
  assert(frame[6] == 0xE0U);
  assert(frame[13] == 0x60U);
  assert(frame[14] == ((uint8_t)'A' << 1));
  assert(frame[15] == ((uint8_t)'P' << 1));
  assert(frame[16] == ((uint8_t)'Z' << 1));
  assert(frame[17] == ((uint8_t)'D' << 1));
  assert(frame[18] == ((uint8_t)'Q' << 1));
  assert(frame[19] == ((uint8_t)'7' << 1));
  /* The path text has no '*'; H=1 in the SSID byte makes the decoder show it. */
  assert(frame[20] == 0xE1U);
  assert(AFSK_Prepare(&stream, frame, (size_t)length) == 0);
  puts("AX.25 path tests passed");
  return 0;
}
