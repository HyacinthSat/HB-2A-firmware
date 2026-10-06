#include "ax25_afsk.h"

#include <assert.h>
#include <stdio.h>
#include <string.h>

int main(void)
{
  uint8_t frame[AX25_MAX_FRAME];
  AFSK_Stream stream;
  int length = AX25_BuildPath("CALL1", "APZDQ7", "HB-2A",
                              (const uint8_t *)"X", 1U,
                              frame, sizeof(frame));
  assert(length == 26);
  assert(frame[6] == 0xE0U);
  assert(frame[13] == 0x60U);
  assert(frame[14] == ((uint8_t)'H' << 1));
  assert(frame[15] == ((uint8_t)'B' << 1));
  assert(frame[16] == ((uint8_t)'-' << 1));
  assert(frame[20] == 0xE1U);
  assert(AFSK_Prepare(&stream, frame, (size_t)length) == 0);
  puts("AX.25 path tests passed");
  return 0;
}
