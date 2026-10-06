#include "ground_station.h"
#include "uplink_radio.h"
#include "usbd_cdc_if.h"
#include <assert.h>
#include <stdio.h>
#include <string.h>

static unsigned sent;
static int busy;
static char reply[320];
void UplinkRadio_TxReport(char *out, size_t capacity)
{ (void)snprintf(out,capacity,"TXMS=77 EXPECT=77 SNAP=1"); }
int UplinkRadio_Init(void) { return 0; }
int UplinkRadio_Ready(void) { return 1; }
int UplinkRadio_Send(const uint8_t *frame, size_t size, uint8_t power)
{
  assert(size == 30 && power == 10 && frame[16] == 0x1A);
  ++sent;
  return 0;
}
uint8_t CDC_Transmit_FS(uint8_t *data, uint16_t n)
{
  if (busy) return USBD_BUSY;
  assert(n < sizeof(reply));
  memcpy(reply, data, n);
  reply[n] = 0;
  return USBD_OK;
}
static void receive(const char *s)
{
  GroundStation_Receive((const uint8_t *)s, (uint32_t)strlen(s));
  for (unsigned i = 0; i < 8; ++i) GroundStation_Poll();
}
int main(void)
{
  GroundStation_Init();
  GroundStation_ResetInput();
  receive("STATUS\n"); /* A request arriving before the first poll must survive. */
  assert(strstr(reply, "OK HB2A-UPLINK/1"));
  receive("SEND 10 4558");
  assert(sent == 0);
  receive("5052202020\r\n");
  assert(sent == 1 && strstr(reply, "OK SENT"));
  assert(strstr(reply,"TXMS=77 EXPECT=77"));
  receive("SEND 10 45585052202020");
  receive("\x18\n"); /* Abort a retained partial request during reconnect. */
  assert(sent == 1);
  busy = 1;
  receive("STATUS\nSEND 10 45585052202020\n");
  assert(sent == 1); /* Response backpressure, then exactly one pending command. */
  busy = 0;
  receive("");
  assert(sent == 2);
  char large[301];
  memset(large, 'X', 300); large[300] = 0;
  receive(large);
  receive("SEND 10 45585052202020\n");
  assert(sent == 2 && strstr(reply, "ERR LINE"));
  receive("SEND 10 45585052202020\n");
  assert(sent == 3);
  receive("SEND 10 45585052202020");
  GroundStation_ResetInput();
  receive("\n");
  assert(sent == 3);
  puts("USB fragmentation, abort, overflow, reconnect and backpressure tests passed");
}
