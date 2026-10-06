#include "main.h"
#include "uplink_radio.h"
#include "uplink_protocol.h"
#include <assert.h>
#include <stdio.h>
#include <string.h>

SPI_HandleTypeDef hspi1;
static uint8_t regs[128], fifo[66];
static unsigned fifo_size, resets;
static uint32_t ticks;
static int fail_spi, suppress_done, fail_stop, transmitting, led;
static int early_done;
static uint32_t tx_started, airtime;

uint32_t HAL_GetTick(void) { return ticks; }
void HAL_Delay(uint32_t ms) { ticks += ms; }
void HAL_GPIO_WritePin(int port, int pin, int value)
{
  if (port == RA02_RST_GPIO_Port && pin == RA02_RST_Pin && !value) {
    ++resets;
    memset(regs, 0, sizeof(regs));
    regs[0x42] = 0x12;
    transmitting = 0;
  }
  if (port == LED_TX_GPIO_Port && pin == LED_TX_Pin) led = value;
}
HAL_StatusTypeDef HAL_SPI_Transmit(SPI_HandleTypeDef *spi, uint8_t *tx, uint16_t n, uint32_t timeout)
{
  (void)spi; (void)timeout;
  if (fail_spi) return HAL_ERROR;
  unsigned addr = tx[0] & 0x7F;
  if (!addr) {
    fifo_size = n - 1;
    memcpy(fifo, tx + 1, fifo_size);
  } else {
    assert(n == 2);
    if (addr == 1 && tx[1] == 9 && transmitting && fail_stop) return HAL_ERROR;
    regs[addr] = tx[1];
    if (addr == 0x3F && tx[1] == 0x10) regs[0x3F] = 0;
    if (addr == 1) {
      transmitting = tx[1] == 0x0B;
      if (transmitting) {
        tx_started = ticks;
        unsigned preamble = (regs[0x25]<<8) | regs[0x26];
        unsigned sync = (regs[0x27]&0x10) ? (regs[0x27]&7)+1 : 0;
        unsigned divisor = (regs[2]<<8) | regs[3];
        airtime = (uint32_t)(((preamble+sync+fifo_size)*8ULL*divisor+31999)/32000);
      } else regs[0x3f] &= (uint8_t)~8U;
    }
  }
  return HAL_OK;
}
HAL_StatusTypeDef HAL_SPI_TransmitReceive(SPI_HandleTypeDef *spi, uint8_t *tx, uint8_t *rx,
                                         uint16_t n, uint32_t timeout)
{
  (void)spi; (void)timeout;
  assert(n == 2);
  if (fail_spi) return HAL_ERROR;
  rx[0] = 0;
  if (transmitting && !suppress_done && (early_done || ticks-tx_started>=airtime))
    regs[0x3f] |= 8U;
  rx[1] = regs[tx[0]];
  return HAL_OK;
}

int main(void)
{
  uint8_t frame[86];
  int n = uplink_frame_encode((const uint8_t *)"EXPR   ", 7, frame, sizeof(frame));
  assert(UplinkRadio_Init() == 0);
  assert(regs[2] == 0x1A && regs[3] == 0x0B && regs[5] == 20);
  assert(regs[0x0A] == 0x49 && regs[0x27] == 0x33);
  assert(regs[0x26] == 32 && regs[0x31] == 0x40 && regs[0x30] == 0);
  assert(regs[6] == 0x6C && regs[7] == 0xD9 && regs[8] == 0x9A);
  assert(memcmp(regs + 0x28, frame + 16, 4) == 0);
  assert(UplinkRadio_Send(frame, n, 17) == 0);
  assert(fifo_size == (unsigned)n - 20 && memcmp(fifo, frame + 20, fifo_size) == 0);
  assert(regs[0x32] == fifo_size && regs[0x35] == 0x80);
  assert(regs[9] == 0x8F && !transmitting && !led && regs[1] == 9);
  char report[240];
  UplinkRadio_TxReport(report,sizeof(report));
  assert(strstr(report,"TXMS=77 EXPECT=77 SNAP=1 MODE=0B"));
  n=uplink_frame_encode((const uint8_t *)"TRANPD BG7ZDQ APZDQ7 APZDQ7 HELLO  ",35,frame,sizeof(frame));
  for (unsigned repeat=0; repeat<10; ++repeat) {
    assert(UplinkRadio_Send(frame,n,17)==0);
    UplinkRadio_TxReport(report,sizeof(report));
    assert(strstr(report,"TXMS=131 EXPECT=131"));
    assert(strstr(report,"FRF=6CD99A RATE=1A0B DEV=0014 SYNC=33 PKT=40"));
    /* Reconstruct the air bytes from the programmed packet engine and FIFO. */
    UplinkDecoder decoder;
    UplinkCommand command;
    uplink_decoder_init(&decoder);
    for (unsigned i=0; i<32+4+fifo_size; ++i) {
      uint8_t byte=i<32 ? ((regs[0x27]&0x20) ? 0x55 : 0xaa) :
                   i<36 ? regs[0x28+i-32] : fifo[i-36];
      for (unsigned bit=0; bit<8; ++bit)
        uplink_decoder_push_bit(&decoder,(byte>>(7-bit))&1);
    }
    assert(uplink_decoder_take(&decoder,&command)==1);
    assert(memcmp(command.data,"TRANPD BG7ZDQ APZDQ7 APZDQ7 HELLO  ",35)==0);
  }
  early_done=1;
  assert(UplinkRadio_Send(frame,n,17)==-7 && !UplinkRadio_Ready());
  early_done=0;
  suppress_done = 1;
  unsigned before = resets;
  uint32_t start = ticks;
  assert(UplinkRadio_Send(frame, n, 2) < 0);
  assert(ticks - start >= 500 && resets > before && !transmitting && !led);
  assert(!UplinkRadio_Ready());
  suppress_done = 0;
  assert(UplinkRadio_Send(frame, n, 10) == 0); /* Reinitializes after failure. */
  fail_stop = 1;
  before = resets;
  assert(UplinkRadio_Send(frame, n, 10) < 0);
  assert(resets > before && !transmitting && !led);
  fail_stop = 0;
  fail_spi = 1;
  assert(UplinkRadio_Init() < 0 && !UplinkRadio_Ready());
  fail_spi = 0;
  /* All 66 FIFO bytes are supported; threshold must not use reserved bit 6. */
  memset(frame,0x55,16); memcpy(frame+16,"\x1a\xcf\xfc\x1d",4);
  assert(UplinkRadio_Send(frame,sizeof(frame),10)==0 && fifo_size==66);
  assert(UplinkRadio_Send(frame, n, 18) < 0);
  puts("radio register, FIFO, timeout and reset tests passed");
}
