#include "main.h"
#include "spi.h"
#include "sx1278_aprs.h"
#include "gmsk_downlink.h"
#include <assert.h>
#include <stdio.h>
#include <string.h>

GPIO_TypeDef mock_a, mock_b, mock_c;
TIM_TypeDef mock_tim;
RCC_TypeDef mock_rcc;
EXTI_TypeDef mock_exti;
SPI_HandleTypeDef hspi2;
static uint8_t regs[128], tx_regs[128];
static uint32_t tick, dio1_mode, dio2_mode;
static unsigned irq_enabled[32], symbol_count, service_count, mode_error;
static int failure_reg = -1;
static unsigned stuck_rx_mode, missing_rx_ready;
static unsigned block_swit;
void test_generate_swit(uint32_t pin)
{
  assert(regs[1]==9U && !SX1278_APRS_RxActive());
  assert(irq_enabled[EXTI11_IRQn] && dio1_mode==GPIO_MODE_IT_RISING);
  if (!block_swit) HAL_GPIO_EXTI_Rising_Callback((uint16_t)pin);
}
static uint8_t bits[GMSK_DOWNLINK_MAX_PACKED_BYTES];
static size_t bit_count;

void HAL_GPIO_WritePin(GPIO_TypeDef *p, uint32_t pin, GPIO_PinState v)
{ if (v) p->ODR |= pin; else p->ODR &= ~pin; }
GPIO_PinState HAL_GPIO_ReadPin(GPIO_TypeDef *p, uint32_t pin)
{ return (p->ODR & pin) ? GPIO_PIN_SET : GPIO_PIN_RESET; }
void HAL_GPIO_Init(GPIO_TypeDef *p, const GPIO_InitTypeDef *g)
{ (void)p; if (g->Pin == DIO1_Pin) dio1_mode=g->Mode;
  if (g->Pin == DIO1_Pin && g->Mode == GPIO_MODE_IT_RISING) {
    mock_exti.IMR1 |= DIO1_Pin;
    mock_exti.RTSR1 |= DIO1_Pin;
  }
  if (g->Pin == DIO2_Pin) dio2_mode=g->Mode; }
void HAL_NVIC_ClearPendingIRQ(int i) { (void)i; }
void HAL_NVIC_DisableIRQ(int i) { irq_enabled[i]=0; }
void HAL_NVIC_EnableIRQ(int i) { irq_enabled[i]=1; }
uint32_t NVIC_GetEnableIRQ(int i) { return irq_enabled[i]; }
void HAL_NVIC_SetPriority(int i,unsigned a,unsigned b) { (void)i;(void)a;(void)b; }
uint32_t HAL_GetTick(void) { return tick; }
uint32_t HAL_RCC_GetPCLK1Freq(void) { return 16000000U; }
void HAL_Delay(uint32_t ms) { tick+=ms; }
uint32_t HAL_SPI_GetError(SPI_HandleTypeDef *h) { (void)h; return 0; }
HAL_StatusTypeDef HAL_SPI_TransmitReceive(SPI_HandleTypeDef *h,uint8_t *tx,
                                         uint8_t *rx,uint16_t n,uint32_t t)
{
  (void)h;(void)t; memset(rx,0,n);
  assert(n==2U);
  if (tx[0]==0x3e) {
    switch (regs[1]) {
      case 0x0c: rx[1]=0x18; break; /* User's failed FSRX snapshot. */
      case 0x0d: rx[1]=missing_rx_ready ? 0x90 : 0xd0; break;
      case 0x0a: case 0x0b: rx[1]=0xb0; break;
      default: rx[1]=0x80; break;
    }
  } else rx[1]=regs[tx[0]];
  return HAL_OK;
}
HAL_StatusTypeDef HAL_SPI_Transmit(SPI_HandleTypeDef *h,uint8_t *tx,
                                  uint16_t n,uint32_t t)
{
  unsigned reg=tx[0]&127U;
  (void)h;(void)t; assert(n==2U);
  if ((int)reg == failure_reg) { failure_reg=-1; return HAL_ERROR; }
  assert(reg!=1U || tx[1]!=0x0cU); /* RX must use the direct standby->RX path. */
  regs[reg]=tx[1];
  /* Model startup held before a preamble-triggered AGC/AFC sequence. A
     continuous listener must start even when no transmitter is on air. */
  if (reg==1U && tx[1]==0x0dU &&
      (stuck_rx_mode || (regs[0x0d]&7U)!=0U)) regs[1]=0x0cU;
  if (reg==1 && tx[1]==0x0b) memcpy(tx_regs,regs,sizeof(regs));
  return HAL_OK;
}

static void service(void *context)
{
  unsigned clock_enabled=*(unsigned *)context;
  ++tick; ++service_count;
  if (mode_error && service_count==10U) regs[1]=9U;
  /* Five DCLK falling edges per millisecond: sufficient timing simulation
     to verify exact bit order/count, no timer IRQ, and bounded cleanup. */
  for (unsigned i=0; i<5U && irq_enabled[EXTI11_IRQn] && clock_enabled; ++i) {
    assert(dio1_mode==GPIO_MODE_IT_FALLING);
    assert(dio2_mode==GPIO_MODE_OUTPUT_PP);
    HAL_GPIO_EXTI_Rising_Callback(DIO1_Pin); /* Must not advance TX. */
    HAL_GPIO_EXTI_Falling_Callback(DIO2_Pin); /* Unrelated EXTI line. */
    HAL_GPIO_EXTI_Falling_Callback(DIO1_Pin);
    if (symbol_count < bit_count) {
      unsigned expected=(bits[symbol_count/8U]>>(7U-symbol_count%8U))&1U;
      assert(mock_a.BSRR==(expected ? DIO2_Pin : DIO2_Pin<<16));
    }
    ++symbol_count;
  }
}

static void setup(void)
{
  uint8_t version;
  memset(regs,0,sizeof(regs)); regs[0x42]=0x12;
  tick=0; service_count=0; symbol_count=0; mode_error=0;
  assert(SX1278_APRS_Init(&version)==0 && version==0x12);
  assert(SX1278_APRS_RxStart()==0);
  assert(dio1_mode==GPIO_MODE_IT_RISING);
  assert(gmsk_downlink_encode((const uint8_t *)"ACK,EXPR",8,
                            bits,sizeof(bits),&bit_count)==0);
}

static void check_rx(void)
{
  assert(SX1278_APRS_RxActive()==1 && regs[1]==0x0d);
  assert(dio1_mode==GPIO_MODE_IT_RISING && dio2_mode==GPIO_MODE_INPUT);
  assert(regs[2]==0x1a && regs[3]==0x0b && regs[5]==20);
  assert(regs[0x30]==0 && regs[0x31]==0 && regs[0x0a]==0x49);
  assert(regs[0x0d]==0x08); /* AGC on, AFC off, no RSSI/preamble startup gate. */
  assert((regs[0x14]&0x20)!=0); /* Continuous DCLK explicitly enabled. */
}

static void feed_frame(const uint8_t *frame, int bytes)
{
  assert(bytes>0);
  assert(irq_enabled[EXTI11_IRQn] && dio1_mode==GPIO_MODE_IT_RISING);
  for (int i=0; i<bytes; ++i) {
    for (unsigned bit=0; bit<8U; ++bit) {
      HAL_GPIO_WritePin(DIO2_GPIO_Port,DIO2_Pin,
        (frame[i] & (0x80U>>bit)) ? GPIO_PIN_SET : GPIO_PIN_RESET);
      HAL_GPIO_EXTI_Falling_Callback(DIO1_Pin); /* RX samples rising only. */
      HAL_GPIO_EXTI_Rising_Callback(DIO2_Pin);
      HAL_GPIO_EXTI_Rising_Callback(DIO1_Pin);
    }
    ++tick;
  }
}

static void receive_command(const char *data, size_t length)
{
  uint8_t frame[86];
  UplinkCommand command;
  int bytes=uplink_frame_encode((const uint8_t *)data,length,frame,sizeof(frame));
  feed_frame(frame,bytes);
  assert(SX1278_APRS_RxPoll(&command)==1);
  assert(command.block_count*7U==length && memcmp(command.data,data,length)==0);
  assert(SX1278_APRS_RxPoll(&command)==0);
}

static void test_rx_diagnostics(void)
{
  char report[512];
  uint8_t frame[86];
  UplinkCommand command;
  int bytes=uplink_frame_encode((const uint8_t *)"EXPR   ",7,frame,sizeof(frame));
  tick+=5000U;
  SX1278_APRS_RxReport(report,sizeof(report));
  assert(strstr(report,"start rc=0 stage=ready mode=0D irq1=D0"));
  assert(strstr(report,"rxcfg=08"));
  assert(strstr(report,"irqtest=PASS"));
  SX1278_APRS_RxReport(report,sizeof(report));
  assert(strstr(report,"clocks=0 ") && strstr(report,"active=1 "));
  assert(strstr(report,"RX DIO peak=20 map=00 pkt=00"));
  assert(strstr(report,"imr=1 rise=1 pend=0 nvic=1 primask=0"));
  receive_command("EXPR   ",7U);
  SX1278_APRS_RxReport(report,sizeof(report));
  assert(strstr(report,"RX IRQ first:"));
  SX1278_APRS_RxReport(report,sizeof(report));
  assert(report[0]==0); /* No per-bit or per-poll log flood. */
  tick+=1000U;
  SX1278_APRS_RxReport(report,sizeof(report));
  assert(strstr(report,"pre=1 asm=1 ") && strstr(report,"ok=1 "));
  assert(irq_enabled[EXTI11_IRQn]); /* Reporting never masks sampling. */
  frame[bytes-1]^=1U;
  feed_frame(frame,bytes);
  assert(SX1278_APRS_RxPoll(&command)==0);
  tick+=1000U;
  SX1278_APRS_RxReport(report,sizeof(report));
  assert(strstr(report,"bch_crc_bad=1 "));
  frame[bytes-1]^=1U;
  feed_frame(frame,21); /* Truncated after ASM and one payload byte. */
  tick+=101U;
  assert(SX1278_APRS_RxPoll(&command)==0);
  tick+=1000U;
  SX1278_APRS_RxReport(report,sizeof(report));
  assert(strstr(report,"gap=1 "));
  feed_frame(frame,bytes);
  tick+=101U; /* Completed frames survive foreground delays. */
  assert(SX1278_APRS_RxPoll(&command)==1);
  tick+=1000U;
  SX1278_APRS_RxReport(report,sizeof(report));
  assert(strstr(report,"ok=2 ") && strstr(report,"gap=1 "));
  SX1278_APRS_RxReport(report,sizeof(report));
  assert(report[0]==0);
  tick+=20U;
  regs[0x11]=100U; /* A short signal at -50 dBm between printed reports. */
  assert(SX1278_APRS_RxPoll(&command)==0);
  tick+=20U;
  regs[0x11]=250U;
  assert(SX1278_APRS_RxPoll(&command)==0);
  tick+=5000U;
  SX1278_APRS_RxReport(report,sizeof(report));
  assert(strstr(report,"RX RSSI peak=-50.0 samples=2 errors=0"));
}

static void test_rx_start_recovery(void)
{
  UplinkCommand command;
  char report[384];
  uint32_t start;
  setup();
  regs[0x14]=0x0b;
  assert(SX1278_APRS_RxStart()==0 && regs[0x14]==0x2b);
  stuck_rx_mode=1U;
  start=tick;
  assert(SX1278_APRS_RxStart()==-4);
  assert(tick-start>=20U && tick-start<100U);
  assert(!SX1278_APRS_RxActive() && !irq_enabled[EXTI11_IRQn] && regs[1]==9U);
  SX1278_APRS_RxReport(report,sizeof(report));
  assert(strstr(report,"rc=-4 stage=mode mode=0C irq1=18") && strstr(report,"wanted=1"));
  stuck_rx_mode=0U;
  assert(SX1278_APRS_RxPoll(&command)==0 && !SX1278_APRS_RxActive());
  tick+=1000U;
  assert(SX1278_APRS_RxPoll(&command)==0);
  check_rx();
  receive_command("EXPR   ",7U);

  missing_rx_ready=1U;
  assert(SX1278_APRS_RxStart()==-5);
  assert(!SX1278_APRS_RxActive() && regs[1]==9U);
  SX1278_APRS_RxReport(report,sizeof(report));
  assert(strstr(report,"rc=-5 stage=rx-ready mode=0D irq1=90"));
  SX1278_APRS_RxStop(); /* WSPR or explicit stop must cancel retry. */
  missing_rx_ready=0U;
  tick+=2000U;
  assert(SX1278_APRS_RxPoll(&command)==0 && !SX1278_APRS_RxActive());
  assert(regs[1]==9U && !irq_enabled[EXTI11_IRQn]);
  assert(SX1278_APRS_RxStart()==0);
  check_rx();

  /* Configuration SPI failure follows the same cleanup/retry path. */
  failure_reg=0x12;
  assert(SX1278_APRS_RxStart()==-3 && regs[1]==9U);
  tick+=1000U;
  assert(SX1278_APRS_RxPoll(&command)==0);
  check_rx();
}

int main(void)
{
  unsigned clocks=1;
  uint8_t before[128];
  int duration;
  setup();
  test_rx_diagnostics();
  test_rx_start_recovery();
  setup();
  receive_command("EXPR   ",7U);
  receive_command("TRANPD BG7ZDQ APZDQ7 HB-2A  HELLO  ",35U);
  memcpy(before,regs,sizeof(regs));
  duration=SX1278_APRS_PrepareGmsk(bits,bit_count,4800,1200);
  assert(duration==893 && memcmp(before,regs,sizeof(regs))==0);
  assert(SX1278_APRS_SendGmsk(144925000,0,service,&clocks)==-2);
  assert(SX1278_APRS_SendGmsk(144925000,1400,service,&clocks)==0);
  assert(symbol_count==bit_count+4U && service_count>0);
  assert(tx_regs[2]==0x1a && tx_regs[3]==0x0b && tx_regs[5]==20);
  assert(tx_regs[0x0a]==0x49 && tx_regs[0x30]==0 && tx_regs[0x31]==0);
  assert(tx_regs[6]==0x24 && tx_regs[7]==0x3b && tx_regs[8]==0x33);
  check_rx();
  receive_command("EXPR   ",7U);
  assert(SX1278_APRS_SendGmsk(144925000,1400,service,&clocks)==-1);
  /* Missing DCLK must timeout and turn RF off, with RX restored. */
  setup(); clocks=0;
  assert(SX1278_APRS_PrepareGmsk(bits,bit_count,4800,1200)>0);
  assert(SX1278_APRS_SendGmsk(144925000,1400,service,&clocks)==-4);
  check_rx();
  /* SPI setup failure and unexpected radio state also restore RX. */
  setup(); clocks=1;
  assert(SX1278_APRS_PrepareGmsk(bits,bit_count,4800,1200)>0);
  failure_reg=0x04;
  assert(SX1278_APRS_SendGmsk(144925000,1400,service,&clocks)==-3);
  check_rx();
  setup(); mode_error=1;
  assert(SX1278_APRS_PrepareGmsk(bits,bit_count,4800,1200)>0);
  assert(SX1278_APRS_SendGmsk(144925000,1400,service,&clocks)==-8);
  check_rx();
  /* A broken EXTI vector cannot be reported as a working receiver. */
  SX1278_APRS_RxStop();
  uint8_t version;
  char report[512];
  assert(SX1278_APRS_Init(&version)==0);
  block_swit=1U;
  assert(SX1278_APRS_RxStart()==-6 && !SX1278_APRS_RxActive());
  SX1278_APRS_RxReport(report,sizeof(report));
  assert(strstr(report,"stage=exti-selftest") && strstr(report,"irqtest=FAIL"));
  assert(regs[1]==9U && !irq_enabled[EXTI11_IRQn]);
  puts("SX1278 U5 edge callbacks, uplink RX, GMSK TX, RX restore and failure cleanup: passed");
  return 0;
}
