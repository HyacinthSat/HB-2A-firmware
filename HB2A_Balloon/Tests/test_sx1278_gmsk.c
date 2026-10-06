#include "main.h"
#include "spi.h"
#include "sx1278_aprs.h"
#include "gmsk_downlink.h"
#include "app_config.h"
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
static unsigned irq_enabled[32], service_count, mode_error;
static uint8_t tx_fifo[66], wire[8192];
static unsigned tx_head, tx_size, wire_bytes, tx_fraction, tx_shift_bits;
static unsigned tx_done, tx_underrun, tx_delay_ms = 1U, suppress_packet_sent;
static uint32_t tx_started_tick, tx_finished_tick;
static int failure_reg = -1;
static unsigned stuck_rx_mode, missing_rx_ready;
static unsigned block_swit;
#if APP_UPLINK_RX_FIFO
static uint8_t fifo[64];
static unsigned fifo_size, fifo_pos, fifo_sync, fifo_overrun;
static unsigned fifo_preamble, missing_pll;
static int read_failure = -1;
#endif
void test_generate_swit(uint32_t pin)
{
  assert(regs[1]==9U && !SX1278_APRS_RxActive());
  assert(irq_enabled[EXTI11_IRQn] && dio1_mode==GPIO_MODE_IT_RISING);
  if (!block_swit) HAL_GPIO_EXTI_Rising_Callback((uint16_t)pin);
}
static uint8_t bits[GMSK_HIGH_SPEED_MAX_PACKED_BYTES];
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
  if (tx[0]==0x3f && (regs[1]==0x0b || regs[1]==9U) &&
      (regs[0x31]&0x40U)) {
    rx[1]=(tx_size==0U ? 0x40U : 0U) |
          (tx_size>(regs[0x35]&0x3fU) ? 0x20U : 0U) |
          (tx_done && !suppress_packet_sent ? 8U : 0U);
    return HAL_OK;
  }
#if APP_UPLINK_RX_FIFO
  if ((int)tx[0]==read_failure) { read_failure=-1; return HAL_ERROR; }
  if (tx[0]==0U) { assert(fifo_pos<fifo_size); rx[1]=fifo[fifo_pos++]; return HAL_OK; }
  if (tx[0]==0x3f && regs[0x31]==0x40) {
    rx[1]=(fifo_pos==fifo_size ? 0x40U : 0U) | (fifo_overrun ? 0x10U : 0U);
    return HAL_OK;
  }
#endif
  if (tx[0]==0x3e) {
    switch (regs[1]) {
      case 0x0c: rx[1]=0x18; break; /* Waiting for the acquisition trigger. */
      case 0x0d: rx[1]=missing_rx_ready ? 0x90 : 0xd0; break;
      case 0x0a: case 0x0b: rx[1]=0xb0; break;
      default: rx[1]=0x80; break;
    }
  } else rx[1]=regs[tx[0]];
#if APP_UPLINK_RX_FIFO
  if (missing_pll && (regs[1]==0x0c || regs[1]==0x0d)) rx[1]&=(uint8_t)~0x10U;
  if (tx[0]==0x3e && regs[1]==0x0d && fifo_sync) rx[1]|=3U;
  else if (tx[0]==0x3e && regs[1]==0x0d && fifo_preamble) rx[1]|=2U;
#endif
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
  if (reg==0x3f && (tx[1]&0x10)) tx_head=tx_size=0;
  if (reg==0U && (regs[0x31]&0x40U)) {
    assert(tx_size<sizeof(tx_fifo));
    tx_fifo[(tx_head+tx_size)%sizeof(tx_fifo)]=tx[1];
    ++tx_size;
  }
#if APP_UPLINK_RX_FIFO
  if (reg==0x3f && (tx[1]&0x10)) fifo_pos=fifo_size=fifo_overrun=0;
  if (reg==1 && tx[1]==9) fifo_sync=fifo_preamble=0;
#endif
  /* Model startup held before a preamble-triggered AGC/AFC sequence. A
     continuous listener must start even when no transmitter is on air. */
  if (reg==1U && tx[1]==0x0dU &&
      (stuck_rx_mode || (regs[0x0d]&7U)!=0U)) regs[1]=0x0cU;
#if APP_UPLINK_RX_FIFO
  if (reg==1U && tx[1]==0x0dU && stuck_rx_mode) regs[1]=9U;
#endif
  if (reg==1 && tx[1]==0x0b) {
    memcpy(tx_regs,regs,sizeof(regs));
    tx_started_tick=tick;
  }
  if (reg==1 && tx[1]==9U && wire_bytes!=0U) tx_finished_tick=tick;
  return HAL_OK;
}

static void advance_tx(unsigned elapsed_ms)
{
  unsigned divisor=((unsigned)regs[2]<<8U)|regs[3];
  unsigned length=((unsigned)(regs[0x31]&7U)<<8U)|regs[0x32];
  tx_fraction+=(32000000U/divisor)*elapsed_ms;
  while (tx_fraction>=1000U && regs[1]==0x0b && !tx_done) {
    tx_fraction-=1000U;
    if (tx_shift_bits==0U) {
      if (tx_size==0U) { tx_underrun=1U; break; }
      assert(wire_bytes<sizeof(wire));
      wire[wire_bytes++]=tx_fifo[tx_head];
      tx_head=(tx_head+1U)%sizeof(tx_fifo);
      --tx_size;
      tx_shift_bits=8U;
    }
    if (--tx_shift_bits==0U && length!=0U && wire_bytes==length) tx_done=1U;
  }
}

static void service(void *context)
{
  unsigned enabled=*(unsigned *)context;
  tick+=tx_delay_ms; ++service_count;
  if (mode_error && service_count==10U) regs[1]=9U;
  assert(!irq_enabled[EXTI11_IRQn]);
  assert(dio1_mode==GPIO_MODE_INPUT && dio2_mode==GPIO_MODE_INPUT);
  if (enabled) advance_tx(tx_delay_ms);
}

static void setup(void)
{
  uint8_t version;
  memset(regs,0,sizeof(regs)); regs[0x42]=0x12;
  tick=0; service_count=0; mode_error=0;
  tx_head=tx_size=wire_bytes=tx_fraction=tx_shift_bits=0;
  tx_done=tx_underrun=suppress_packet_sent=0;
  tx_delay_ms=1U;
  assert(SX1278_APRS_Init(&version)==0 && version==0x12);
  assert(SX1278_APRS_RxStart()==0);
  assert(dio1_mode==GPIO_MODE_IT_RISING);
  assert(gmsk_downlink_encode((const uint8_t *)"ACK,EXPR",8,
                            bits,sizeof(bits),&bit_count)==0);
}

static void check_rx(void)
{
  assert(SX1278_APRS_RxActive()==1);
  assert(regs[1]==0x0d || (APP_UPLINK_RX_FIFO && regs[1]==0x0c));
  assert(dio1_mode==GPIO_MODE_IT_RISING && dio2_mode==GPIO_MODE_INPUT);
  assert(regs[2]==0x1a && regs[3]==0x0b && regs[5]==20);
  assert(regs[0x30]==0 && regs[0x31]==(APP_UPLINK_RX_FIFO ? 0x40 : 0) && regs[0x0a]==0x49);
#if APP_UPLINK_RX_FIFO
  assert(regs[0x27]==0x33 && regs[0x32]==0 && regs[0x35]==0x87);
  assert(regs[0x0d]==0x1e && regs[0x13]==0x0b && regs[0x1a]==1);
  assert(memcmp(regs+0x28,"\x1a\xcf\xfc\x1d",4)==0);
#else
  assert(regs[0x0d]==0x08); /* AGC on, AFC off, no RSSI/preamble startup gate. */
#endif
  assert((regs[0x14]&0x20)!=0); /* Continuous DCLK explicitly enabled. */
}

static void feed_frame(const uint8_t *frame, int bytes)
{
  assert(bytes>0);
  assert(irq_enabled[EXTI11_IRQn] && dio1_mode==GPIO_MODE_IT_RISING);
#if APP_UPLINK_RX_FIFO
  assert(bytes>=20 && (unsigned)(bytes-20)<=sizeof(fifo));
  assert(regs[0x31]==0x40 && (regs[1]==0x0d || regs[1]==0x0c));
  regs[1]=0x0d; /* Preamble completes the simulated AFC/AGC acquisition. */
  assert(memcmp(frame+16,regs+0x28,4)==0);
  fifo_sync=1; fifo_size=(unsigned)bytes-20; fifo_pos=0;
  memcpy(fifo,frame+20,fifo_size);
  /* Deliberately do not fire DIO1: the user's DCLK is absent. */
#else
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
#endif
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

void TIM2_IRQHandler(void);
static void afsk_service(void *context)
{
  (void)context;
  ++tick;
  assert(regs[0x31]==0 && regs[0x27]==0 && regs[0x0a]==9);
  for (unsigned i=0; i<27U && mock_tim.CR1!=0; ++i) {
    mock_tim.SR=TIM_SR_UIF;
    TIM2_IRQHandler();
  }
}

static void test_aprs_rx_restore(void)
{
  assert(SX1278_APRS_PrepareAddressedPath("BG7ZDQ", "APZDQ7", "APZDQ7",
                                        (const uint8_t *)"HELLO",5)>0);
  assert(SX1278_APRS_SendWithContext(144925000,2000,afsk_service,NULL)==0);
  assert(tx_regs[0x31]==0 && tx_regs[2]==2 && tx_regs[5]==49);
  check_rx();
  receive_command("TRANPD BG7ZDQ APZDQ7 APZDQ7 HELLO  ",35);
}

#if !APP_UPLINK_RX_FIFO
static void test_rx_diagnostics(void)
{
  char report[768];
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
#else
static void test_rx_fifo(void)
{
  uint8_t frame[86];
  UplinkCommand command;
  char report[512];
  int length=uplink_frame_encode((const uint8_t *)"TRANPD BG7ZDQ APZDQ7 APZDQ7 HELLO  ",35,frame,sizeof(frame));
  SX1278_APRS_RxReport(report,sizeof(report));
  assert(strstr(report,"path=FIFO") && strstr(report,"irqtest=PASS"));
  /* Ten frames without a single physical DIO1 interrupt must still decode. */
  for (unsigned i=0; i<10; ++i) receive_command("TRANPD BG7ZDQ APZDQ7 APZDQ7 HELLO  ",35U);
  SX1278_APRS_RxReport(report,sizeof(report));
  tick+=5000;
  SX1278_APRS_RxReport(report,sizeof(report));
  assert(strstr(report,"clocks=0 ") && strstr(report,"ok=10 "));
  assert(strstr(report,"RX FIFO bytes=420 irq=0 spi_errors=0"));
  /* Partial arrivals must wait for the actual CRC, not FIFO-empty or an IRQ. */
  feed_frame(frame,28);
  HAL_GPIO_EXTI_Rising_Callback(DIO1_Pin);
  assert(SX1278_APRS_RxPoll(&command)==0);
  for (int i=28; i<length; ++i) {
    fifo[0]=frame[i]; fifo_size=1; fifo_pos=0; tick+=2;
    assert(SX1278_APRS_RxPoll(&command)==(i==length-1));
  }
  assert(memcmp(command.data,"TRANPD BG7ZDQ APZDQ7 APZDQ7 HELLO  ",35)==0);
  /* Bad CRC, unknown command, truncated input and FIFO overrun rearm RX. */
  frame[length-1]^=1;
  feed_frame(frame,length);
  assert(SX1278_APRS_RxPoll(&command)==0 && !fifo_sync);
  frame[length-1]^=1;
  feed_frame(frame,21);
  assert(SX1278_APRS_RxPoll(&command)==0);
  tick+=101;
  assert(SX1278_APRS_RxPoll(&command)==0 && !fifo_sync);
  uint8_t bad[86];
  int badlen=uplink_frame_encode((const uint8_t *)"UNKNOWN",7,bad,sizeof(bad));
  feed_frame(bad,badlen);
  assert(SX1278_APRS_RxPoll(&command)==0 && !fifo_sync);
  feed_frame(frame,28); fifo_overrun=1;
  assert(SX1278_APRS_RxPoll(&command)==0 && !fifo_sync && !fifo_overrun);
  feed_frame(frame,length); read_failure=0;
  assert(SX1278_APRS_RxPoll(&command)==0 && !fifo_sync);
  receive_command("EXPR   ",7);
  check_rx();
  /* A received FIFO notification is logged separately from bit clocks. */
  SX1278_APRS_RxReport(report,sizeof(report));
  SX1278_APRS_RxReport(report,sizeof(report));
  assert(strstr(report,"DIO1 FIFO level"));
  tick+=5000;
  SX1278_APRS_RxReport(report,sizeof(report));
  assert(strstr(report,"bch_crc_bad=1 ") && strstr(report,"header_bad=1 "));
  assert(strstr(report,"gap=1 overflow=1 ") && strstr(report,"spi_errors=1"));
  /* Acquired preamble but no ASM: record signed frequency estimates, then
     clear that acquisition so the next packet can perform AFC again. */
  fifo_preamble=1; regs[1]=0x0d;
  regs[0x1b]=0xff; regs[0x1c]=0xf0;
  regs[0x1d]=0x00; regs[0x1e]=0x20;
  assert(SX1278_APRS_RxPoll(&command)==0);
  tick+=251;
  assert(SX1278_APRS_RxPoll(&command)==0 && !fifo_preamble);
  check_rx();
  SX1278_APRS_RxReport(report,sizeof(report));
  tick+=5000;
  SX1278_APRS_RxReport(report,sizeof(report));
  assert(strstr(report,"nosync=1"));
  assert(strstr(report,"valid=1 afc_hz=-976 fei_est_hz=1953"));
  receive_command("EXPR   ",7);
}
#endif

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
#if APP_UPLINK_RX_FIFO
  assert(strstr(report,"rc=-4 stage=mode mode=09") && strstr(report,"wanted=1"));
#else
  assert(strstr(report,"rc=-4 stage=mode mode=0C irq1=18") && strstr(report,"wanted=1"));
#endif
  stuck_rx_mode=0U;
  assert(SX1278_APRS_RxPoll(&command)==0 && !SX1278_APRS_RxActive());
  tick+=1000U;
  assert(SX1278_APRS_RxPoll(&command)==0);
  check_rx();
  receive_command("EXPR   ",7U);

  missing_rx_ready=1U;
#if APP_UPLINK_RX_FIFO
  /* No RxReady before a preamble is normal; missing PLL lock is not. */
  assert(SX1278_APRS_RxStart()==0);
  tick+=1000;
  assert(SX1278_APRS_RxPoll(&command)==0 && SX1278_APRS_RxActive());
  missing_pll=1;
  assert(SX1278_APRS_RxStart()==-4);
  assert(!SX1278_APRS_RxActive() && regs[1]==9U);
  missing_pll=0;
#else
  assert(SX1278_APRS_RxStart()==-5);
  assert(!SX1278_APRS_RxActive() && regs[1]==9U);
  SX1278_APRS_RxReport(report,sizeof(report));
  assert(strstr(report,"rc=-5 stage=rx-ready mode=0D irq1=90"));
#endif
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

static void fail_stop_once(void *context)
{
  service(context);
  if (tx_done) failure_reg = 1;
}

static void test_reset_recovery(void)
{
  unsigned clocks = 1;
  UplinkCommand command;
  setup();
  assert(SX1278_APRS_PrepareGmsk(bits, bit_count, 4800, 1200) > 0);
  assert(SX1278_APRS_SendGmsk(144925000, 1400, fail_stop_once, &clocks) < 0);
  assert(!SX1278_APRS_Ready() && !SX1278_APRS_RxActive());
  assert(SX1278_APRS_RxPoll(&command) == 0 && !SX1278_APRS_Ready());
  tick += 1000U;
  failure_reg = 0x4B; /* First full initialization also fails. */
  assert(SX1278_APRS_RxPoll(&command) == 0 && !SX1278_APRS_Ready());
  tick += 1000U;
  assert(SX1278_APRS_RxPoll(&command) == 0 && SX1278_APRS_Ready());
  check_rx();
  receive_command("EXPR   ", 7U);
  test_aprs_rx_restore();
}

static void check_wire(void)
{
  size_t bytes=(bit_count+7U)/8U;
  assert(wire_bytes==bytes && tx_shift_bits==0U);
  assert(memcmp(wire,bits,bit_count/8U)==0);
  if ((bit_count&7U)!=0U) {
    uint8_t mask=(uint8_t)(0xffU<<(8U-(bit_count&7U)));
    assert(wire[bytes-1U]==(uint8_t)(bits[bytes-1U]&mask));
  }
}

static void test_fifo_profiles(void)
{
  static const uint32_t rates[]={4800,9600,19200,38400,57600,115200};
  static uint8_t frames[8U*GMSK_HIGH_SPEED_DATA_BYTES];
  unsigned enabled=1U;
  char report[768];
  for (unsigned i=0;i<sizeof(frames);++i) frames[i]=(uint8_t)(i*37U);
  for (unsigned profile=0;profile<6U;++profile) {
    for (unsigned code=0;code<4U;++code) {
      for (unsigned depth=1U;depth<=8U;depth+=7U) {
        setup();
        assert(gmsk_high_speed_encode(frames,depth,(uint8_t)code,(uint8_t)depth,
                                      bits,sizeof(bits),&bit_count)==0);
        /* Nonzero unused input bits must never leak into the padded tail. */
        if (bit_count&7U) bits[bit_count/8U]|=(uint8_t)(0xffU>>(bit_count&7U));
        int duration=SX1278_APRS_PrepareGmsk(bits,bit_count,rates[profile],rates[profile]/4U);
        assert(duration>0);
        assert(SX1278_APRS_SendGmsk(144925000,(uint32_t)duration+500U,service,&enabled)==0);
        check_wire();
        assert(tx_regs[0x0a]==0x49 && tx_regs[0x27]==0 && tx_regs[0x30]==0);
        assert(tx_regs[0x25]==0 && tx_regs[0x26]==0);
        unsigned length=((tx_regs[0x31]&7U)<<8U)|tx_regs[0x32];
        assert(length==(wire_bytes<=2047U ? wire_bytes : 0U));
        assert(tx_finished_tick-tx_started_tick>=(uint32_t)duration);
        SX1278_APRS_TxReport(report,sizeof(report));
        assert(strstr(report,"GMSK FIFO queued=")!=NULL);
        assert(strstr(report,length ? "packet_sent=1 drained=0" : "packet_sent=0 drained=1"));
        check_rx();
      }
    }
  }
  setup();
  assert(SX1278_APRS_PrepareGmsk(bits,bit_count,115200,28800)>0);
  tx_delay_ms=10U; /* More than a full FIFO at 115200: detect the gap. */
  assert(SX1278_APRS_SendGmsk(144925000,1500,service,&enabled)==-11);
  check_rx();

  setup();
  assert(SX1278_APRS_PrepareGmsk(bits,bit_count,4800,1200)>0);
  suppress_packet_sent=1U;
  assert(SX1278_APRS_SendGmsk(144925000,1400,service,&enabled)==-4);
  check_rx();
}

int main(void)
{
  unsigned clocks=1;
  uint8_t before[128];
  int duration;
  setup();
#if APP_UPLINK_RX_FIFO
  test_rx_fifo();
#else
  test_rx_diagnostics();
#endif
  test_rx_start_recovery();
  test_reset_recovery();
  test_fifo_profiles();
  setup();
  receive_command("EXPR   ",7U);
  receive_command("TRANPD BG7ZDQ APZDQ7 APZDQ7 HELLO  ",35U);
  test_aprs_rx_restore();
  memcpy(before,regs,sizeof(regs));
  duration=SX1278_APRS_PrepareGmsk(bits,bit_count,4800,1200);
  assert(duration==894 && memcmp(before,regs,sizeof(regs))==0);
  assert(SX1278_APRS_SendGmsk(144925000,0,service,&clocks)==-2);
  assert(SX1278_APRS_SendGmsk(144925000,1400,service,&clocks)==0);
  assert(wire_bytes==(bit_count+7U)/8U && service_count>0 && tx_done);
  assert(tx_shift_bits==0U && !tx_underrun);
  assert(memcmp(wire,bits,wire_bytes)==0);
  assert(tx_regs[2]==0x1a && tx_regs[3]==0x0b && tx_regs[5]==20);
  assert(tx_regs[0x0a]==0x49 && tx_regs[0x30]==0 && tx_regs[0x31]==0x42);
  assert(tx_regs[0x25]==0 && tx_regs[0x26]==0 && tx_regs[0x27]==0);
  assert(tx_regs[0x32]==0x18 && tx_regs[0x35]==0xa0);
  assert(tx_regs[6]==0x24 && tx_regs[7]==0x3b && tx_regs[8]==0x33);
  check_rx();
  receive_command("EXPR   ",7U);
  assert(SX1278_APRS_SendGmsk(144925000,1400,service,&clocks)==-1);
  /* A stalled FIFO must timeout and turn RF off, with RX restored. */
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
  puts("SX1278 RX, FIFO GMSK (six rates/four codes), underrun, drain, APRS restore: passed");
  return 0;
}
