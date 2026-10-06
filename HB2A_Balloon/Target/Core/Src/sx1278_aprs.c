/* Semtech SX1276/77/78/79 Rev.4, sections 4.2.1 and 4.2.12:
   unshaped continuous FSK accepts asynchronous modulation on DIO2.
   DIO2 is a Bell 202 AUDIO square wave, not AX.25 data at 1200 bit/s.
   Board: PA12=DIO2, PB12=NSS, PA15=NRESET, PA_BOOST -> VHF filter.
   RXTX pin 20 directly controls the external antenna switch.
   U14 supplies an AC-coupled external 32 MHz reference to XTA. */
#include "sx1278_aprs.h"
#include "ax25_afsk.h"
#include "uplink_protocol.h"
#include "app_config.h"
#include "main.h"
#include "spi.h"
#include <stdio.h>
#include <string.h>

#define REG_MODE 0x01U
#define REG_BITRATE_MSB 0x02U
#define REG_IRQ1 0x3EU
#define MODE_SLEEP 0x08U
#define MODE_STANDBY 0x09U
#define MODE_FSTX 0x0AU
#define MODE_TX 0x0BU
#define MODE_RX 0x0DU
#define RX_PA_RAMP_GMSK_BT05 0x49U
#define RX_CONFIG_CONTINUOUS 0x08U
#define MODE_CHANGE_TIMEOUT_MS 20U
#define UPLINK_FREQUENCY_HZ 435400000UL
#define UPLINK_BITRATE 6667U
#define UPLINK_FDEV 20U
#define GMSK_MAX_PACKED_BYTES 600U
#define TX_KIND_AFSK 0U
#define TX_KIND_GMSK 1U

static AFSK_Stream stream;
static uint8_t frame[AX25_MAX_FRAME];
static uint8_t ready;
static volatile uint8_t playing;
static volatile uint8_t rx_active;
static uint8_t rx_wanted;
static UplinkDecoder rx_decoder;
static volatile uint32_t rx_last_bit_tick;
static volatile struct {
  uint32_t clocks, changes, preambles, syncs, header_bad;
  uint32_t valid, rejected, gaps, overflows;
} rx_stats;
static uint8_t rx_previous_bit, rx_irq_reported;
static uint32_t rx_report_tick, rx_report_events;
static uint32_t rx_attempt_tick, rx_attempts;
static int rx_start_result;
static const char *rx_start_stage = "idle";
static uint8_t rx_start_mode, rx_start_irq1, rx_start_config, rx_start_notice;
static volatile uint8_t rx_irq_probe, rx_irq_probe_seen;
static uint8_t rx_irq_probe_result; /* 0=untested, 1=pass, 2=fail */
static uint32_t rx_rssi_tick, rx_rssi_samples, rx_rssi_errors;
static uint8_t rx_rssi_peak = 255U;
static uint32_t timer_base, timer_remainder, timer_fraction;
static uint8_t version_reads[3];
static uint8_t gmsk_bits[GMSK_MAX_PACKED_BYTES];
static size_t gmsk_bit_count;
static volatile size_t gmsk_bit_index;
static volatile uint8_t gmsk_flush_clocks;
static uint32_t gmsk_bitrate;
static uint32_t gmsk_deviation_hz;
static uint8_t tx_kind;
static struct {
  uint8_t mode, irq1, irq2, datamod, frf[3], pa, ramp, ocp, packet, tcxo, dac, fdev[2];
  uint8_t valid;
  uint8_t direct_fallback;
  int result;
} tx_state;
static volatile uint32_t emitted_samples, data_edges;
static int previous_level;
static void (*legacy_service)(void);
static void level_write(int level);

static void legacy_service_adapter(void *context)
{
  (void)context;
  if (legacy_service != NULL) legacy_service();
}

/* STM32U5 HAL dispatches separate edge callbacks, not HAL_GPIO_EXTI_Callback.
   RX samples DIO2 on the recovered DIO1 rising edge. */
void HAL_GPIO_EXTI_Rising_Callback(uint16_t pin)
{
  uint8_t bit, preamble, collecting;
  if (pin != DIO1_Pin) return;
  if (rx_irq_probe != 0U) { rx_irq_probe_seen = 1U; return; }
  if (rx_active == 0U) return;
  bit = (uint8_t)HAL_GPIO_ReadPin(DIO2_GPIO_Port, DIO2_Pin);
  if (rx_stats.clocks != 0U && bit != rx_previous_bit) ++rx_stats.changes;
  rx_previous_bit = bit;
  ++rx_stats.clocks;
  rx_last_bit_tick = HAL_GetTick();
  preamble = rx_decoder.preamble_seen;
  collecting = rx_decoder.collecting;
  uplink_decoder_push_bit(&rx_decoder, bit);
  if (preamble == 0U && rx_decoder.preamble_seen != 0U) ++rx_stats.preambles;
  if (collecting == 0U && rx_decoder.collecting != 0U) ++rx_stats.syncs;
  if (collecting != 0U && rx_decoder.collecting == 0U) ++rx_stats.overflows;
  if (rx_decoder.raw_count == 64U && rx_decoder.expected_blocks == 0U)
    ++rx_stats.header_bad;
}

void HAL_GPIO_EXTI_Falling_Callback(uint16_t pin)
{
  if (pin != DIO1_Pin) return;
  if (tx_kind == TX_KIND_GMSK && playing != 0U) {
    /* SX1278 continuous shaped TX samples DATA on the rising DCLK edge.
       Change DATA on the falling edge, half a symbol before that sample. */
    if (gmsk_bit_index < gmsk_bit_count) {
      int level = (gmsk_bits[gmsk_bit_index >> 3U] >>
                   (7U - (gmsk_bit_index & 7U))) & 1U;
      level_write(level);
      ++gmsk_bit_index;
      ++emitted_samples;
      if (level != previous_level) ++data_edges;
      previous_level = level;
    } else if (++gmsk_flush_clocks >= 4U) {
      /* Last data bit was sampled on the preceding rising edge. Allow the
         Gaussian filter to drain before foreground switches RF off. */
      playing = 0U;
      HAL_NVIC_DisableIRQ(EXTI11_IRQn);
    }
    return;
  }
}
/* Preserve the first failure through standby cleanup and diagnostic reads. */
static struct {
  uint8_t stage, reg, expected, actual;
  uint32_t spi_error;
} first_failure;

static int reg_read(uint8_t reg, uint8_t *value)
{
  uint8_t tx[2] = {reg, 0U}, rx[2];
  HAL_GPIO_WritePin(SX1278_NSS_GPIO_Port, SX1278_NSS_Pin, GPIO_PIN_RESET);
  HAL_StatusTypeDef status = HAL_SPI_TransmitReceive(&hspi2, tx, rx, 2U, 5U);
  HAL_GPIO_WritePin(SX1278_NSS_GPIO_Port, SX1278_NSS_Pin, GPIO_PIN_SET);
  if (status != HAL_OK) return -1;
  *value = rx[1];
  return 0;
}

static int reg_write(uint8_t reg, uint8_t value)
{
  uint8_t tx[2] = {(uint8_t)(reg | 0x80U), value};
  HAL_GPIO_WritePin(SX1278_NSS_GPIO_Port, SX1278_NSS_Pin, GPIO_PIN_RESET);
  HAL_StatusTypeDef status = HAL_SPI_Transmit(&hspi2, tx, 2U, 5U);
  HAL_GPIO_WritePin(SX1278_NSS_GPIO_Port, SX1278_NSS_Pin, GPIO_PIN_SET);
  return status == HAL_OK ? 0 : -1;
}

static int checked_write(uint8_t reg, uint8_t value)
{
  uint8_t actual = 0xFFU;
  uint8_t stage = 0U;
  if (reg_write(reg, value)) stage = 1U;
  else {
    uint32_t start = HAL_GetTick();
    /* RegOpMode reflects an asynchronous transition. Write once, then wait
       for its readback; ordinary configuration registers must match at once.
       ModeReady/TxReady are checked separately by the existing callers. */
    for (;;) {
      if (reg_read(reg, &actual)) { stage = 2U; break; }
      if (actual == value) break;
      if (reg != REG_MODE) { stage = 3U; break; }
      if (HAL_GetTick() - start >= MODE_CHANGE_TIMEOUT_MS) { stage = 4U; break; }
      HAL_Delay(1U);
    }
  }
  if (stage != 0U && first_failure.stage == 0U) {
    first_failure.stage = stage;
    first_failure.reg = reg;
    first_failure.expected = value;
    first_failure.actual = actual;
    first_failure.spi_error = HAL_SPI_GetError(&hspi2);
  }
  return stage == 0U ? 0 : -1;
}

static int wait_flag(uint8_t mask)
{
  uint32_t start = HAL_GetTick();
  uint8_t flags;
  do {
    if (reg_read(REG_IRQ1, &flags)) return -1;
    if ((flags & mask) == mask) return 0;
    HAL_Delay(1U);
  } while (HAL_GetTick() - start < 20U);
  return -1;
}

static void level_write(int level)
{
  DIO2_GPIO_Port->BSRR = level ? DIO2_Pin : (uint32_t)DIO2_Pin << 16;
}

static void data_pin(uint32_t mode)
{
  GPIO_InitTypeDef gpio = {0};
  gpio.Pin = DIO2_Pin;
  gpio.Mode = mode;
  gpio.Pull = GPIO_NOPULL;
  gpio.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(DIO2_GPIO_Port, &gpio);
}

static void clock_pin(uint32_t mode)
{
  GPIO_InitTypeDef gpio = {0};
  gpio.Pin = DIO1_Pin;
  gpio.Mode = mode;
  gpio.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(DIO1_GPIO_Port, &gpio);
}

static uint32_t next_period(void)
{
  uint32_t period = timer_base;
  timer_fraction += timer_remainder;
  if (timer_fraction >= AFSK_SAMPLE_HZ) {
    timer_fraction -= AFSK_SAMPLE_HZ;
    ++period;
  }
  return period - 1U;
}

void TIM2_IRQHandler(void)
{
  if ((TIM2->SR & TIM_SR_UIF) == 0U) return;
  TIM2->SR = 0U;
  if (!playing) return;
  int level = AFSK_NextSample(&stream);
  if (level < 0) {
    TIM2->CR1 = 0U;
    TIM2->DIER = 0U;
    level_write(0);
    playing = 0U;
  } else {
    level_write(level);
    ++emitted_samples;
    if (level != previous_level) ++data_edges;
    previous_level = level;
    TIM2->ARR = next_period();
  }
}

static int stop_radio(void)
{
  TIM2->CR1 = 0U;
  TIM2->DIER = 0U;
  HAL_NVIC_DisableIRQ(EXTI11_IRQn);
  playing = 0U;
  tx_kind = TX_KIND_AFSK;
  level_write(0);
  data_pin(GPIO_MODE_INPUT); /* Release before the radio changes DIO direction. */
  HAL_NVIC_ClearPendingIRQ(TIM2_IRQn);
  if (checked_write(REG_MODE, MODE_STANDBY) == 0 && wait_flag(0x80U) == 0) return 0;
  /* Failed SPI must not leave a continuous carrier on air. Hardware reset
     returns the radio to standby and invalidates its configuration. */
  HAL_GPIO_WritePin(RSY_GPIO_Port, RSY_Pin, GPIO_PIN_RESET);
  HAL_Delay(1U);
  HAL_GPIO_WritePin(RSY_GPIO_Port, RSY_Pin, GPIO_PIN_SET);
  HAL_Delay(5U);
  ready = 0U;
  return -1;
}

static int rx_stop_internal(uint8_t clear_request)
{
  if (clear_request != 0U) rx_wanted = 0U;
  rx_active = 0U;
  HAL_NVIC_DisableIRQ(EXTI11_IRQn);
  __HAL_GPIO_EXTI_CLEAR_IT(DIO1_Pin);
  data_pin(GPIO_MODE_INPUT);
  if (ready == 0U) return 0;
  return checked_write(REG_MODE, MODE_STANDBY) || wait_flag(0x80U) ? -1 : 0;
}

static int rx_start_internal(void)
{
  uint32_t frf;
  uint8_t ook_peak;
  if (ready == 0U || playing != 0U) return -1;
  ++rx_attempts;
  rx_start_result = -2;
  rx_start_stage = "standby";
  if (rx_stop_internal(0U) != 0) goto failed;
  rx_start_result = -3;
  rx_start_stage = "config";
  /* Continuous RX has no DCLK when BitSyncOn is clear (datasheet 4.2.12.3).
     Preserve OOK/reserved fields and do not depend on its reset value. */
  if (reg_read(0x14U, &ook_peak) ||
      checked_write(0x14U, (uint8_t)(ook_peak | 0x20U))) goto failed;
  frf = (uint32_t)((((uint64_t)UPLINK_FREQUENCY_HZ << 19U) + 16000000ULL) /
                   32000000ULL);
  if (checked_write(REG_MODE, MODE_STANDBY) || wait_flag(0x80U) ||
      checked_write(0x06U, (uint8_t)(frf >> 16U)) ||
      checked_write(0x07U, (uint8_t)(frf >> 8U)) ||
      checked_write(0x08U, (uint8_t)frf) ||
      checked_write(0x02U, (uint8_t)(UPLINK_BITRATE >> 8U)) ||
      checked_write(0x03U, (uint8_t)UPLINK_BITRATE) ||
      checked_write(0x04U, (uint8_t)(UPLINK_FDEV >> 8U)) ||
      checked_write(0x05U, (uint8_t)UPLINK_FDEV) ||
      checked_write(0x0AU, RX_PA_RAMP_GMSK_BT05) ||
      /* RadioLib SX127x::directMode() uses RxTrigger=NONE for continuous RX.
         Keep AGC on and AFC off, as in its default FSK/direct profile: waiting
         for a preamble here conflicts with foreground startup validation. */
      checked_write(0x0DU, RX_CONFIG_CONTINUOUS) ||
      checked_write(0x12U, 0x05U) || /* approximately 15.6 kHz channel BW */
      checked_write(0x13U, 0x05U) ||
      checked_write(0x27U, 0x00U) || /* custom ASM is decoded in software */
      checked_write(0x30U, 0x00U) ||
      checked_write(0x31U, 0x00U) || /* continuous mode, no whitening/CRC */
      checked_write(0x40U, 0x00U) || /* DIO1=DCLK, DIO2=Data */
      checked_write(0x41U, 0x00U)) goto failed;
  if (rx_irq_probe_result == 0U) {
    /* Software EXTI exercises NVIC -> vector -> HAL -> callback in standby.
       It does not drive a pin or feed an invented bit into the decoder. */
    rx_start_result = -6;
    rx_start_stage = "exti-selftest";
    clock_pin(GPIO_MODE_IT_RISING);
    HAL_NVIC_SetPriority(EXTI11_IRQn, 3U, 0U);
    __HAL_GPIO_EXTI_CLEAR_IT(DIO1_Pin);
    HAL_NVIC_ClearPendingIRQ(EXTI11_IRQn);
    rx_irq_probe_seen = 0U;
    rx_irq_probe = 1U;
    HAL_NVIC_EnableIRQ(EXTI11_IRQn);
    __HAL_GPIO_EXTI_GENERATE_SWIT(DIO1_Pin);
    HAL_Delay(1U);
    HAL_NVIC_DisableIRQ(EXTI11_IRQn);
    __HAL_GPIO_EXTI_CLEAR_IT(DIO1_Pin);
    HAL_NVIC_ClearPendingIRQ(EXTI11_IRQn);
    rx_irq_probe = 0U;
    rx_irq_probe_result = rx_irq_probe_seen != 0U ? 1U : 2U;
  }
  if (rx_irq_probe_result != 1U) {
    rx_start_result = -6;
    rx_start_stage = "exti-selftest";
    goto failed;
  }
  /* Semtech's FSK SetRx enters receiver directly from standby. The radio
     sequences its synthesizer internally; FSRX is not a receive-ready state. */
  rx_start_result = -4;
  rx_start_stage = "mode";
  if (checked_write(REG_MODE, MODE_RX)) goto failed;
  rx_start_result = -5;
  rx_start_stage = "rx-ready";
  if (wait_flag(0xC0U)) goto failed; /* ModeReady + RxReady, not just PllLock. */
  uplink_decoder_init(&rx_decoder);
  rx_last_bit_tick = HAL_GetTick();
  data_pin(GPIO_MODE_INPUT);
  clock_pin(GPIO_MODE_IT_RISING);
  HAL_NVIC_SetPriority(EXTI11_IRQn, 3U, 0U);
  __HAL_GPIO_EXTI_CLEAR_IT(DIO1_Pin);
  HAL_NVIC_ClearPendingIRQ(EXTI11_IRQn);
  /* Mark the decoder live before unmasking DIO1. A clock edge can arrive
     immediately after the pending flag is cleared. */
  rx_active = 1U;
  HAL_NVIC_EnableIRQ(EXTI11_IRQn);
  rx_start_result = 0;
  rx_start_stage = "ready";
  rx_start_mode = MODE_RX;
  (void)reg_read(REG_IRQ1, &rx_start_irq1);
  (void)reg_read(0x0DU, &rx_start_config);
  rx_attempt_tick = HAL_GetTick();
  rx_start_notice = 1U;
  return 0;
failed:
  /* Capture the failed transition before cleanup changes the registers. */
  rx_start_mode = rx_start_irq1 = rx_start_config = 0xFFU;
  (void)reg_read(REG_MODE, &rx_start_mode);
  (void)reg_read(REG_IRQ1, &rx_start_irq1);
  (void)reg_read(0x0DU, &rx_start_config);
  rx_active = 0U;
  (void)stop_radio();
  rx_attempt_tick = HAL_GetTick();
  rx_start_notice = 1U;
  return rx_start_result;
}

int SX1278_APRS_RxStart(void)
{
  rx_wanted = 1U;
  return rx_start_internal();
}

void SX1278_APRS_RxStop(void)
{
  (void)rx_stop_internal(1U);
}

uint8_t SX1278_APRS_RxActive(void) { return rx_active; }

int SX1278_APRS_RxPoll(UplinkCommand *command)
{
  int result;
  if (command == NULL) return 0;
  if (rx_active == 0U) {
    if (rx_wanted != 0U && ready != 0U && playing == 0U &&
        HAL_GetTick() - rx_attempt_tick >= 1000U)
      (void)rx_start_internal();
    return 0;
  }
  if (HAL_GetTick() - rx_rssi_tick >= 20U) {
    uint8_t rssi;
    rx_rssi_tick = HAL_GetTick();
    if (reg_read(0x11U, &rssi) == 0) {
      if (rssi < rx_rssi_peak) rx_rssi_peak = rssi;
      ++rx_rssi_samples;
    } else ++rx_rssi_errors;
  }
  HAL_NVIC_DisableIRQ(EXTI11_IRQn);
  if (rx_decoder.collecting != 0U &&
      rx_decoder.ready == 0U &&
      HAL_GetTick() - rx_last_bit_tick > 100U) {
    ++rx_stats.gaps;
    uplink_decoder_init(&rx_decoder);
    rx_last_bit_tick = HAL_GetTick();
    HAL_NVIC_EnableIRQ(EXTI11_IRQn);
    return 0;
  }
  uint8_t complete = rx_decoder.ready;
  result = uplink_decoder_take(&rx_decoder, command);
  if (result != 0) ++rx_stats.valid;
  else if (complete != 0U) ++rx_stats.rejected;
  HAL_NVIC_EnableIRQ(EXTI11_IRQn);
  return result;
}

void SX1278_APRS_RxReport(char *out, size_t capacity)
{
  uint32_t now = HAL_GetTick();
  uint32_t events = rx_stats.syncs + rx_stats.header_bad + rx_stats.valid +
                    rx_stats.rejected + rx_stats.gaps + rx_stats.overflows;
  uint8_t mode = 0U, irq1 = 0U, rssi = 0U;
  uint8_t peak = 0U, mapping = 0U, packet = 0U;
  int spi_result;
  if (out == NULL || capacity == 0U) return;
  out[0] = 0;
  if (rx_start_notice != 0U) {
    rx_start_notice = 0U;
    (void)snprintf(out, capacity,
      "UPLINK RX start rc=%d stage=%s mode=%02X irq1=%02X rxcfg=%02X attempt=%lu wanted=%u irqtest=%s\r\n",
      rx_start_result, rx_start_stage, rx_start_mode, rx_start_irq1,
      rx_start_config, (unsigned long)rx_attempts, rx_wanted,
      rx_irq_probe_result == 1U ? "PASS" : rx_irq_probe_result == 2U ? "FAIL" : "NONE");
    return;
  }
  if (rx_irq_reported == 0U && rx_stats.clocks != 0U) {
    rx_irq_reported = 1U;
    (void)snprintf(out, capacity,
      "UPLINK RX IRQ first: DIO1 clock detected; waiting for preamble/ASM/CRC\r\n");
    return;
  }
  if (now - rx_report_tick < 1000U ||
      (now - rx_report_tick < 5000U && events == rx_report_events)) return;
  rx_report_tick = now;
  rx_report_events = events;
  /* Reads and formatting stay outside EXTI. Counters are cumulative since boot;
     individual 32-bit reads are atomic, but this is not a frozen RF snapshot. */
  spi_result = reg_read(REG_MODE, &mode);
  spi_result |= reg_read(REG_IRQ1, &irq1);
  spi_result |= reg_read(0x11U, &rssi);
  spi_result |= reg_read(0x14U, &peak);
  spi_result |= reg_read(0x40U, &mapping);
  spi_result |= reg_read(0x31U, &packet);
  (void)snprintf(out, capacity,
    "UPLINK RX active=%u mode=%02X irq1=%02X rssi=-%u.%u spi=%d "
    "clocks=%lu changes=%lu pre=%lu asm=%lu header_bad=%lu ok=%lu "
    "bch_crc_bad=%lu gap=%lu overflow=%lu raw=%u blocks=%u\r\n"
    "RX DIO peak=%02X map=%02X pkt=%02X pins=%u/%u gpio=%lu/%lu "
    "exti_port=%lu imr=%u rise=%u pend=%u nvic=%lu primask=%lu\r\n"
    "RX RSSI peak=-%u.%u samples=%lu errors=%lu\r\n",
    rx_active, mode, irq1, rssi / 2U, (rssi & 1U) * 5U, spi_result,
    (unsigned long)rx_stats.clocks, (unsigned long)rx_stats.changes,
    (unsigned long)rx_stats.preambles, (unsigned long)rx_stats.syncs,
    (unsigned long)rx_stats.header_bad, (unsigned long)rx_stats.valid,
    (unsigned long)rx_stats.rejected, (unsigned long)rx_stats.gaps,
    (unsigned long)rx_stats.overflows, rx_decoder.raw_count,
    rx_decoder.expected_blocks, peak, mapping, packet,
    (unsigned)HAL_GPIO_ReadPin(DIO1_GPIO_Port, DIO1_Pin),
    (unsigned)HAL_GPIO_ReadPin(DIO2_GPIO_Port, DIO2_Pin),
    (unsigned long)((DIO1_GPIO_Port->MODER >> 22U) & 3U),
    (unsigned long)((DIO2_GPIO_Port->MODER >> 24U) & 3U),
    (unsigned long)((EXTI->EXTICR[2] >> 24U) & 15U),
    (unsigned)((EXTI->IMR1 & DIO1_Pin) != 0U),
    (unsigned)((EXTI->RTSR1 & DIO1_Pin) != 0U),
    (unsigned)((EXTI->RPR1 & DIO1_Pin) != 0U),
    (unsigned long)NVIC_GetEnableIRQ(EXTI11_IRQn),
    (unsigned long)__get_PRIMASK(), rx_rssi_peak / 2U, (rx_rssi_peak & 1U) * 5U,
    (unsigned long)rx_rssi_samples, (unsigned long)rx_rssi_errors);
  rx_rssi_peak = 255U;
  rx_rssi_samples = rx_rssi_errors = 0U;
}

uint8_t SX1278_APRS_Ready(void) { return ready; }

static int capture_tx(void)
{
  tx_state.valid = 0U;
  if (reg_read(REG_MODE, &tx_state.mode) || reg_read(REG_IRQ1, &tx_state.irq1) ||
      reg_read(0x3FU, &tx_state.irq2) || reg_read(REG_BITRATE_MSB, &tx_state.datamod) ||
      reg_read(0x09U, &tx_state.pa) ||
      reg_read(0x0AU, &tx_state.ramp) || reg_read(0x0BU, &tx_state.ocp) ||
      reg_read(0x31U, &tx_state.packet) || reg_read(0x4BU, &tx_state.tcxo) ||
      reg_read(0x4DU, &tx_state.dac)) return -1;
  for (unsigned i = 0; i < 3; ++i)
    if (reg_read((uint8_t)(0x06U + i), &tx_state.frf[i])) return -1;
  for (unsigned i = 0; i < 2; ++i)
    if (reg_read((uint8_t)(0x04U + i), &tx_state.fdev[i])) return -1;
  tx_state.valid = 1U;
  return 0;
}

static int tx_still_active(void)
{
  if (reg_read(REG_MODE, &tx_state.mode) || reg_read(REG_IRQ1, &tx_state.irq1)) return -1;
  uint8_t required = tx_state.direct_fallback ? 0xA0U : 0xB0U;
  return tx_state.mode == MODE_TX && (tx_state.irq1 & required) == required ? 0 : -1;
}

/* An explicit FSTX stage exposes PLL failure separately from PA readiness.
   The hardware also supports direct standby->TX; omitting FSTX alone is not
   evidence for the missing carrier. No frequency changes are made in TX. */
static int begin_tx(uint32_t frequency_hz, uint8_t ramp)
{
  uint32_t frf = (uint32_t)((((uint64_t)frequency_hz << 19) + 16000000ULL) / 32000000ULL);
  if (checked_write(REG_MODE, MODE_STANDBY) || wait_flag(0x80U) ||
      checked_write(0x06U, (uint8_t)(frf >> 16)) ||
      checked_write(0x07U, (uint8_t)(frf >> 8)) ||
      checked_write(0x08U, (uint8_t)frf) ||
      checked_write(0x0AU, ramp)) return -3;
  if (checked_write(REG_MODE, MODE_FSTX) || wait_flag(0x10U)) return -6;
  if (checked_write(REG_MODE, MODE_TX) || wait_flag(0xB0U)) return -7;
  if (capture_tx() || tx_still_active()) return -8;
  return 0;
}

void SX1278_APRS_TxReport(char *out, size_t capacity)
{
  if (!out || !capacity) return;
  (void)snprintf(out, capacity,
    "SX TX snapshot valid=%u rc=%d direct=%u mode=%02X irq1=%02X irq2=%02X bitrate_msb=%02X frf=%02X%02X%02X pa=%02X ramp=%02X ocp=%02X pkt=%02X tcxo=%02X dac=%02X dev=%02X%02X samples=%lu edges=%lu\r\n",
    tx_state.valid, tx_state.result, tx_state.direct_fallback, tx_state.mode, tx_state.irq1, tx_state.irq2,
    tx_state.datamod, tx_state.frf[0], tx_state.frf[1], tx_state.frf[2], tx_state.pa, tx_state.ramp,
    tx_state.ocp, tx_state.packet, tx_state.tcxo, tx_state.dac,
    tx_state.fdev[0], tx_state.fdev[1], (unsigned long)emitted_samples, (unsigned long)data_edges);
}

int SX1278_APRS_CarrierTest(uint32_t frequency_hz, uint32_t duration_ms, void (*service)(void))
{
  if (!ready || playing || !duration_ms || duration_ms > 5000U ||
      frequency_hz < 137000000UL || frequency_hz > 175000000UL) return -2;
  memset(&tx_state, 0, sizeof(tx_state));
  first_failure.stage = 0U;
  emitted_samples = data_edges = 0U;
  int result = 0;
  /* Fdev=0 removes all dependence on DATA, AFSK and timer activity. */
  if (checked_write(REG_MODE, MODE_STANDBY) || wait_flag(0x80U) ||
      checked_write(0x04U, 0U) || checked_write(0x05U, 0U)) result = -3;
  if (!result) result = begin_tx(frequency_hz, 0x09U);
  /* Diagnostic-only fallback: if FSTX has no PLL lock, request TX directly.
     Normal APRS never uses this path. It separates a missing PLL indication
     from a dead PA/RF path. */
  if (result == -6) {
    first_failure.stage = 0U;
    tx_state.direct_fallback = 1U; /* Records the attempt, including failures. */
    if (checked_write(REG_MODE, MODE_TX) || wait_flag(0xA0U)) result = -7;
    else if (capture_tx()) result = -8;
    else result = 1; /* TX ready was observed; RF output is still unverified. */
  }
  if (result < 0) (void)capture_tx();
  else {
    level_write(0);
    data_pin(GPIO_MODE_OUTPUT_PP);
    uint32_t start = HAL_GetTick();
    while (HAL_GetTick() - start < duration_ms) {
      if (tx_still_active()) { result = -8; break; }
      if (service) service();
      HAL_Delay(10U);
    }
  }
  if (stop_radio()) result = -5;
  if (ready && (checked_write(0x04U, 0U) || checked_write(0x05U, 49U))) {
    ready = 0U;
    result = -9;
  }
  tx_state.result = result;
  return result;
}

int SX1278_APRS_SetPowerDbm(uint8_t dbm)
{
  uint8_t pa_dac;
  uint8_t output_power;

  if (!ready || dbm < 2U || dbm > 20U || playing) return -1;
  if (dbm == 20U) {
    pa_dac = 0x87U;
    output_power = 15U;
  } else {
    pa_dac = 0x84U;
    output_power = (uint8_t)(dbm - 2U);
  }
  if (checked_write(0x4DU, pa_dac) ||
      checked_write(0x09U, (uint8_t)(0x80U | output_power))) return -2;
  return 0;
}

void SX1278_APRS_ConfigError(char *out, size_t capacity)
{
  if (out == NULL || capacity == 0U) return;
  if (first_failure.stage == 0U) { out[0] = 0; return; }
  const char *stage = first_failure.stage == 1U ? "write" :
                      first_failure.stage == 2U ? "read" :
                      first_failure.stage == 4U ? "mode-timeout" : "verify";
  (void)snprintf(out, capacity,
    "SX1278 config fail stage=%s reg=0x%02X want=0x%02X got=0x%02X spi_err=0x%08lX\r\n",
    stage, first_failure.reg, first_failure.expected, first_failure.actual,
    (unsigned long)first_failure.spi_error);
}

void SX1278_APRS_TestNss(void (*log)(const char *))
{
  if (playing || ready || log == NULL) return;
  char line[100];
  /* Keep the shared Flash deselected. Do not issue SPI clocks or change pin
     configuration: report the original GPIO state rather than hide a fault. */
  HAL_GPIO_WritePin(FLASH_NSS_GPIO_Port, FLASH_NSS_Pin, GPIO_PIN_SET);
  log("NSS test PB12 -> U18 pin19: LOW 2s, HIGH 2s; no SPI clocks\r\n");
  for (unsigned level = 0; level < 2; ++level) {
    HAL_GPIO_WritePin(SX1278_NSS_GPIO_Port, SX1278_NSS_Pin,
                     level ? GPIO_PIN_SET : GPIO_PIN_RESET);
    HAL_Delay(2U);
    (void)snprintf(line, sizeof(line), "NSS hold %s ODR=%u IDR=%u MODE=%lu OT=%lu\r\n",
      level ? "HIGH" : "LOW",
      (unsigned)((SX1278_NSS_GPIO_Port->ODR & SX1278_NSS_Pin) != 0),
      (unsigned)HAL_GPIO_ReadPin(SX1278_NSS_GPIO_Port, SX1278_NSS_Pin),
      (unsigned long)((SX1278_NSS_GPIO_Port->MODER >> 24U) & 3U),
      (unsigned long)((SX1278_NSS_GPIO_Port->OTYPER >> 12U) & 1U));
    log(line);
    HAL_Delay(2000U);
  }
  HAL_GPIO_WritePin(SX1278_NSS_GPIO_Port, SX1278_NSS_Pin, GPIO_PIN_SET);
  log("NSS test done; NSS released HIGH\r\n");
}

void SX1278_APRS_Diagnose(char *out, size_t capacity)
{
  if (out == NULL || !capacity || playing) return;
  uint8_t mode = 0xFF;
  int mode_rc = reg_read(REG_MODE, &mode);
  uint8_t tx[4] = {0x9F, 0, 0, 0}, rx[4] = {0};
  HAL_GPIO_WritePin(SX1278_NSS_GPIO_Port, SX1278_NSS_Pin, GPIO_PIN_SET);
  HAL_GPIO_WritePin(FLASH_NSS_GPIO_Port, FLASH_NSS_Pin, GPIO_PIN_RESET);
  int flash_rc = (int)HAL_SPI_TransmitReceive(&hspi2, tx, rx, 4U, 10U);
  HAL_GPIO_WritePin(FLASH_NSS_GPIO_Port, FLASH_NSS_Pin, GPIO_PIN_SET);
  (void)snprintf(out, capacity,
    "SPI2 diag ver=%02X/%02X/%02X mode=%02X rc=%d RST=%u NSS=%u MISO=%u flash=%02X%02X%02X rc=%d err=%08lX\r\n",
    version_reads[0], version_reads[1], version_reads[2], mode, mode_rc,
    (unsigned)HAL_GPIO_ReadPin(RSY_GPIO_Port, RSY_Pin),
    (unsigned)HAL_GPIO_ReadPin(SX1278_NSS_GPIO_Port, SX1278_NSS_Pin),
    (unsigned)HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_2),
    rx[1], rx[2], rx[3], flash_rc, (unsigned long)HAL_SPI_GetError(&hspi2));
}

int SX1278_APRS_Init(uint8_t *version)
{
  uint8_t v = 0U;
  ready = 0U;
  rx_irq_probe_result = 0U;
  first_failure.stage = 0U;
  for (unsigned i = 0; i < 3U; ++i) version_reads[i] = 0xFFU;
  if (version == NULL) return -1;
  *version = 0U;
  HAL_GPIO_WritePin(FLASH_NSS_GPIO_Port, FLASH_NSS_Pin, GPIO_PIN_SET);
  HAL_GPIO_WritePin(SX1278_NSS_GPIO_Port, SX1278_NSS_Pin, GPIO_PIN_SET);
  HAL_GPIO_WritePin(RSY_GPIO_Port, RSY_Pin, GPIO_PIN_RESET);
  HAL_Delay(1U);
  HAL_GPIO_WritePin(RSY_GPIO_Port, RSY_Pin, GPIO_PIN_SET);
  HAL_Delay(10U);
  uint8_t mismatch = 0U;
  for (unsigned i = 0; i < 3; ++i) {
    version_reads[i] = 0xFF;
    if (reg_read(0x42U, &v)) return -1;
    version_reads[i] = v;
    *version = v;
    if (v != 0x12U) mismatch = 1U;
    HAL_Delay(2U);
  }
  if (mismatch) return -2;

  /* Stop DIO IRQs: in continuous TX they would fire on clock/audio edges. */
  HAL_NVIC_DisableIRQ(EXTI11_IRQn);
  HAL_NVIC_DisableIRQ(EXTI12_IRQn);
  HAL_NVIC_DisableIRQ(EXTI13_IRQn);
  data_pin(GPIO_MODE_INPUT);
  level_write(0);
  __HAL_RCC_TIM2_CLK_ENABLE();
  TIM2->CR1 = 0U;
  TIM2->DIER = 0U;
  HAL_NVIC_SetPriority(TIM2_IRQn, 2U, 0U);
  HAL_NVIC_ClearPendingIRQ(TIM2_IRQn);
  HAL_NVIC_EnableIRQ(TIM2_IRQn);

  /* PA_BOOST power = 2 + OutputPower, normal PaDac, 100 mA OCP.
     49 * (32 MHz / 2^19) = 2990.72 Hz peak FM deviation. */
  const uint8_t settings[][2] = {
    {REG_MODE, MODE_SLEEP}, {0x4BU, 0x19U},
    {0x09U, (uint8_t)(0x80U | (APP_APRS_POWER_DBM - 2U))},
    {0x4DU, 0x84U}, {0x0BU, 0x2BU}, {0x0AU, 0x09U},
    /* Continuous FSK with asynchronous data on DIO2.  RegDataModul[1:0]
       must be 10b; packet mode (including 0x80) does not use DIO2 input. */
    {REG_BITRATE_MSB, 0x02U}, {0x03U, 0x0BU},
    {0x04U, 0x00U}, {0x05U, 49U},
    {0x30U, 0x00U}, {0x31U, 0x00U}, /* no HW CRC/whitening; continuous */
    {0x40U, 0x00U}, {0x41U, 0x00U}, {REG_MODE, MODE_STANDBY}
  };
  for (size_t i = 0; i < sizeof(settings) / sizeof(settings[0]); ++i) {
    if (checked_write(settings[i][0], settings[i][1])) {
      (void)stop_radio();
      return -3;
    }
  }
  if (wait_flag(0x80U)) { (void)stop_radio(); return -4; }
  ready = 1U;
  return 0;
}

int SX1278_APRS_Prepare(const char *info, size_t length)
{
  return SX1278_APRS_PrepareAddressed(APP_APRS_CALL, APP_APRS_PATH,
                                      (const uint8_t *)info, length);
}

int SX1278_APRS_PrepareAddressed(const char *source, const char *destination,
                                 const uint8_t *info, size_t length)
{
  if (playing || !ready) return -1;
  stream.count = 0U;
  int bytes = AX25_Build(source, destination, info, length, frame, sizeof(frame));
  if (bytes < 0 || AFSK_Prepare(&stream, frame, (size_t)bytes)) return -2;
  return (int)AFSK_DurationMs(&stream);
}

int SX1278_APRS_PrepareAddressedPath(const char *source, const char *destination,
                                     const char *path, const uint8_t *info,
                                     size_t length)
{
  if (playing || !ready) return -1;
  stream.count = 0U;
  int bytes = AX25_BuildPath(source, destination, path, info, length,
                             frame, sizeof(frame));
  if (bytes < 0 || AFSK_Prepare(&stream, frame, (size_t)bytes)) return -2;
  return (int)AFSK_DurationMs(&stream);
}

int SX1278_APRS_SendWithContext(uint32_t frequency_hz, uint32_t budget_ms,
                                SX1278_APRS_ServiceFn service, void *context)
{
  uint8_t resume_rx;
  if (!ready || playing || stream.count == 0U) return -1;
  if (frequency_hz < 137000000UL || frequency_hz > 175000000UL ||
      budget_ms < AFSK_DurationMs(&stream) + 200U) return -2;
  resume_rx = rx_wanted;
  if (rx_active != 0U && rx_stop_internal(0U) != 0) return -3;
  /* RX uses 4800 bps and 1.2 kHz deviation. Restore the asynchronous APRS
     settings before every VHF packet, otherwise the first packet after an
     uplink would be sent with the receive modem parameters. */
  if (checked_write(0x02U, 0x02U) || checked_write(0x03U, 0x0BU) ||
      checked_write(0x04U, 0x00U) || checked_write(0x05U, 49U) ||
      checked_write(0x0AU, 0x09U)) {
    if (resume_rx != 0U) (void)rx_start_internal();
    return -3;
  }
  memset(&tx_state, 0, sizeof(tx_state));
  first_failure.stage = 0U;
  emitted_samples = data_edges = 0U;
  uint32_t start = HAL_GetTick();
  int result = begin_tx(frequency_hz, 0x09U);
  if (result) {
    (void)capture_tx();
    if (stop_radio()) result = -5;
    tx_state.result = result;
    if (resume_rx != 0U) (void)rx_start_internal();
    return result;
  }
  /* Derive TIM2 clock from the actual APB1 prescaler (TIMPRE is reset/default). */
  uint32_t timer_hz = HAL_RCC_GetPCLK1Freq();
  if ((RCC->CFGR2 & RCC_CFGR2_PPRE1) != 0U) timer_hz *= 2U;
  timer_base = timer_hz / AFSK_SAMPLE_HZ;
  timer_remainder = timer_hz % AFSK_SAMPLE_HZ;
  timer_fraction = 0U;
  stream.index = 0U;
  stream.sample = stream.phase = 0U;
  TIM2->CR1 = 0U; /* no ARR preload: fractional periods apply immediately */
  TIM2->PSC = 0U;
  TIM2->ARR = next_period();
  TIM2->EGR = TIM_EGR_UG;
  TIM2->SR = 0U;
  TIM2->CNT = 0U;
  /* Only drive DATA after continuous TX has made it a radio input. */
  previous_level = AFSK_NextSample(&stream);
  level_write(previous_level);
  emitted_samples = 1U;
  data_pin(GPIO_MODE_OUTPUT_PP);
  __DMB();
  playing = 1U;
  TIM2->DIER = TIM_DIER_UIE;
  TIM2->CR1 = TIM_CR1_CEN;
  uint32_t last_check = HAL_GetTick();
  while (playing) {
    if (HAL_GetTick() - start >= budget_ms) { result = -4; break; }
    if (HAL_GetTick() - last_check >= 100U) {
      if (tx_still_active()) { result = -8; break; }
      last_check = HAL_GetTick();
    }
    if (service != NULL) service(context);
  }
  if (!result && tx_still_active()) result = -8;
  if (stop_radio()) result = -5;
  if (resume_rx != 0U) (void)rx_start_internal();
  tx_state.result = result;
  return result;
}

int SX1278_APRS_PrepareGmsk(const uint8_t *bits, size_t bit_count,
                            uint32_t bitrate, uint32_t deviation_hz)
{
  /* This milestone implements the conventional profile only. Higher rates
     require a separately validated transport, not this 4800-baud ISR. */
  if (playing) return -1;
  gmsk_bit_count = 0U;
  if (!ready || bits == NULL || bit_count == 0U ||
      bit_count > GMSK_MAX_PACKED_BYTES * 8U || bitrate != 4800U ||
      deviation_hz != 1200U) return -1;
  memcpy(gmsk_bits, bits, (bit_count + 7U) / 8U);
  gmsk_bit_count = bit_count;
  gmsk_bitrate = bitrate;
  gmsk_deviation_hz = deviation_hz;
  return (int)((bit_count * 1000U + bitrate - 1U) / bitrate);
}

int SX1278_APRS_SendGmsk(uint32_t frequency_hz, uint32_t budget_ms,
                         SX1278_APRS_ServiceFn service, void *context)
{
  uint8_t resume_rx;
  uint32_t start, last_check, bitrate_reg, fdev_reg;
  int result = 0;
  if (!ready || playing || gmsk_bit_count == 0U || gmsk_bitrate == 0U)
    return -1;
  if (frequency_hz < 137000000UL || frequency_hz > 175000000UL ||
      budget_ms < (gmsk_bit_count * 1000U + gmsk_bitrate - 1U) /
                    gmsk_bitrate + 200U) return -2;
  resume_rx = rx_wanted;
  memset(&tx_state, 0, sizeof(tx_state));
  first_failure.stage = 0U;
  emitted_samples = data_edges = 0U;
  start = HAL_GetTick();
  if (rx_stop_internal(0U) != 0) { result = -3; goto cleanup; }
  bitrate_reg = (32000000UL + gmsk_bitrate / 2U) / gmsk_bitrate;
  fdev_reg = (uint32_t)((((uint64_t)gmsk_deviation_hz << 19U) +
                         16000000ULL) / 32000000ULL);
  /* RegPaRamp[6:5]=10 selects Gaussian BT=0.5 (not PacketConfig1).
     No chip-generated CRC, whitening, NRZI, preamble or sync bytes. */
  if (checked_write(0x02U, (uint8_t)(bitrate_reg >> 8U)) ||
      checked_write(0x03U, (uint8_t)bitrate_reg) ||
      checked_write(0x04U, (uint8_t)(fdev_reg >> 8U)) ||
      checked_write(0x05U, (uint8_t)fdev_reg) ||
      checked_write(0x27U, 0x00U) || checked_write(0x30U, 0x00U) ||
      checked_write(0x31U, 0x00U) || checked_write(0x40U, 0x00U) ||
      checked_write(0x41U, 0x00U) ||
      SX1278_APRS_SetPowerDbm(APP_GMSK_POWER_DBM)) {
    result = -3;
    goto cleanup;
  }
  result = begin_tx(frequency_hz, 0x49U);
  if (result != 0) goto cleanup;
  clock_pin(GPIO_MODE_IT_FALLING);
  level_write(0);
  data_pin(GPIO_MODE_OUTPUT_PP);
  previous_level = 0;
  gmsk_bit_index = 0U;
  gmsk_flush_clocks = 0U;
  tx_kind = TX_KIND_GMSK;
  __HAL_GPIO_EXTI_CLEAR_IT(DIO1_Pin);
  HAL_NVIC_ClearPendingIRQ(EXTI11_IRQn);
  HAL_NVIC_SetPriority(EXTI11_IRQn, 2U, 0U);
  __DMB();
  playing = 1U;
  HAL_NVIC_EnableIRQ(EXTI11_IRQn);
  last_check = HAL_GetTick();
  while (playing) {
    if (HAL_GetTick() - start >= budget_ms) { result = -4; break; }
    if (HAL_GetTick() - last_check >= 100U) {
      if (tx_still_active() != 0) { result = -8; break; }
      last_check = HAL_GetTick();
    }
    if (service != NULL) service(context);
  }
  if (result == 0 && tx_still_active() != 0) result = -8;
cleanup:
  if (result != 0) (void)capture_tx();
  if (stop_radio()) result = -5;
  gmsk_bit_count = 0U;
  if (ready && SX1278_APRS_SetPowerDbm(APP_APRS_POWER_DBM)) result = -9;
  /* Preserve TX diagnostics even though the RX profile replaces registers. */
  if (resume_rx != 0U && rx_start_internal() != 0) result = -10;
  tx_state.result = result;
  return result;
}

int SX1278_APRS_Send(uint32_t frequency_hz, uint32_t budget_ms,
                     void (*service)(void))
{
  int result;

  legacy_service = service;
  result = SX1278_APRS_SendWithContext(
      frequency_hz, budget_ms,
      service == NULL ? NULL : legacy_service_adapter, NULL);
  legacy_service = NULL;
  return result;
}
