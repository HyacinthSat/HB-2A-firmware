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
/* High-speed IntlvID=8, CodeID=1 needs just over 4 KiB of packed bits. */
#define GMSK_MAX_PACKED_BYTES 8192U
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
#if APP_UPLINK_RX_FIFO
static volatile uint32_t rx_fifo_irqs;
static uint32_t rx_fifo_bytes, rx_fifo_spi_errors, rx_packet_tick;
static uint8_t rx_preamble_seen, rx_frequency_valid;
static int16_t rx_afc, rx_fei;
static uint32_t rx_preamble_tick, rx_sync_misses;
#endif
static uint32_t timer_base, timer_remainder, timer_fraction;
static uint8_t version_reads[3];
static uint8_t gmsk_bits[GMSK_MAX_PACKED_BYTES];
static size_t gmsk_bit_count;
static size_t gmsk_tx_byte_count;
static size_t gmsk_tx_byte_index;
static uint32_t gmsk_bitrate;
static uint32_t gmsk_deviation_hz;
static uint8_t tx_kind;
static struct {
  uint8_t mode, irq1, irq2, datamod, frf[3], pa, ramp, ocp, packet, tcxo, dac, fdev[2];
  uint8_t dio1_level, dio2_level, dio1_moder, dio2_moder;
  uint8_t exti_imr, exti_rise, exti_fall, exti_rpend, exti_fpend, nvic_enabled;
  uint8_t fifo_tx, pad_bits, packet_sent, drained;
  uint32_t fifo_bytes, fifo_total;
  uint8_t valid;
  uint8_t direct_fallback;
  int result;
} tx_state;
static volatile uint32_t emitted_samples, data_edges;
static int previous_level;
static void (*legacy_service)(void);
static void level_write(int level);

/* Fixed-length packets support up to 2047 bytes. Larger groups use unlimited
   mode and explicitly drain the final byte and Gaussian filter before stop. */
#define GMSK_FIFO_THRESHOLD 32U
#define IRQ2_FIFO_EMPTY 0x40U
#define IRQ2_FIFO_LEVEL 0x20U
#define IRQ2_PACKET_SENT 0x08U

static int reg_write(uint8_t reg, uint8_t value);

static int gmsk_fifo_write(size_t count)
{
  while (count != 0U && gmsk_tx_byte_index < gmsk_tx_byte_count) {
    if (reg_write(0x00U, gmsk_bits[gmsk_tx_byte_index])) return -1;
    ++gmsk_tx_byte_index;
    tx_state.fifo_bytes = (uint32_t)gmsk_tx_byte_index;
    --count;
  }
  return 0;
}

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
#if APP_UPLINK_RX_FIFO
  /* DIO1 is FifoLevel in packet mode. SPI and decoding stay in foreground. */
  ++rx_fifo_irqs;
  return;
#endif
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
  (void)pin;
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
  rx_attempt_tick = HAL_GetTick();
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
#if APP_UPLINK_RX_FIFO
  /* Datasheet 4.2.13.2: fixed format + zero length is unlimited RX. No
     length/address byte is added to the wire format. Software derives the
     BCH block count from the command and rearms after its trailing CRC. */
  if (checked_write(0x0DU, 0x1EU) || /* AGC + AFC on preamble, datasheet 4.2.6 */
      checked_write(0x13U, 0x0BU) || /* 50 kHz AFC capture BW; RX stays 15.625 kHz */
      checked_write(0x1AU, 0x01U) || /* clear the old AFC correction at acquisition */
      checked_write(0x1FU, 0xAAU) || /* two-byte preamble detector */
      checked_write(0x27U, 0x33U) ||
      checked_write(0x28U, 0x1AU) || checked_write(0x29U, 0xCFU) ||
      checked_write(0x2AU, 0xFCU) || checked_write(0x2BU, 0x1DU) ||
      checked_write(0x31U, 0x40U) || checked_write(0x32U, 0U) ||
      checked_write(0x35U, 0x87U) || /* FIFO level at eight payload bytes */
      reg_write(0x3FU, 0x10U)) goto failed; /* flush FIFO */
#endif
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
#if APP_UPLINK_RX_FIFO
  /* Triggered acquisition need not assert RxReady before the first preamble.
     Request RX once, allow its intermediate FSRX state, and check PLL lock.
     Do not write FSRX explicitly or abandon acquisition after 20 ms silence. */
  if (reg_write(REG_MODE, MODE_RX)) goto failed;
  {
    uint32_t start = HAL_GetTick();
    for (;;) {
      if (reg_read(REG_MODE, &rx_start_mode) ||
          reg_read(REG_IRQ1, &rx_start_irq1)) goto failed;
      if ((rx_start_mode == MODE_RX || rx_start_mode == 0x0CU) &&
          (rx_start_irq1 & 0x10U) != 0U) break;
      if (HAL_GetTick() - start >= MODE_CHANGE_TIMEOUT_MS) goto failed;
      HAL_Delay(1U);
    }
  }
  rx_preamble_seen = 0U;
#else
  if (checked_write(REG_MODE, MODE_RX)) goto failed;
  rx_start_result = -5;
  rx_start_stage = "rx-ready";
  if (wait_flag(0xC0U)) goto failed; /* ModeReady + RxReady, not just PllLock. */
#endif
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
  rx_start_stage = APP_UPLINK_RX_FIFO ? "armed" : "ready";
  (void)reg_read(REG_MODE, &rx_start_mode);
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

#if APP_UPLINK_RX_FIFO
static int rx_poll_fifo(UplinkCommand *command)
{
  uint8_t irq1, irq2, byte;
  if (reg_read(REG_IRQ1, &irq1)) goto spi_failed;
  if (rx_preamble_seen == 0U && (irq1 & 0x02U) != 0U) {
    uint8_t afc_msb, afc_lsb, fei_msb, fei_lsb;
    rx_preamble_seen = 1U;
    rx_preamble_tick = HAL_GetTick();
    ++rx_stats.preambles;
    /* FEI is only an estimate at this observation; AFC is the applied
       correction. Always read MSB before LSB as required by the radio. */
    rx_frequency_valid = 0U;
    if (reg_read(0x1BU, &afc_msb) || reg_read(0x1CU, &afc_lsb) ||
        reg_read(0x1DU, &fei_msb) || reg_read(0x1EU, &fei_lsb)) goto spi_failed;
    rx_afc = (int16_t)(((uint16_t)afc_msb << 8U) | afc_lsb);
    rx_fei = (int16_t)(((uint16_t)fei_msb << 8U) | fei_lsb);
    rx_frequency_valid = 1U;
  }
  if (rx_decoder.collecting == 0U && (irq1 & 0x01U) != 0U) {
    uplink_decoder_begin_payload(&rx_decoder);
    ++rx_stats.syncs;
    rx_packet_tick = rx_last_bit_tick = HAL_GetTick();
  }
  /* A bounded drain also handles 66-byte commands, larger than the FIFO.
     Polling remains active even if the external DIO1 signal is absent. */
  for (unsigned i = 0U; i < UPLINK_MAX_RAW_BITS / 8U; ++i) {
    if (reg_read(0x3FU, &irq2)) goto spi_failed;
    if ((irq2 & 0x10U) != 0U) {
      ++rx_stats.overflows;
      goto rearm;
    }
    if (rx_decoder.collecting == 0U || (irq2 & 0x40U) != 0U) break;
    if (reg_read(0U, &byte)) goto spi_failed;
    ++rx_fifo_bytes;
    rx_last_bit_tick = HAL_GetTick();
    for (unsigned bit = 0U; bit < 8U; ++bit)
      uplink_decoder_push_bit(&rx_decoder, (uint8_t)((byte >> (7U - bit)) & 1U));
    if (rx_decoder.raw_count == 64U && rx_decoder.expected_blocks == 0U) {
      ++rx_stats.header_bad;
      goto rearm;
    }
    if (rx_decoder.ready != 0U) {
      int result = uplink_decoder_take(&rx_decoder, command);
      if (result != 0) ++rx_stats.valid;
      else ++rx_stats.rejected;
      (void)rx_start_internal();
      return result;
    }
  }
  /* A latched preamble without ASM must not retain a bad AFC/gain estimate
     indefinitely. The longest supported frame is under 200 ms on air. */
  if (rx_preamble_seen != 0U && rx_decoder.collecting == 0U &&
      HAL_GetTick() - rx_preamble_tick > 250U) {
    ++rx_sync_misses;
    goto rearm;
  }
  if (rx_decoder.collecting != 0U &&
      (HAL_GetTick() - rx_last_bit_tick > 100U ||
       HAL_GetTick() - rx_packet_tick > 250U)) {
    ++rx_stats.gaps;
    goto rearm;
  }
  return 0;
spi_failed:
  ++rx_fifo_spi_errors;
rearm:
  (void)rx_start_internal();
  return 0;
}
#endif

int SX1278_APRS_RxPoll(UplinkCommand *command)
{
  int result;
  if (command == NULL) return 0;
  if (rx_active == 0U) {
    if (rx_wanted != 0U && playing == 0U &&
        HAL_GetTick() - rx_attempt_tick >= 1000U) {
      uint8_t version;
      rx_attempt_tick = HAL_GetTick();
      if (ready == 0U && SX1278_APRS_Init(&version) != 0) {
        rx_attempt_tick = HAL_GetTick();
        return 0;
      }
      (void)rx_start_internal();
    }
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
#if APP_UPLINK_RX_FIFO
  return rx_poll_fifo(command);
#endif
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
      "UPLINK RX start rc=%d stage=%s mode=%02X irq1=%02X rxcfg=%02X attempt=%lu wanted=%u irqtest=%s path=%s\r\n",
      rx_start_result, rx_start_stage, rx_start_mode, rx_start_irq1,
      rx_start_config, (unsigned long)rx_attempts, rx_wanted,
      rx_irq_probe_result == 1U ? "PASS" : rx_irq_probe_result == 2U ? "FAIL" : "NONE",
      APP_UPLINK_RX_FIFO ? "FIFO" : "DCLK");
    return;
  }
  if (rx_irq_reported == 0U &&
#if APP_UPLINK_RX_FIFO
      rx_fifo_irqs != 0U) {
#else
      rx_stats.clocks != 0U) {
#endif
    rx_irq_reported = 1U;
    (void)snprintf(out, capacity,
      "UPLINK RX IRQ first: DIO1 %s detected\r\n",
      APP_UPLINK_RX_FIFO ? "FIFO level" : "clock");
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
#if APP_UPLINK_RX_FIFO
  size_t used = strlen(out);
  (void)snprintf(out + used, capacity - used,
    "RX FIFO bytes=%lu irq=%lu spi_errors=%lu nosync=%lu\r\n"
    "RX frequency valid=%u afc_hz=%ld fei_est_hz=%ld\r\n",
    (unsigned long)rx_fifo_bytes, (unsigned long)rx_fifo_irqs,
    (unsigned long)rx_fifo_spi_errors, (unsigned long)rx_sync_misses,
    rx_frequency_valid, (long)((int32_t)rx_afc * 15625L / 256L),
    (long)((int32_t)rx_fei * 15625L / 256L));
#endif
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
  tx_state.dio1_level = (uint8_t)HAL_GPIO_ReadPin(DIO1_GPIO_Port, DIO1_Pin);
  tx_state.dio2_level = (uint8_t)HAL_GPIO_ReadPin(DIO2_GPIO_Port, DIO2_Pin);
  tx_state.dio1_moder = (uint8_t)((DIO1_GPIO_Port->MODER >> 22U) & 3U);
  tx_state.dio2_moder = (uint8_t)((DIO2_GPIO_Port->MODER >> 24U) & 3U);
  tx_state.exti_imr = (uint8_t)((EXTI->IMR1 & DIO1_Pin) != 0U);
  tx_state.exti_rise = (uint8_t)((EXTI->RTSR1 & DIO1_Pin) != 0U);
  tx_state.exti_fall = (uint8_t)((EXTI->FTSR1 & DIO1_Pin) != 0U);
  tx_state.exti_rpend = (uint8_t)((EXTI->RPR1 & DIO1_Pin) != 0U);
  tx_state.exti_fpend = (uint8_t)((EXTI->FPR1 & DIO1_Pin) != 0U);
  tx_state.nvic_enabled = (uint8_t)NVIC_GetEnableIRQ(EXTI11_IRQn);
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
  uint16_t length = tx_kind == TX_KIND_GMSK && gmsk_tx_byte_count <= 2047U ?
                    (uint16_t)gmsk_tx_byte_count : 0U;
  uint8_t packet_config2 = tx_kind == TX_KIND_GMSK ?
                           (uint8_t)(0x40U | (length >> 8U)) : 0x00U;
  if (checked_write(REG_MODE, MODE_STANDBY) || wait_flag(0x80U) ||
      checked_write(0x06U, (uint8_t)(frf >> 16)) ||
      checked_write(0x07U, (uint8_t)(frf >> 8)) ||
      checked_write(0x08U, (uint8_t)frf) ||
      checked_write(0x27U, 0U) || checked_write(0x30U, 0U) ||
      checked_write(0x31U, packet_config2) ||
      checked_write(0x40U, 0U) || checked_write(0x41U, 0U) ||
      (tx_kind == TX_KIND_GMSK &&
       (checked_write(0x25U, 0U) || checked_write(0x26U, 0U) ||
        checked_write(0x32U, (uint8_t)length) ||
        checked_write(0x35U, (uint8_t)(0x80U | GMSK_FIFO_THRESHOLD)) ||
        reg_write(0x3FU, 0x10U) || /* Discard any payload left by RX. */
        gmsk_fifo_write(64U))) ||
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
    "SX TX snapshot valid=%u rc=%d direct=%u mode=%02X irq1=%02X irq2=%02X bitrate_msb=%02X frf=%02X%02X%02X pa=%02X ramp=%02X ocp=%02X pkt=%02X tcxo=%02X dac=%02X dev=%02X%02X samples=%lu edges=%lu dio=%u/%u moder=%u/%u exti=%u/%u/%u pend=%u/%u nvic=%u\r\n",
    tx_state.valid, tx_state.result, tx_state.direct_fallback, tx_state.mode, tx_state.irq1, tx_state.irq2,
    tx_state.datamod, tx_state.frf[0], tx_state.frf[1], tx_state.frf[2], tx_state.pa, tx_state.ramp,
    tx_state.ocp, tx_state.packet, tx_state.tcxo, tx_state.dac,
    tx_state.fdev[0], tx_state.fdev[1], (unsigned long)emitted_samples, (unsigned long)data_edges,
    tx_state.dio1_level, tx_state.dio2_level, tx_state.dio1_moder, tx_state.dio2_moder,
    tx_state.exti_imr, tx_state.exti_rise, tx_state.exti_fall,
    tx_state.exti_rpend, tx_state.exti_fpend, tx_state.nvic_enabled);
  if (tx_state.fifo_tx != 0U) {
    size_t used = strlen(out);
    (void)snprintf(out + used, capacity - used,
      "GMSK FIFO queued=%lu/%lu pad_bits=%u packet_sent=%u drained=%u\r\n",
      (unsigned long)tx_state.fifo_bytes, (unsigned long)tx_state.fifo_total,
      tx_state.pad_bits, tx_state.packet_sent, tx_state.drained);
  }
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
  /* The negotiated RateID selects the radio's internal bit clock and Fdev. */
  static const uint32_t rates[] = {4800U, 9600U, 19200U, 38400U,
                                   57600U, 115200U};
  static const uint32_t deviations[] = {1200U, 2400U, 4800U, 9600U,
                                        14400U, 28800U};
  uint8_t profile = 0U;
  if (playing) return -1;
  while (profile < sizeof(rates) / sizeof(rates[0]) &&
         (rates[profile] != bitrate || deviations[profile] != deviation_hz))
    ++profile;
  gmsk_bit_count = 0U;
  if (!ready || bits == NULL || bit_count == 0U ||
      bit_count > GMSK_MAX_PACKED_BYTES * 8U ||
      profile >= sizeof(rates) / sizeof(rates[0])) return -1;
  memcpy(gmsk_bits, bits, (bit_count + 7U) / 8U);
  if ((bit_count & 7U) != 0U)
    gmsk_bits[(bit_count >> 3U)] &= (uint8_t)(0xFFU << (8U - (bit_count & 7U)));
  gmsk_bit_count = bit_count;
  gmsk_tx_byte_count = (bit_count + 7U) / 8U;
  gmsk_tx_byte_index = 0U;
  gmsk_bitrate = bitrate;
  gmsk_deviation_hz = deviation_hz;
  return (int)((gmsk_tx_byte_count * 8000U + bitrate - 1U) / bitrate);
}

int SX1278_APRS_SendGmsk(uint32_t frequency_hz, uint32_t budget_ms,
                         SX1278_APRS_ServiceFn service, void *context)
{
  uint8_t resume_rx;
  uint32_t start, last_check, bitrate_reg, fdev_reg;
  uint32_t empty_tick = 0U;
  uint8_t empty_seen = 0U;
  int result = 0;
  if (!ready || playing || gmsk_bit_count == 0U || gmsk_bitrate == 0U)
    return -1;
  if (frequency_hz < 137000000UL || frequency_hz > 175000000UL ||
      budget_ms < (gmsk_tx_byte_count * 8000U + gmsk_bitrate - 1U) /
                    gmsk_bitrate + 200U) return -2;
  resume_rx = rx_wanted;
  memset(&tx_state, 0, sizeof(tx_state));
  first_failure.stage = 0U;
  emitted_samples = data_edges = 0U;
  tx_state.fifo_tx = 1U;
  tx_state.fifo_total = (uint32_t)gmsk_tx_byte_count;
  tx_state.pad_bits = (uint8_t)(gmsk_tx_byte_count * 8U - gmsk_bit_count);
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
      checked_write(0x31U, 0x40U) || checked_write(0x40U, 0x00U) ||
      checked_write(0x41U, 0x00U) ||
      checked_write(0x5DU, 0x00U) ||
      SX1278_APRS_SetPowerDbm(APP_GMSK_POWER_DBM)) {
    result = -3;
    goto cleanup;
  }
  /* FIFO TX keeps Gaussian BT=0.5 without an external DCLK dependency. */
  tx_kind = TX_KIND_GMSK;
  clock_pin(GPIO_MODE_INPUT);
  data_pin(GPIO_MODE_INPUT);
  gmsk_tx_byte_index = 0U;
  __HAL_GPIO_EXTI_CLEAR_IT(DIO1_Pin);
  HAL_NVIC_ClearPendingIRQ(EXTI11_IRQn);
  result = begin_tx(frequency_hz, 0x49U);
  if (result != 0) goto cleanup;
  __DMB();
  playing = 1U;
  last_check = HAL_GetTick();
  while (playing) {
    uint8_t irq2 = 0U;
    if (reg_read(0x3FU, &irq2)) { result = -8; break; }
    if (gmsk_tx_byte_index < gmsk_tx_byte_count &&
        (irq2 & IRQ2_FIFO_EMPTY) != 0U) {
      result = -11; /* Underrun: do not report a broken stream as success. */
      break;
    }
    if (gmsk_tx_byte_index < gmsk_tx_byte_count &&
        (irq2 & IRQ2_FIFO_LEVEL) == 0U) {
      if (gmsk_fifo_write(GMSK_FIFO_THRESHOLD) != 0) {
        result = -8;
        break;
      }
      continue; /* Re-read status after refill, before calling other work. */
    }
    if (gmsk_tx_byte_index == gmsk_tx_byte_count) {
      if (gmsk_tx_byte_count <= 2047U && (irq2 & IRQ2_PACKET_SENT) != 0U) {
        tx_state.packet_sent = 1U;
        break;
      }
      if (gmsk_tx_byte_count > 2047U && (irq2 & IRQ2_FIFO_EMPTY) != 0U) {
        if (empty_seen == 0U) { empty_tick = HAL_GetTick(); empty_seen = 1U; }
        /* FIFO empty precedes completion of the shift register. Allow eight
           bits plus four filter clocks, with one tick for quantisation. */
        if (HAL_GetTick() - empty_tick >=
            (12000U + gmsk_bitrate - 1U) / gmsk_bitrate + 1U) {
          tx_state.drained = 1U;
          break;
        }
      }
    }
    if (HAL_GetTick() - start >= budget_ms) { result = -4; break; }
    if (HAL_GetTick() - last_check >= 100U) {
      if (tx_still_active() != 0) { result = -8; break; }
      last_check = HAL_GetTick();
    }
    if (service != NULL) service(context);
  }
  if (result == 0 && tx_still_active() != 0) result = -8;
cleanup:
  (void)capture_tx();
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
