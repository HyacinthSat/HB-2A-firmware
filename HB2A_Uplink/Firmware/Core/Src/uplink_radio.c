#include "uplink_radio.h"
#include "main.h"
#include <string.h>
#include <stdio.h>

extern SPI_HandleTypeDef hspi1;
static uint8_t ready;
static struct {
  uint32_t elapsed_ms, expected_ms;
  uint8_t valid, mode, irq1, irq2, frf[3], rate[2], fdev[2];
  uint8_t sync, packet, pa, ramp, length;
} tx_diag;

static int read_reg(uint8_t reg, uint8_t *value)
{
  uint8_t tx[2] = {reg, 0}, rx[2] = {0};
  HAL_GPIO_WritePin(RA02_NSS_GPIO_Port, RA02_NSS_Pin, GPIO_PIN_RESET);
  HAL_StatusTypeDef result = HAL_SPI_TransmitReceive(&hspi1, tx, rx, 2U, 5U);
  HAL_GPIO_WritePin(RA02_NSS_GPIO_Port, RA02_NSS_Pin, GPIO_PIN_SET);
  *value = rx[1];
  return result == HAL_OK ? 0 : -1;
}

static int write_reg(uint8_t reg, uint8_t value)
{
  uint8_t tx[2] = {(uint8_t)(reg | 0x80U), value};
  HAL_GPIO_WritePin(RA02_NSS_GPIO_Port, RA02_NSS_Pin, GPIO_PIN_RESET);
  HAL_StatusTypeDef result = HAL_SPI_Transmit(&hspi1, tx, 2U, 5U);
  HAL_GPIO_WritePin(RA02_NSS_GPIO_Port, RA02_NSS_Pin, GPIO_PIN_SET);
  return result == HAL_OK ? 0 : -1;
}

static int checked_write(uint8_t reg, uint8_t value)
{
  uint8_t actual;
  uint32_t start = HAL_GetTick();
  if (write_reg(reg, value)) return -1;
  do {
    if (read_reg(reg, &actual)) return -1;
    if (actual == value) return 0;
    if (reg != 0x01U) return -1;
    HAL_Delay(1U);
  } while (HAL_GetTick() - start < 20U);
  return -1;
}

static void reset_radio(void)
{
  ready = 0U;
  HAL_GPIO_WritePin(RA02_RST_GPIO_Port, RA02_RST_Pin, GPIO_PIN_RESET);
  HAL_Delay(1U);
  HAL_GPIO_WritePin(RA02_RST_GPIO_Port, RA02_RST_Pin, GPIO_PIN_SET);
  HAL_Delay(6U);
}

static int standby(void)
{
  if (checked_write(0x01U, 0x09U) == 0) return 0;
  reset_radio();
  return -1;
}

int UplinkRadio_Init(void)
{
  uint8_t version;
  uint32_t frf = (uint32_t)((435400000ULL * 524288ULL + 16000000ULL) / 32000000ULL);
  static const uint8_t config[][2] = {
    {0x02, 0x1A}, {0x03, 0x0B}, /* 32 MHz / 6667 = 4799.76 bit/s */
    {0x04, 0x00}, {0x05, 0x14}, /* deviation = 1220.703 Hz */
    {0x09, 0x88},               /* PA_BOOST, nominal +10 dBm */
    {0x0A, 0x49},               /* Gaussian BT=0.5, 40 us PA ramp */
    {0x0B, 0x2B}, {0x4D, 0x84}, /* OCP 100 mA, normal PA DAC */
    /* The decoder needs 128 alternating bits after clock acquisition. */
    {0x25, 0x00}, {0x26, UPLINK_RADIO_PREAMBLE_BYTES},
    {0x27, 0x33},               /* 0x55 preamble, sync on, 4 sync bytes */
    {0x28, 0x1A}, {0x29, 0xCF}, {0x2A, 0xFC}, {0x2B, 0x1D},
    {0x30, 0x00},               /* fixed length, no whitening/CRC/address */
    {0x31, 0x40},               /* packet mode (FIFO -> shaped FSK) */
    {0x35, 0x00}, {0x40, 0x00}, {0x41, 0x00}, {0x5D, 0x00}
  };
  reset_radio();
  if (read_reg(0x42U, &version) || version != 0x12U) return -1;
  if (checked_write(0x01U, 0x08U) || checked_write(0x01U, 0x09U)) return -2;
  if (checked_write(0x06U, (uint8_t)(frf >> 16)) ||
      checked_write(0x07U, (uint8_t)(frf >> 8)) ||
      checked_write(0x08U, (uint8_t)frf)) return -3;
  for (unsigned i = 0U; i < sizeof(config) / sizeof(config[0]); ++i)
    if (checked_write(config[i][0], config[i][1])) return -4;
  ready = 1U;
  return 0;
}

int UplinkRadio_Ready(void) { return ready != 0U; }

static void capture_tx(void)
{
  tx_diag.valid = 0U;
  if (read_reg(1U, &tx_diag.mode) || read_reg(0x3eU, &tx_diag.irq1) ||
      read_reg(0x3fU, &tx_diag.irq2) || read_reg(0x27U, &tx_diag.sync) ||
      read_reg(0x31U, &tx_diag.packet) || read_reg(9U, &tx_diag.pa) ||
      read_reg(0x0aU, &tx_diag.ramp) || read_reg(0x32U, &tx_diag.length)) return;
  for (unsigned i=0; i<3U; ++i)
    if (read_reg((uint8_t)(6U+i), &tx_diag.frf[i])) return;
  for (unsigned i=0; i<2U; ++i)
    if (read_reg((uint8_t)(2U+i), &tx_diag.rate[i]) ||
        read_reg((uint8_t)(4U+i), &tx_diag.fdev[i])) return;
  tx_diag.valid = 1U;
}

void UplinkRadio_TxReport(char *out, size_t capacity)
{
  if (!out || !capacity) return;
  (void)snprintf(out, capacity,
    "TXMS=%lu EXPECT=%lu SNAP=%u MODE=%02X IRQ=%02X/%02X FRF=%02X%02X%02X "
    "RATE=%02X%02X DEV=%02X%02X SYNC=%02X PKT=%02X PA=%02X RAMP=%02X LEN=%u",
    (unsigned long)tx_diag.elapsed_ms, (unsigned long)tx_diag.expected_ms,
    tx_diag.valid, tx_diag.mode, tx_diag.irq1, tx_diag.irq2,
    tx_diag.frf[0], tx_diag.frf[1], tx_diag.frf[2], tx_diag.rate[0], tx_diag.rate[1],
    tx_diag.fdev[0], tx_diag.fdev[1], tx_diag.sync, tx_diag.packet,
    tx_diag.pa, tx_diag.ramp, tx_diag.length);
}

int UplinkRadio_Send(const uint8_t *frame, size_t length, uint8_t power_dbm)
{
  uint8_t fifo[67], flags;
  uint32_t start;
  int result = -5;
  memset(&tx_diag, 0, sizeof(tx_diag));
  if (!frame || length < 30U || length > 86U || power_dbm < 2U || power_dbm > 17U)
    return -1;
  for (unsigned i = 0; i < 16U; ++i) if (frame[i] != 0x55U) return -1;
  if (memcmp(frame + 16U, "\x1A\xCF\xFC\x1D", 4U)) return -1;
  if (!ready && UplinkRadio_Init()) return -2;
  if (standby()) return -3;
  /* Only NRZI BCH+CRC bytes enter the FIFO: radio supplies preamble and ASM. */
  size_t payload = length - 20U;
  tx_diag.expected_ms = (uint32_t)(((UPLINK_RADIO_PREAMBLE_BYTES + 4U + payload) *
                                     8ULL * 6667ULL + 31999ULL) / 32000ULL);
  if (checked_write(0x09U, (uint8_t)(0x80U | (power_dbm - 2U))) ||
      checked_write(0x32U, (uint8_t)payload) ||
      checked_write(0x35U, 0x80U) || /* Start with nonempty FIFO; filled before TX. */
      write_reg(0x3FU, 0x10U)) goto done; /* clear FIFO before filling */
  if (read_reg(0x3FU, &flags)) goto done;
  if (flags & 0x08U) { result = -7; goto done; } /* Stale PacketSent is not success. */
  fifo[0] = 0x80U;
  memcpy(fifo + 1U, frame + 20U, payload);
  HAL_GPIO_WritePin(RA02_NSS_GPIO_Port, RA02_NSS_Pin, GPIO_PIN_RESET);
  HAL_StatusTypeDef sent = HAL_SPI_Transmit(&hspi1, fifo, (uint16_t)(payload + 1U), 10U);
  HAL_GPIO_WritePin(RA02_NSS_GPIO_Port, RA02_NSS_Pin, GPIO_PIN_SET);
  if (sent != HAL_OK) goto done;
  HAL_GPIO_WritePin(LED_TX_GPIO_Port, LED_TX_Pin, GPIO_PIN_SET);
  start = HAL_GetTick();
  if (checked_write(0x01U, 0x0BU)) goto done;
  do {
    if (read_reg(0x3FU, &flags)) goto done;
    tx_diag.elapsed_ms = HAL_GetTick() - start;
    if (flags & 0x08U) {
      /* Allow tick quantization; completing much before airtime means stale
         flags or a wrong packet/rate setup. This still cannot prove RF power. */
      result = tx_diag.elapsed_ms + 3U >= tx_diag.expected_ms ? 0 : -7;
      break;
    }
    HAL_Delay(1U);
  } while (HAL_GetTick() - start < 500U);
done:
  capture_tx();
  if (standby()) result = -6;
  if (result != 0) reset_radio();
  HAL_GPIO_WritePin(LED_TX_GPIO_Port, LED_TX_Pin, GPIO_PIN_RESET);
  return result;
}
