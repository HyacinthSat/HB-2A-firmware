/* MAX-M10S SPG 5.10 configuration database, RAM only (no flash writes).
   Keys cross-checked against SparkFun_u-blox_GNSS_v3/u-blox_config_keys.h. */
#include "ubx.h"
#include "app_config.h"
#include <string.h>

#define UBX_MAX_PAYLOAD 512U
#define RX_SIZE 2048U

static UART_HandleTypeDef *s_uart;
static uint8_t s_rx_byte;
static uint8_t s_ring[RX_SIZE];
static volatile uint16_t s_head, s_tail;
static volatile uint32_t s_rx_errors;
static uint8_t s_state, s_cls, s_id, s_a, s_b;
static uint16_t s_len, s_pos;
static uint8_t s_payload[UBX_MAX_PAYLOAD];
static int s_ack, s_verified;

typedef struct {
  uint32_t key;
  uint16_t value;
  uint8_t size;
} ConfigEntry;

static const ConfigEntry s_config[] = {
  {0x20110021UL, APP_GNSS_DYN_MODEL, 1U}, /* NAVSPG-DYNMODEL: airborne <1g */
  {0x20110011UL, 3U, 1U},               /* NAVSPG-FIXMODE: auto 2D/3D */
  {0x30210001UL, 1000U, 2U},            /* RATE-MEAS: ms */
  {0x30210002UL, 1U, 2U},               /* RATE-NAV: every measurement */
  {0x20210003UL, 0U, 1U},               /* RATE-TIMEREF: UTC */
  {0x20910007UL, 1U, 1U},               /* MSGOUT-UBX_NAV_PVT_UART1 */
  {0x20910016UL, 1U, 1U},               /* MSGOUT-UBX_NAV_SAT_UART1: every solution */
  {0x10730001UL, 1U, 1U},               /* UART1INPROT-UBX */
  {0x10740001UL, 1U, 1U},               /* UART1OUTPROT-UBX */
  {0x10740002UL, 0U, 1U}                /* UART1OUTPROT-NMEA */
};
#define CONFIG_COUNT (sizeof(s_config) / sizeof(s_config[0]))

static uint32_t rd32(const uint8_t *p)
{
  return p[0] | ((uint32_t)p[1] << 8) | ((uint32_t)p[2] << 16) | ((uint32_t)p[3] << 24);
}

static void wr32(uint8_t *p, uint32_t value)
{
  for (unsigned i = 0U; i < 4U; ++i) p[i] = (uint8_t)(value >> (8U * i));
}

static void checksum(uint8_t byte)
{
  s_a = (uint8_t)(s_a + byte);
  s_b = (uint8_t)(s_b + s_a);
}

static HAL_StatusTypeDef send_frame(uint8_t id, const uint8_t *payload, uint16_t length)
{
  uint8_t wire[96], a = 0U, b = 0U;
  if (s_uart == NULL || length > sizeof(wire) - 8U) return HAL_ERROR;
  wire[0] = 0xB5U; wire[1] = 0x62U; wire[2] = 0x06U; wire[3] = id;
  wire[4] = (uint8_t)length; wire[5] = (uint8_t)(length >> 8);
  memcpy(wire + 6, payload, length);
  for (uint16_t i = 2U; i < length + 6U; ++i) {
    a = (uint8_t)(a + wire[i]); b = (uint8_t)(b + a);
  }
  wire[length + 6U] = a; wire[length + 7U] = b;
  return HAL_UART_Transmit(s_uart, wire, length + 8U, 200U);
}

int Ubx_Init(UART_HandleTypeDef *huart)
{
  UART_HandleTypeDef *previous = s_uart;
  if (huart == NULL) return -1;
  /* Detach callbacks before aborting: an IRQ must not rearm the old RX. */
  s_uart = NULL;
  __DMB();
  if (previous != NULL && previous != huart &&
      HAL_UART_AbortReceive(previous) != HAL_OK) return -1;
  if (HAL_UART_AbortReceive(huart) != HAL_OK) return -1;
  s_state = 0U; s_head = 0U; s_tail = 0U; s_rx_errors = 0U;
  s_ack = 0; s_verified = 0;
  __HAL_UART_CLEAR_OREFLAG(huart);
  s_uart = huart;
  return HAL_UART_Receive_IT(huart, &s_rx_byte, 1U) == HAL_OK ? 0 : -1;
}

/* Single producer in IRQ, single consumer in main; no blocking UART TX here. */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
  if (huart != s_uart) return;
  uint16_t next = (uint16_t)((s_head + 1U) % RX_SIZE);
  if (next == s_tail) ++s_rx_errors;
  else {
    s_ring[s_head] = s_rx_byte;
    __DMB();
    s_head = next;
  }
  if (HAL_UART_Receive_IT(huart, &s_rx_byte, 1U) != HAL_OK) ++s_rx_errors;
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
  if (huart != s_uart) return;
  ++s_rx_errors;
  __HAL_UART_CLEAR_OREFLAG(huart);
  if (huart->RxState == HAL_UART_STATE_READY)
    (void)HAL_UART_Receive_IT(huart, &s_rx_byte, 1U);
}

int Ubx_ReadByte(uint8_t *b)
{
  if (s_tail == s_head) return 0;
  *b = s_ring[s_tail];
  __DMB();
  s_tail = (uint16_t)((s_tail + 1U) % RX_SIZE);
  return 1;
}

uint32_t Ubx_RxErrors(void) { return s_rx_errors; }

uint8_t Ubx_NavPvtHas3DFix(const uint8_t *p, uint16_t len)
{
  /* M10 NAV-PVT is 92 bytes. Require date, time, fullyResolved and gnssFixOK. */
  return p != NULL && len == 92U && p[20] == 3U && (p[21] & 1U) != 0U &&
         (p[11] & 7U) == 7U;
}

static void receive_frame(void)
{
  if (s_cls == 0x01U && s_id == 0x07U && s_len == 92U) {
    (void)GNSS_OnUbxNavPvt(s_payload, s_len);
  } else if (s_cls == 0x05U && s_len == 2U && s_payload[0] == 0x06U) {
    if (s_payload[1] == 0x8AU && (s_id == 0U || s_id == 1U)) s_ack = s_id ? 1 : -1;
    if (s_payload[1] == 0x8BU && s_id == 0U) s_verified = -1;
  } else if (s_cls == 0x06U && s_id == 0x8BU && s_len >= 4U &&
             s_payload[0] == 1U && s_payload[1] == 0U &&
             s_payload[2] == 0U && s_payload[3] == 0U) {
    uint16_t pos = 4U;
    uint32_t seen = 0U;
    while (pos + 4U <= s_len) {
      uint32_t key = rd32(s_payload + pos);
      unsigned i;
      pos += 4U;
      for (i = 0U; i < CONFIG_COUNT && key != s_config[i].key; ++i) { }
      if (i == CONFIG_COUNT || pos + s_config[i].size > s_len) { s_verified = -1; return; }
      uint16_t value = s_payload[pos];
      if (s_config[i].size == 2U) value |= (uint16_t)s_payload[pos + 1U] << 8;
      if (value != s_config[i].value) { s_verified = -1; return; }
      seen |= 1UL << i;
      pos += s_config[i].size;
    }
    s_verified = pos == s_len && seen == (1UL << CONFIG_COUNT) - 1U ? 1 : -1;
  }
}

void Ubx_OnByte(uint8_t b)
{
  switch (s_state) {
    case 0: if (b == 0xB5U) s_state = 1U; break;
    case 1:
      s_state = b == 0x62U ? 2U : (b == 0xB5U ? 1U : 0U);
      s_a = 0U; s_b = 0U; break;
    case 2: s_cls = b; checksum(b); s_state = 3U; break;
    case 3: s_id = b; checksum(b); s_state = 4U; break;
    case 4: s_len = b; checksum(b); s_state = 5U; break;
    case 5:
      s_len |= (uint16_t)b << 8; checksum(b); s_pos = 0U;
      /* Reject an impossible payload before entering a long discard loop.
         A corrupted 0xFFFF length must not consume the next 65 KB of UART
         traffic and hide valid NAV-PVT frames. The next byte is treated as
         a fresh sync candidate. */
      if (s_len > UBX_MAX_PAYLOAD) {
        s_state = 0U;
        s_len = 0U;
      } else {
        s_state = s_len ? 6U : 7U;
      }
      break;
    case 6:
      /* Long NAV-SAT frames still need framing; never scan their data for sync. */
      if (s_pos < UBX_MAX_PAYLOAD) s_payload[s_pos] = b;
      ++s_pos; checksum(b);
      if (s_pos == s_len) s_state = 7U;
      break;
    case 7: s_state = b == s_a ? 8U : (b == 0xB5U ? 1U : 0U); break;
    case 8:
      s_state = 0U;
      if (b == s_b) {
        if (s_len <= UBX_MAX_PAYLOAD) receive_frame();
      }
      else if (b == 0xB5U) s_state = 1U;
      break;
    default: s_state = 0U; break;
  }
}

static void service_for(uint32_t ms, void (*service)(void))
{
  uint32_t start = HAL_GetTick();
  do { service(); HAL_Delay(1U); } while (HAL_GetTick() - start < ms);
}

int Ubx_ConfigureModule(void (*service)(void))
{
  uint8_t payload[64] = {0U, 1U, 0U, 0U}; /* VALSET v0, RAM layer */
  uint8_t query[4U + CONFIG_COUNT * 4U] = {0U}; /* VALGET v0, RAM layer 0 */
  uint16_t len = 4U;
  if (service == NULL || s_uart == NULL) return -1;
  for (unsigned i = 0U; i < CONFIG_COUNT; ++i) {
    wr32(payload + len, s_config[i].key); len += 4U;
    payload[len++] = (uint8_t)s_config[i].value;
    if (s_config[i].size == 2U) payload[len++] = (uint8_t)(s_config[i].value >> 8);
    wr32(query + 4U + i * 4U, s_config[i].key);
  }
  /* Service RX while the receiver starts; no blind HAL_Delay that loses ACKs. */
  service_for(500U, service);
  s_ack = 0; s_verified = 0;
  if (send_frame(0x8AU, payload, len) != HAL_OK) return -1;
  uint32_t start = HAL_GetTick();
  while (!s_ack && HAL_GetTick() - start < 1500U) service_for(1U, service);
  if (s_ack != 1) return s_ack < 0 ? -2 : -3;
  if (send_frame(0x8BU, query, sizeof(query)) != HAL_OK) return -1;
  start = HAL_GetTick();
  while (!s_verified && HAL_GetTick() - start < 1500U) service_for(1U, service);
  return s_verified == 1 ? 0 : (s_verified < 0 ? -4 : -5);
}
