#include "ground_station.h"
#include "ground_command.h"
#include "uplink_radio.h"
#include "main.h"
#include "usbd_cdc_if.h"
#include <stdio.h>
#include <string.h>

#define RX_SIZE 256U
static uint8_t rx[RX_SIZE];
static volatile uint16_t head, tail;
static volatile uint8_t overflow, reset_input;
static char line[96], response[320], radio_report[240];
static size_t used;
static uint8_t dropping, pending;
static uint8_t line_coding[7] = {0x00, 0xC2, 0x01, 0, 0, 0, 8};

void GroundStation_SetLineCoding(const uint8_t *data) { memcpy(line_coding, data, 7U); }
void GroundStation_GetLineCoding(uint8_t *data) { memcpy(data, line_coding, 7U); }
void GroundStation_ResetInput(void) { head = tail = 0U; overflow = 0U; reset_input = 1U; }

void GroundStation_Receive(const uint8_t *data, uint32_t length)
{
  for (uint32_t i = 0U; i < length; ++i) {
    if (data[i] == 0x18U) {
      /* USB host uses CAN to abandon a partial line after reconnect. */
      reset_input = 1U;
      continue;
    }
    uint16_t next = (uint16_t)((head + 1U) % RX_SIZE);
    if (overflow || next == tail) { overflow = 1U; continue; }
    rx[head] = data[i];
    __DMB();
    head = next;
  }
}

void GroundStation_Init(void) { (void)UplinkRadio_Init(); }

static void execute(void)
{
  uint8_t frame[86], power;
  int length, result;
  if (strcmp(line, "STATUS") == 0) {
    if (!UplinkRadio_Ready()) (void)UplinkRadio_Init();
    (void)snprintf(response, sizeof(response),
      "OK HB2A-UPLINK/1 RADIO=%s FREQ=435400000 RATE=4799.76 FDEV=1220.70 BT=0.5 PREAMBLE=32\r\n",
      UplinkRadio_Ready() ? "READY" : "ERROR");
  } else {
    length = GroundCommand_Encode(line, &power, frame, sizeof(frame));
    if (length < 0) {
      (void)snprintf(response, sizeof(response), "ERR COMMAND %d\r\n", length);
    } else {
      result = UplinkRadio_Send(frame, (size_t)length, power);
      UplinkRadio_TxReport(radio_report, sizeof(radio_report));
      if (result) (void)snprintf(response, sizeof(response), "ERR RADIO %d %s\r\n",
                                 result, radio_report);
      else (void)snprintf(response, sizeof(response), "OK SENT BYTES=%u DBM=%u %s\r\n",
                         (unsigned)length + UPLINK_RADIO_PREAMBLE_BYTES - 16U, power,
                         radio_report);
    }
  }
  pending = 1U;
}

void GroundStation_Poll(void)
{
  uint8_t c;
  uint32_t irq = __get_PRIMASK();
  __disable_irq();
  if (reset_input) {
    used = 0U;
    dropping = reset_input = 0U;
    pending = 0U;
  }
  if (overflow) {
    head = tail = 0U;
    used = 0U;
    dropping = 1U; /* Discard through newline after overflow; never send a suffix. */
    overflow = 0U;
  }
  __set_PRIMASK(irq);
  if (pending) {
    if (CDC_Transmit_FS((uint8_t *)response, (uint16_t)strlen(response)) == USBD_OK)
      pending = 0U;
    return;
  }
  while (tail != head) {
    c = rx[tail];
    tail = (uint16_t)((tail + 1U) % RX_SIZE);
    if (c == '\r') continue;
    if (c == '\n') {
      if (dropping) {
        strcpy(response, "ERR LINE_TOO_LONG_OR_OVERFLOW\r\n");
        pending = 1U;
      } else if (used) {
        line[used] = 0;
        execute();
      }
      used = 0U;
      dropping = 0U;
      return;
    }
    if (dropping) continue;
    if (used + 1U >= sizeof(line) || c < 0x20U || c > 0x7EU) dropping = 1U;
    else line[used++] = (char)c;
  }
}
