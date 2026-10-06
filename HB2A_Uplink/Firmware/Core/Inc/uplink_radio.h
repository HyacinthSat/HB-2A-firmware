#ifndef UPLINK_RADIO_H
#define UPLINK_RADIO_H
#include <stddef.h>
#include <stdint.h>
#define UPLINK_RADIO_PREAMBLE_BYTES 32U
int UplinkRadio_Init(void);
int UplinkRadio_Ready(void);
int UplinkRadio_Send(const uint8_t *frame, size_t length, uint8_t power_dbm);
/* Cached hardware observations from the last send, including failures. */
void UplinkRadio_TxReport(char *out, size_t capacity);
#endif
