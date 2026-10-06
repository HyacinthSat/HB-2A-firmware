#ifndef SX1278_APRS_H
#define SX1278_APRS_H
#include <stddef.h>
#include <stdint.h>
#include "uplink_protocol.h"

int SX1278_APRS_Init(uint8_t *version);
/* Read-only startup diagnostics, called only while the radios are idle. */
void SX1278_APRS_Diagnose(char *out, size_t capacity);
/* First checked-write failure; empty string if there was no such failure. */
void SX1278_APRS_ConfigError(char *out, size_t capacity);
/* Startup failure diagnostic only: hold NSS low/high for 2 s each, no SPI. */
void SX1278_APRS_TestNss(void (*log)(const char *));
/* Build before reserving the airtime; returns one packet's duration in ms. */
int SX1278_APRS_Prepare(const char *info, size_t length);
/* Same as above, with an explicit AX.25 source and destination address. */
int SX1278_APRS_PrepareAddressed(const char *source, const char *destination,
                                 const uint8_t *info, size_t length);
int SX1278_APRS_PrepareAddressedPath(const char *source, const char *destination,
                                     const char *path, const uint8_t *info,
                                     size_t length);
typedef void (*SX1278_APRS_ServiceFn)(void *context);
int SX1278_APRS_SendWithContext(uint32_t frequency_hz, uint32_t budget_ms,
                                SX1278_APRS_ServiceFn service, void *context);
/* Synchronous, bounded; service() keeps GNSS alive. Idle state is standby.
   Returns 0 only after the timer has emitted the complete packet and RF is off. */
int SX1278_APRS_Send(uint32_t frequency_hz, uint32_t budget_ms, void (*service)(void));
/* Prepare/send a raw MSB-first GMSK bitstream. The radio applies its
   Gaussian BT=0.5 shaping; the caller supplies the already coded bits. */
int SX1278_APRS_PrepareGmsk(const uint8_t *bits, size_t bit_count,
                            uint32_t bitrate, uint32_t deviation_hz);
int SX1278_APRS_SendGmsk(uint32_t frequency_hz, uint32_t budget_ms,
                         SX1278_APRS_ServiceFn service, void *context);
uint8_t SX1278_APRS_Ready(void);
/* Start/stop the 435.400 MHz continuous-FSK DIO receive path. Failed starts
   remain requested and RxPoll retries once per second; RxStop cancels this. */
int SX1278_APRS_RxStart(void);
void SX1278_APRS_RxStop(void);
uint8_t SX1278_APRS_RxActive(void);
/* Poll a frame completed by the DIO1 clock/DIO2 data receiver. */
int SX1278_APRS_RxPoll(UplinkCommand *command);
/* Foreground-only, rate-limited diagnostics; empty output when nothing is due.
   DCLK activity alone does not mean a packet was received. */
void SX1278_APRS_RxReport(char *out, size_t capacity);
/* Diagnostic only: 0=normal TX, 1=TX ready after missing PLL indication.
   Negative values indicate failure. Neither result confirms RF output. */
int SX1278_APRS_CarrierTest(uint32_t frequency_hz, uint32_t duration_ms, void (*service)(void));
int SX1278_APRS_SetPowerDbm(uint8_t dbm);
/* Cached registers captured while TX was requested/active, never post-standby. */
void SX1278_APRS_TxReport(char *out, size_t capacity);
#endif
