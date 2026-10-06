#ifndef HB2A_HIGH_SPEED_SERVICE_H
#define HB2A_HIGH_SPEED_SERVICE_H

#include "uplink_protocol.h"

#include <stdint.h>

typedef enum {
  HS_STATE_IDLE = 0,
  HS_STATE_WAIT_PARAMETERS,
  HS_STATE_WAIT_CONFIRM,
  HS_STATE_TRANSFER
} HighSpeedState;

typedef enum {
  HS_REQUEST_NONE = 0,
  HS_REQUEST_CAPTURE,
  HS_REQUEST_FILE
} HighSpeedRequest;

typedef enum {
  HS_EVENT_NONE = 0,
  HS_EVENT_SEND_ACK,
  HS_EVENT_SEND_SUPPORT,
  HS_EVENT_SEND_CONNECTION,
  HS_EVENT_SEND_REJECT,
  HS_EVENT_START_TRANSFER,
  HS_EVENT_CANCELLED,
  HS_EVENT_TIMEOUT,
  HS_EVENT_TRANSFER_DONE
} HighSpeedEvent;

typedef struct {
  uint8_t rate_id;
  uint8_t code_id;
  uint8_t intlv_id;
} HighSpeedParameters;

typedef struct {
  HighSpeedState state;
  HighSpeedRequest request;
  HighSpeedParameters parameters;
  uint8_t camera;
  uint16_t width;
  uint16_t height;
  int8_t quality;
  uint8_t file_index;
  uint8_t connection_attempts;
  uint32_t deadline_ms;
  HighSpeedEvent events[4];
  uint8_t event_head;
  uint8_t event_count;
  char reject_reason[8];
} HighSpeedService;

/* The current balloon build advertises all six rate IDs, four code IDs and
   six interleave IDs defined by AI_需求.md. */
#define HS_SUPPORTED_RATE_MASK  0x3FU
#define HS_SUPPORTED_CODE_MASK  0x0FU
#define HS_SUPPORTED_INTLV_MASK 0x3FU

void high_speed_service_init(HighSpeedService *service);

/* Start negotiation after a valid CAPTR or REQF command. */
int high_speed_service_start(HighSpeedService *service,
                             const UplinkCommand *command,
                             uint32_t now_ms);

/* Consume HSPAR, CHPRAM, CFRM or CANCEL while negotiating/transmitting. */
int high_speed_service_command(HighSpeedService *service,
                               const UplinkCommand *command,
                               uint32_t now_ms);

/* Advance timeout/retry state. Returns one when an event is available. */
int high_speed_service_poll(HighSpeedService *service, uint32_t now_ms);

/* Start the peer's response timeout when the radio returns to RX, so our
   own downlink airtime does not consume the 10 s / 5 s listening window. */
void high_speed_service_control_sent(HighSpeedService *service,
                                     HighSpeedEvent event, uint32_t now_ms);

HighSpeedEvent high_speed_service_take_event(HighSpeedService *service);
HighSpeedState high_speed_service_state(const HighSpeedService *service);

/* Mark the requested image transfer complete or cancelled. */
void high_speed_service_transfer_done(HighSpeedService *service,
                                      uint8_t cancelled);

/* Build the ASCII control message sent by the normal GMSK downlink. */
int high_speed_service_event_text(const HighSpeedService *service,
                                  HighSpeedEvent event,
                                  char *out, uint32_t capacity);

#endif
