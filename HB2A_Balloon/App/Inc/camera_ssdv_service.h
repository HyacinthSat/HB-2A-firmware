#ifndef HB2A_CAMERA_SSDV_SERVICE_H
#define HB2A_CAMERA_SSDV_SERVICE_H

#include "aprs_service.h"
#include "camera.h"
#include "gnss_service.h"
#include "image_storage.h"

#include <stddef.h>
#include <stdint.h>

typedef void (*CameraSsdvLogFn)(const char *text);
typedef int (*CameraSsdvPacketFn)(const uint8_t *packet, size_t length,
                                  void *context);

void camera_ssdv_set_storage(ImageStorage *storage);

/* Legacy one-shot validation helper. */
int camera_ssdv_run_sequence(AprsService *radio, GnssService *gnss,
                             int8_t quality, CameraSsdvLogFn log);

/* Capture one camera and keep its SSDV encoder ready for scheduled groups. */
int camera_ssdv_capture_start(AprsService *radio, GnssService *gnss,
                              CameraPort port, uint8_t image_id,
                              int8_t quality, CameraSsdvLogFn log);
int camera_ssdv_capture_start_sequence(AprsService *radio, GnssService *gnss,
                                       CameraPort port, uint32_t image_sequence,
                                       int8_t quality, CameraSsdvLogFn log);

/* High-speed SSDV source. The callback is called once per raw 256-byte SSDV
   packet and runs synchronously while the encoder owns its working buffer. */
int camera_ssdv_run_high_speed_capture(
    GnssService *gnss, CameraPort port, uint32_t image_sequence,
    uint16_t width, uint16_t height, int8_t quality,
    CameraSsdvPacketFn send, void *context,
    CameraSsdvLogFn log);
int camera_ssdv_run_high_speed_file(
    GnssService *gnss, const ImageStorage *storage, uint32_t image_sequence,
    int8_t quality, CameraSsdvPacketFn send, void *context,
    CameraSsdvLogFn log);

/* Send exactly one I/J/K group. Returns 1 when the image is complete. */
int camera_ssdv_send_next_group(AprsService *radio, GnssService *gnss,
                               CameraSsdvLogFn log);

uint8_t camera_ssdv_active(void);

#endif
