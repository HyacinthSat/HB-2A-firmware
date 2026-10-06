#ifndef HB2A_CAMERA_SSDV_SERVICE_H
#define HB2A_CAMERA_SSDV_SERVICE_H

#include "aprs_service.h"
#include "camera.h"
#include "gnss_service.h"

#include <stdint.h>

typedef void (*CameraSsdvLogFn)(const char *text);

/* Legacy one-shot validation helper. */
int camera_ssdv_run_sequence(AprsService *radio, GnssService *gnss,
                             int8_t quality, CameraSsdvLogFn log);

/* Capture one camera and keep its SSDV encoder ready for scheduled groups. */
int camera_ssdv_capture_start(AprsService *radio, GnssService *gnss,
                              CameraPort port, uint8_t image_id,
                              int8_t quality, CameraSsdvLogFn log);

/* Send exactly one I/J/K group. Returns 1 when the image is complete. */
int camera_ssdv_send_next_group(AprsService *radio, GnssService *gnss,
                               CameraSsdvLogFn log);

uint8_t camera_ssdv_active(void);

#endif
