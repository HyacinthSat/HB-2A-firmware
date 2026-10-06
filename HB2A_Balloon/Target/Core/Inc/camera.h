#ifndef CAMERA_H
#define CAMERA_H
#include <stdint.h>

/* GPDMA normal transfers are limited to 0xFFFF bytes in HAL_DCMI_Start_DMA.
   Keep the JPEG buffer just below that limit; a VGA OV5640 JPEG at quality 24
   normally fits, and an over-size frame is reported as a capture failure. */
#define CAMERA_JPEG_BUFFER_SIZE (64U * 1024U - 4U)

/* OV5640 JPEG presets used by the balloon. */
#define CAMERA_RESOLUTION_128X96  0x05U
#define CAMERA_RESOLUTION_256X192 0x06U

typedef enum {
  CAMERA_A = 0, /* CAM_A1, ground-facing */
  CAMERA_B = 1  /* CAM_B1, sky-facing */
} CameraPort;

typedef void (*CameraCapturePollFn)(void *context);

int camera_init(CameraPort port);
int camera_init_resolution(CameraPort port, uint32_t resolution);
/* SSDV-aligned existing presets only; arbitrary sensor timing is not used. */
int camera_init_dimensions(CameraPort port, uint16_t width, uint16_t height);
/* Read-only ISP/AWB/AEC snapshot, while the selected sensor is powered. */
void camera_log_state(const char *phase);
/* Stops sensor power/clock; the captured JPEG in RAM remains valid. */
void camera_power_off(void);
int camera_capture(uint8_t **jpeg, uint32_t *length);
int camera_capture_with_poll(uint8_t **jpeg, uint32_t *length,
                             CameraCapturePollFn poll, void *context);
int camera_send_jpeg(const uint8_t *jpeg, uint32_t length);

#endif
