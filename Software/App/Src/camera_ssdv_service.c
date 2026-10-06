#include "camera_ssdv_service.h"

#include "aprs.h"
#include "app_config.h"
#include "dcmi.h"
#include "main.h"
#include "ssdv.h"

#include <stdio.h>
#include <string.h>

#define SSDV_BASE91_CAPACITY 300U
#define SSDV_SPLIT_CAPACITY  256U
#define SSDV_PART_GAP_MS     3000U

static ssdv_t encoder;
static uint8_t packet[APP_SSDV_PACKET_SIZE];
static uint8_t feed[APP_SSDV_FEED_SIZE];
static char encoded[SSDV_BASE91_CAPACITY];
static char left[140U];
static char right[140U];
static uint8_t xor_data[140U];
static char aprs_payload[SSDV_SPLIT_CAPACITY];
static char callsign[] = APP_WSPR_CALL;
static uint8_t ssdv_active;
static uint8_t ssdv_packet_ready;
static uint8_t ssdv_image_id;
static CameraPort ssdv_camera;
static uint8_t *ssdv_jpeg;
static uint32_t ssdv_jpeg_length;
static uint32_t ssdv_offset;
static unsigned ssdv_packets;
static uint8_t dcmi_initialized;

static void write_log(CameraSsdvLogFn log, const char *text)
{
  if (log != NULL && text != NULL) log(text);
}

static size_t base91_encode(const uint8_t *input, size_t length,
                            char *output, size_t capacity)
{
  static const char alphabet[] =
      "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789"
      "!#$%&()*+,./:;<=>?@[]^_`{|}~\"";
  uint32_t buffer = 0U;
  unsigned bits = 0U;
  size_t written = 0U;

  if (input == NULL || output == NULL || capacity == 0U) return 0U;
  for (size_t i = 0U; i < length; ++i) {
    buffer |= (uint32_t)input[i] << bits;
    bits += 8U;
    if (bits > 13U) {
      uint32_t value = buffer & 8191U;
      unsigned take = value > 88U ? 13U : 14U;
      value = buffer & ((1UL << take) - 1UL);
      if (written + 2U >= capacity) return 0U;
      output[written++] = alphabet[value % 91U];
      output[written++] = alphabet[value / 91U];
      buffer >>= take;
      bits -= take;
    }
  }
  if (bits != 0U) {
    unsigned tail = (bits > 7U || buffer > 90U) ? 2U : 1U;
    if (written + tail >= capacity) return 0U;
    output[written++] = alphabet[buffer % 91U];
    if (tail == 2U) output[written++] = alphabet[buffer / 91U];
  }
  output[written] = 0;
  return written;
}

static size_t make_part(char kind, size_t encoded_length, uint8_t image_id,
                        uint16_t packet_id, size_t header_half,
                        const char *data, size_t data_length,
                        char *out, size_t capacity)
{
  int header;
  if (out == NULL || data == NULL || encoded_length > 0xFFFU ||
      header_half == 0U || header_half > 139U) return 0U;
  header = snprintf(out, capacity, "{{%c%02X%04X%03X%03X:", kind,
                    (unsigned)image_id, (unsigned)packet_id,
                    (unsigned)encoded_length, (unsigned)header_half);
  if (header != 16 || (size_t)header + data_length >= capacity) return 0U;
  memcpy(out + header, data, data_length);
  out[header + data_length] = 0;
  return (size_t)header + data_length;
}

static void delay_with_poll(uint32_t milliseconds, AprsServicePollFn poll,
                            void *context)
{
  while (milliseconds != 0U) {
    uint32_t step = milliseconds > 10U ? 10U : milliseconds;
    if (poll != NULL) poll(context);
    HAL_Delay(step);
    milliseconds -= step;
  }
}

static void poll_gnss(void *context)
{
  gnss_service_poll((GnssService *)context);
}

static int send_ssdv_packet(AprsService *radio, GnssService *gnss,
                            const uint8_t *ssdv_packet, uint8_t image_id,
                            CameraSsdvLogFn log)
{
  const GnssSolution *solution;
  size_t encoded_length;
  size_t half;
  uint16_t packet_id;
  char parts[3U] = {'I', 'J', 'K'};
  AprsServicePollFn poll = poll_gnss;

  if (radio == NULL || gnss == NULL || ssdv_packet == NULL) return -1;
  solution = gnss_service_solution(gnss);
  if (solution == NULL) return -2;
  packet_id = ((uint16_t)ssdv_packet[7] << 8U) | ssdv_packet[8];
  encoded_length = base91_encode(ssdv_packet + 1U, 219U,
                                 encoded, sizeof(encoded));
  half = (encoded_length + 1U) / 2U;
  if (encoded_length == 0U || half > sizeof(left)) return -3;
  memcpy(left, encoded, half);
  memcpy(right, encoded + half, encoded_length - half);
  if (encoded_length - half < half)
    memset(right + encoded_length - half, 'A', half - (encoded_length - half));
  for (size_t i = 0U; i < half; ++i)
    xor_data[i] = (uint8_t)left[i] ^ (uint8_t)right[i];

  for (unsigned part = 0U; part < 3U; ++part) {
    size_t payload_length;
    uint32_t frequency = APRS_SelectFrequencyHz(
        solution->latitude_e7, solution->longitude_e7,
        solution->position_valid, NULL);
    if (parts[part] == 'I') {
      payload_length = make_part('I', encoded_length, image_id, packet_id,
                                 half, left, half, aprs_payload,
                                 sizeof(aprs_payload));
    } else if (parts[part] == 'J') {
      payload_length = make_part('J', encoded_length, image_id, packet_id,
                                 half, right, half, aprs_payload,
                                 sizeof(aprs_payload));
    } else {
      size_t k_length = base91_encode(xor_data, half, encoded, sizeof(encoded));
      if (k_length == 0U) return -4;
      payload_length = make_part('K', encoded_length, image_id, packet_id,
                                 half, encoded, k_length, aprs_payload,
                                 sizeof(aprs_payload));
    }
    if (payload_length == 0U) return -4;
    if (aprs_service_send_text_with_poll(radio, frequency, aprs_payload,
                                         payload_length, poll, gnss) != 0)
      return -5;
    if (part != 2U) delay_with_poll(SSDV_PART_GAP_MS, poll, gnss);
  }
  (void)log;
  return 0;
}

static int send_image(AprsService *radio, GnssService *gnss,
                      const uint8_t *jpeg, uint32_t length, uint8_t image_id,
                      int8_t quality, CameraSsdvLogFn log)
{
  uint32_t offset = 0U;
  unsigned packets = 0U;
  int status;

  if (jpeg == NULL || length < 4U || length > APP_SSDV_MAX_JPEG_BYTES ||
      jpeg[0] != 0xFFU || jpeg[1] != 0xD8U) return -1;
  status = ssdv_enc_init(&encoder, SSDV_TYPE_NORMAL, callsign, image_id,
                         quality, APP_SSDV_PACKET_SIZE);
  if (status != SSDV_OK || ssdv_enc_set_buffer(&encoder, packet) != SSDV_OK)
    return -2;
  for (;;) {
    status = ssdv_enc_get_packet(&encoder);
    if (status == SSDV_FEED_ME) {
      uint32_t chunk = length - offset;
      if (chunk == 0U) return -3;
      if (chunk > sizeof(feed)) chunk = sizeof(feed);
      memcpy(feed, jpeg + offset, chunk);
      offset += chunk;
      if (ssdv_enc_feed(&encoder, feed, chunk) != SSDV_OK) return -4;
      continue;
    }
    if (status == SSDV_EOI) break;
    if (status != SSDV_OK) return -5;
    if (send_ssdv_packet(radio, gnss, packet, image_id, log) != 0) return -6;
    ++packets;
    if (packets == 1U || (packets % 10U) == 0U) {
      char line[80];
      (void)snprintf(line, sizeof(line), "SSDV image=%u packet=%u\r\n",
                     (unsigned)image_id, packets);
      write_log(log, line);
    }
  }
  return (int)packets;
}

static void ensure_dcmi(void)
{
  if (dcmi_initialized == 0U) {
    MX_DCMI_Init();
    dcmi_initialized = 1U;
  }
}

static int capture_jpeg(CameraPort port, uint8_t image_id,
                        uint8_t **jpeg, uint32_t *length,
                        CameraSsdvLogFn log)
{
  char line[96];
  int result;

  if (jpeg == NULL || length == NULL) return -1;
  ensure_dcmi();
  result = camera_init_resolution(port, CAMERA_RESOLUTION_256X192);
  if (result != 0) {
    camera_power_off();
    (void)snprintf(line, sizeof(line), "CAM %c init failed rc=%d\r\n",
                   port == CAMERA_A ? 'A' : 'B', result);
    write_log(log, line);
    return result;
  }
  for (unsigned i = 0U; i < APP_CAMERA_WARMUP_COUNT; ++i) {
    uint8_t *discard = NULL;
    uint32_t discard_length = 0U;
    (void)camera_capture(&discard, &discard_length);
    HAL_Delay(APP_CAMERA_WARMUP_INTERVAL_MS);
  }
  result = camera_capture(jpeg, length);
  camera_power_off();
  if (result != 0) return result;
  (void)snprintf(line, sizeof(line),
                 "CAM %c JPEG bytes=%lu image=%u size=256x192\r\n",
                 port == CAMERA_A ? 'A' : 'B', (unsigned long)*length,
                 (unsigned)image_id);
  write_log(log, line);
  return 0;
}

static int prepare_next_packet(void)
{
  int status;
  if (ssdv_packet_ready != 0U) return 0;
  for (;;) {
    status = ssdv_enc_get_packet(&encoder);
    if (status == SSDV_FEED_ME) {
      uint32_t chunk;
      if (ssdv_jpeg == NULL || ssdv_offset >= ssdv_jpeg_length) return -1;
      chunk = ssdv_jpeg_length - ssdv_offset;
      if (chunk > sizeof(feed)) chunk = sizeof(feed);
      memcpy(feed, ssdv_jpeg + ssdv_offset, chunk);
      ssdv_offset += chunk;
      if (ssdv_enc_feed(&encoder, feed, chunk) != SSDV_OK) return -2;
      continue;
    }
    if (status == SSDV_EOI) return 1;
    if (status != SSDV_OK) return -3;
    ssdv_packet_ready = 1U;
    return 0;
  }
}

int camera_ssdv_capture_start(AprsService *radio, GnssService *gnss,
                              CameraPort port, uint8_t image_id,
                              int8_t quality, CameraSsdvLogFn log)
{
  uint8_t *jpeg = NULL;
  uint32_t length = 0U;
  int result;

  if (radio == NULL || gnss == NULL || ssdv_active != 0U) return -1;
  result = capture_jpeg(port, image_id, &jpeg, &length, log);
  if (result != 0) return result;
  result = ssdv_enc_init(&encoder, SSDV_TYPE_NORMAL, callsign, image_id,
                         quality, APP_SSDV_PACKET_SIZE);
  if (result != SSDV_OK || ssdv_enc_set_buffer(&encoder, packet) != SSDV_OK)
    return -2;
  ssdv_active = 1U;
  ssdv_packet_ready = 0U;
  ssdv_image_id = image_id;
  ssdv_camera = port;
  ssdv_jpeg = jpeg;
  ssdv_jpeg_length = length;
  ssdv_offset = 0U;
  ssdv_packets = 0U;
  write_log(log, "SSDV image ready; scheduled groups begin after 06 APRS\r\n");
  return 0;
}

int camera_ssdv_send_next_group(AprsService *radio, GnssService *gnss,
                               CameraSsdvLogFn log)
{
  int result;
  char line[96];

  if (radio == NULL || gnss == NULL || ssdv_active == 0U) return 1;
  result = prepare_next_packet();
  if (result == 1) {
    ssdv_active = 0U;
    (void)snprintf(line, sizeof(line),
                   "SSDV image=%u complete packets=%u camera=%c\r\n",
                   (unsigned)ssdv_image_id, ssdv_packets,
                   ssdv_camera == CAMERA_A ? 'A' : 'B');
    write_log(log, line);
    return 1;
  }
  if (result != 0 || send_ssdv_packet(radio, gnss, packet, ssdv_image_id,
                                      log) != 0) {
    ssdv_active = 0U;
    write_log(log, "SSDV group failed; image aborted\r\n");
    return -1;
  }
  ++ssdv_packets;
  ssdv_packet_ready = 0U;
  (void)snprintf(line, sizeof(line), "SSDV image=%u group=%u camera=%c\r\n",
                 (unsigned)ssdv_image_id, ssdv_packets,
                 ssdv_camera == CAMERA_A ? 'A' : 'B');
  write_log(log, line);
  result = prepare_next_packet();
  if (result == 1) {
    ssdv_active = 0U;
    (void)snprintf(line, sizeof(line),
                   "SSDV image=%u complete packets=%u camera=%c\r\n",
                   (unsigned)ssdv_image_id, ssdv_packets,
                   ssdv_camera == CAMERA_A ? 'A' : 'B');
    write_log(log, line);
    return 1;
  }
  if (result != 0) {
    ssdv_active = 0U;
    write_log(log, "SSDV next group preparation failed\r\n");
    return -2;
  }
  return 0;
}

uint8_t camera_ssdv_active(void)
{
  return ssdv_active;
}

static int capture_send_one(AprsService *radio, GnssService *gnss,
                            CameraPort port, uint8_t image_id, int8_t quality,
                            CameraSsdvLogFn log)
{
  uint8_t *jpeg = NULL;
  uint32_t length = 0U;
  char line[96];
  int result = capture_jpeg(port, image_id, &jpeg, &length, log);
  if (result != 0) return result;
  result = send_image(radio, gnss, jpeg, length, image_id, quality, log);
  if (result >= 0) {
    (void)snprintf(line, sizeof(line), "SSDV image=%u complete packets=%d\r\n",
                   (unsigned)image_id, result);
    write_log(log, line);
  }
  return result;
}

int camera_ssdv_run_sequence(AprsService *radio, GnssService *gnss,
                             int8_t quality, CameraSsdvLogFn log)
{
  int first_result;
  int second_result;
  ensure_dcmi();
  write_log(log, "SSDV startup test: CAM_A capture/send\r\n");
  first_result = capture_send_one(radio, gnss, CAMERA_A, 0U, quality, log);
  if (first_result < 0) {
    write_log(log, "SSDV startup test: CAM_A failed; continuing with CAM_B\r\n");
  }
  write_log(log, "SSDV startup test: CAM_B capture/send\r\n");
  second_result = capture_send_one(radio, gnss, CAMERA_B, 1U, quality, log);
  if (second_result < 0) return second_result;
  return first_result < 0 ? first_result : 0;
}
