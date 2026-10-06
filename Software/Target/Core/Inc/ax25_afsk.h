#ifndef AX25_AFSK_H
#define AX25_AFSK_H
#include <stddef.h>
#include <stdint.h>

#define AX25_MAX_INFO 256U
#define AX25_MAX_FRAME (23U + AX25_MAX_INFO + 2U)
#define AFSK_SAMPLE_HZ 26400UL
#define AFSK_SAMPLES_PER_BIT 22U
#define AFSK_MAX_BITS 4096U
#define AFSK_PREAMBLE_FLAGS 45U /* 300 ms at 1200 baud */
#define AFSK_TAIL_FLAGS 3U

typedef struct {
  uint8_t tones[AFSK_MAX_BITS]; /* NRZI tone selection, one byte per bit */
  uint16_t count;
  uint16_t index;
  uint8_t sample;
  uint8_t phase;
} AFSK_Stream;

uint16_t AX25_Crc(const uint8_t *data, size_t length);
int AX25_Build(const char *source, const char *destination,
               const uint8_t *info, size_t length, uint8_t *out, size_t capacity);
/* Build a UI frame with one digipeater/path address. The path address is
   emitted as supplied and marked H=1 after the HB-2A validation. */
int AX25_BuildPath(const char *source, const char *destination,
                   const char *path, const uint8_t *info, size_t length,
                   uint8_t *out, size_t capacity);
int AFSK_Prepare(AFSK_Stream *stream, const uint8_t *frame, size_t length);
/* 0/1 are DIO2 levels; -1 indicates all samples have been emitted. */
int AFSK_NextSample(AFSK_Stream *stream);
uint32_t AFSK_DurationMs(const AFSK_Stream *stream);
#endif
