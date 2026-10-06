#ifndef HB2A_IMAGE_STORAGE_H
#define HB2A_IMAGE_STORAGE_H

#include <stdint.h>

#define IMAGE_STORAGE_SLOT_SIZE      (512UL * 1024UL)
#define IMAGE_STORAGE_SLOT_COUNT     31U
#define IMAGE_STORAGE_HEADER_BYTES   16U
/* The header is committed after the payload, so it must fit in the slot. */
#define IMAGE_STORAGE_MAX_JPEG_BYTES \
  (IMAGE_STORAGE_SLOT_SIZE - IMAGE_STORAGE_HEADER_BYTES)

typedef struct {
  uint32_t latest_sequence;
  uint8_t initialized;
  uint8_t has_image;
} ImageStorage;

int image_storage_init(ImageStorage *storage);
int image_storage_store(ImageStorage *storage, uint32_t sequence,
                        const uint8_t *jpeg, uint32_t length);
int image_storage_read(const ImageStorage *storage, uint32_t sequence,
                       uint32_t offset, uint8_t *data, uint32_t length);
int image_storage_contains(const ImageStorage *storage, uint32_t sequence);
int image_storage_length(const ImageStorage *storage, uint32_t sequence,
                         uint32_t *length);
uint32_t image_storage_latest(const ImageStorage *storage);

#endif
