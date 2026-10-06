#include "image_storage.h"

#include "flash_selftest.h"
#include "main.h"
#include "spi.h"

#include <string.h>

#define STORAGE_MAGIC          0x48423241UL
#define STORAGE_HEADER_BYTES   IMAGE_STORAGE_HEADER_BYTES
#define FLASH_CMD_WRITE_ENABLE 0x06U
#define FLASH_CMD_READ_STATUS  0x05U
#define FLASH_CMD_PAGE_PROGRAM 0x02U
#define FLASH_CMD_READ         0x03U
#define FLASH_CMD_SECTOR_ERASE 0x20U
#define FLASH_SECTOR_SIZE      4096U
#define STORAGE_BASE_ADDRESS   0x000000UL

typedef struct {
  uint32_t magic;
  uint32_t sequence;
  uint32_t length;
  uint32_t crc32;
} StorageHeader;

static void select_flash(void)
{
  HAL_GPIO_WritePin(FLASH_NSS_GPIO_Port, FLASH_NSS_Pin, GPIO_PIN_RESET);
}

static void deselect_flash(void)
{
  HAL_GPIO_WritePin(FLASH_NSS_GPIO_Port, FLASH_NSS_Pin, GPIO_PIN_SET);
}

static int tx(const uint8_t *data, uint16_t length)
{
  HAL_StatusTypeDef status;
  select_flash();
  status = HAL_SPI_Transmit(&hspi2, (uint8_t *)data, length, 1000U);
  deselect_flash();
  return status == HAL_OK ? 0 : -1;
}

static int status(uint8_t *value)
{
  uint8_t command[2] = {FLASH_CMD_READ_STATUS, 0U};
  uint8_t response[2] = {0U};
  HAL_StatusTypeDef result;
  if (value == NULL) return -1;
  select_flash();
  result = HAL_SPI_TransmitReceive(&hspi2, command, response, 2U, 1000U);
  deselect_flash();
  if (result != HAL_OK) return -1;
  *value = response[1];
  return 0;
}

static int wait_ready(uint32_t timeout_ms)
{
  uint32_t start = HAL_GetTick();
  uint8_t value;
  do {
    if (status(&value) != 0) return -1;
    if ((value & 1U) == 0U) return 0;
  } while ((HAL_GetTick() - start) < timeout_ms);
  return -2;
}

static int write_enable(void)
{
  const uint8_t command = FLASH_CMD_WRITE_ENABLE;
  return tx(&command, 1U);
}

static int read_bytes(uint32_t address, uint8_t *data, uint32_t length)
{
  uint8_t command[4] = {FLASH_CMD_READ, (uint8_t)(address >> 16),
                        (uint8_t)(address >> 8), (uint8_t)address};
  HAL_StatusTypeDef result;
  if (data == NULL || length == 0U || length > 65535U) return -1;
  select_flash();
  result = HAL_SPI_Transmit(&hspi2, command, sizeof(command), 1000U);
  if (result == HAL_OK)
    result = HAL_SPI_Receive(&hspi2, data, (uint16_t)length, 1000U);
  deselect_flash();
  return result == HAL_OK ? 0 : -1;
}

static int erase_sector(uint32_t address)
{
  uint8_t command[4] = {FLASH_CMD_SECTOR_ERASE, (uint8_t)(address >> 16),
                        (uint8_t)(address >> 8), (uint8_t)address};
  if (write_enable() != 0 || tx(command, sizeof(command)) != 0) return -1;
  return wait_ready(3000U);
}

static int program_page(uint32_t address, const uint8_t *data, uint16_t length)
{
  uint8_t command[4] = {FLASH_CMD_PAGE_PROGRAM, (uint8_t)(address >> 16),
                        (uint8_t)(address >> 8), (uint8_t)address};
  if (data == NULL || length == 0U || length > 256U) return -1;
  if (write_enable() != 0) return -1;
  select_flash();
  if (HAL_SPI_Transmit(&hspi2, command, sizeof(command), 1000U) != HAL_OK ||
      HAL_SPI_Transmit(&hspi2, (uint8_t *)data, length, 1000U) != HAL_OK) {
    deselect_flash();
    return -1;
  }
  deselect_flash();
  return wait_ready(1000U);
}

static uint32_t crc32_update(uint32_t crc, const uint8_t *data, uint32_t length)
{
  while (length-- != 0U) {
    crc ^= *data++;
    for (unsigned bit = 0U; bit < 8U; ++bit)
      crc = (crc & 1U) != 0U ? (crc >> 1U) ^ 0xEDB88320UL : crc >> 1U;
  }
  return crc;
}

static uint32_t slot_address(uint32_t slot)
{
  return STORAGE_BASE_ADDRESS + slot * IMAGE_STORAGE_SLOT_SIZE;
}

static uint32_t image_address(uint32_t sequence)
{
  return slot_address(sequence % IMAGE_STORAGE_SLOT_COUNT);
}

static int header_valid(const StorageHeader *header)
{
  return header != NULL && header->magic == STORAGE_MAGIC &&
         header->length != 0U && header->length <= IMAGE_STORAGE_MAX_JPEG_BYTES;
}

static int read_slot_header(uint32_t slot, StorageHeader *header)
{
  if (header == NULL || slot >= IMAGE_STORAGE_SLOT_COUNT) return -1;
  return read_bytes(slot_address(slot), (uint8_t *)header, sizeof(*header));
}

static int read_header(uint32_t sequence, StorageHeader *header)
{
  if (header == NULL || read_slot_header(sequence % IMAGE_STORAGE_SLOT_COUNT,
                                          header) != 0 ||
      !header_valid(header) || header->sequence != sequence)
    return -2;
  return 0;
}

static int image_crc_valid(uint32_t slot, const StorageHeader *header)
{
  uint8_t buffer[256U];
  uint32_t crc = 0xFFFFFFFFUL;
  uint32_t offset = 0U;
  if (header == NULL || slot >= IMAGE_STORAGE_SLOT_COUNT) return 0;
  while (offset < header->length) {
    uint32_t chunk = header->length - offset;
    if (chunk > sizeof(buffer)) chunk = sizeof(buffer);
    if (read_bytes(slot_address(slot) + STORAGE_HEADER_BYTES + offset,
                   buffer, chunk) != 0) return 0;
    crc = crc32_update(crc, buffer, chunk);
    offset += chunk;
  }
  return ~crc == header->crc32;
}

int image_storage_init(ImageStorage *storage)
{
  StorageHeader headers[IMAGE_STORAGE_SLOT_COUNT];
  uint8_t candidates[IMAGE_STORAGE_SLOT_COUNT] = {0U};
  if (storage == NULL) return -1;
  memset(storage, 0, sizeof(*storage));
  storage->initialized = 1U;
  /* The slot header is committed after the JPEG payload. Recovering the
     greatest valid sequence makes REQF work across MCU resets while a
     partially written slot remains invisible. */
  for (uint32_t slot = 0U; slot < IMAGE_STORAGE_SLOT_COUNT; ++slot) {
    if (read_slot_header(slot, &headers[slot]) == 0 &&
        header_valid(&headers[slot])) candidates[slot] = 1U;
  }
  /* Validate candidates newest first and stop at the first complete image.
     A failed latest write therefore falls back to the previous image. */
  for (;;) {
    uint32_t best = IMAGE_STORAGE_SLOT_COUNT;
    for (uint32_t slot = 0U; slot < IMAGE_STORAGE_SLOT_COUNT; ++slot) {
      if (candidates[slot] != 0U &&
          (best == IMAGE_STORAGE_SLOT_COUNT ||
           headers[slot].sequence > headers[best].sequence)) best = slot;
    }
    if (best == IMAGE_STORAGE_SLOT_COUNT) break;
    candidates[best] = 0U;
    if (image_crc_valid(best, &headers[best])) {
      storage->latest_sequence = headers[best].sequence;
      storage->has_image = 1U;
      break;
    }
  }
  return 0;
}

int image_storage_store(ImageStorage *storage, uint32_t sequence,
                        const uint8_t *jpeg, uint32_t length)
{
  StorageHeader header;
  uint32_t address;
  uint32_t erase_bytes;
  if (storage == NULL || storage->initialized == 0U || jpeg == NULL ||
      length < 4U || length > IMAGE_STORAGE_MAX_JPEG_BYTES ||
      jpeg[0] != 0xFFU || jpeg[1] != 0xD8U) return -1;
  address = image_address(sequence);
  /* Old bytes beyond the new length are unreachable through the committed
     header. Erase only the sectors that will be rewritten so CAPTR does not
     spend tens of seconds erasing an entire 512 KiB slot before RF TX. */
  erase_bytes = STORAGE_HEADER_BYTES + length;
  for (uint32_t offset = 0U; offset < erase_bytes;
       offset += FLASH_SECTOR_SIZE)
    if (erase_sector(address + offset) != 0) return -2;
  for (uint32_t offset = 0U; offset < length;) {
    uint16_t chunk = (uint16_t)(256U - ((address + STORAGE_HEADER_BYTES + offset) & 255U));
    if (chunk > length - offset) chunk = (uint16_t)(length - offset);
    if (program_page(address + STORAGE_HEADER_BYTES + offset, jpeg + offset,
                     chunk) != 0) return -3;
    offset += chunk;
  }
  header.magic = STORAGE_MAGIC;
  header.sequence = sequence;
  header.length = length;
  header.crc32 = ~crc32_update(0xFFFFFFFFUL, jpeg, length);
  {
    const uint8_t *raw = (const uint8_t *)&header;
    for (uint32_t offset = 0U; offset < sizeof(header);) {
      uint16_t chunk = (uint16_t)(256U - ((address + offset) & 255U));
      if (chunk > sizeof(header) - offset) chunk = (uint16_t)(sizeof(header) - offset);
      if (program_page(address + offset, raw + offset, chunk) != 0) return -4;
      offset += chunk;
    }
  }
  storage->latest_sequence = sequence;
  storage->has_image = 1U;
  return 0;
}

int image_storage_contains(const ImageStorage *storage, uint32_t sequence)
{
  StorageHeader header;
  return storage != NULL && storage->initialized != 0U &&
         storage->has_image != 0U &&
         read_header(sequence, &header) == 0 &&
         image_crc_valid(sequence % IMAGE_STORAGE_SLOT_COUNT, &header) != 0 ?
      1 : 0;
}

int image_storage_length(const ImageStorage *storage, uint32_t sequence,
                         uint32_t *length)
{
  StorageHeader header;
  if (storage == NULL || storage->initialized == 0U ||
      storage->has_image == 0U || length == NULL ||
      read_header(sequence, &header) != 0 ||
      image_crc_valid(sequence % IMAGE_STORAGE_SLOT_COUNT, &header) == 0)
    return -1;
  *length = header.length;
  return 0;
}

int image_storage_read(const ImageStorage *storage, uint32_t sequence,
                       uint32_t offset, uint8_t *data, uint32_t length)
{
  StorageHeader header;
  if (storage == NULL || storage->initialized == 0U ||
      storage->has_image == 0U || data == NULL ||
      length == 0U || read_header(sequence, &header) != 0 ||
      offset > header.length || length > header.length - offset ||
      length > 65535U) return -1;
  return read_bytes(image_address(sequence) + STORAGE_HEADER_BYTES + offset,
                    data, length);
}

uint32_t image_storage_latest(const ImageStorage *storage)
{
  return storage == NULL ? 0U : storage->latest_sequence;
}
