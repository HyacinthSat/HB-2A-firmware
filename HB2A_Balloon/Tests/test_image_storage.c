#include "image_storage.h"
#include "main.h"

#include <assert.h>
#include <stdio.h>
#include <string.h>

#define FLASH_BYTES (IMAGE_STORAGE_SLOT_SIZE * IMAGE_STORAGE_SLOT_COUNT)

GPIO_TypeDef mock_gpio;
SPI_HandleTypeDef hspi2;
static uint8_t flash[FLASH_BYTES];
static uint8_t selected;
static uint8_t write_enabled;
static uint8_t read_pending;
static uint32_t read_address;
static uint32_t tick;
static unsigned erase_count;

void test_flash_reset(void)
{
  memset(flash, 0xFF, sizeof(flash));
  selected = 0U;
  write_enabled = 0U;
  read_pending = 0U;
  read_address = 0U;
  tick = 0U;
  erase_count = 0U;
}

void test_flash_corrupt(uint32_t address)
{
  assert(address < sizeof(flash));
  flash[address] ^= 0x01U;
}

static uint32_t address_from_command(const uint8_t *data)
{
  return ((uint32_t)data[1] << 16U) | ((uint32_t)data[2] << 8U) | data[3];
}

void HAL_GPIO_WritePin(GPIO_TypeDef *port, uint32_t pin, GPIO_PinState state)
{
  (void)port;
  (void)pin;
  selected = state == GPIO_PIN_RESET ? 1U : 0U;
  if (selected == 0U) read_pending = 0U;
}

uint32_t HAL_GetTick(void) { return tick; }
void HAL_Delay(uint32_t milliseconds) { tick += milliseconds; }

HAL_StatusTypeDef HAL_SPI_Transmit(SPI_HandleTypeDef *handle, uint8_t *data,
                                   uint16_t length, uint32_t timeout)
{
  uint32_t address;
  (void)handle;
  (void)timeout;
  assert(selected != 0U && length != 0U);
  if (data[0] == 0x06U && length == 1U) {
    write_enabled = 1U;
    return HAL_OK;
  }
  if (data[0] == 0x03U && length == 4U) {
    read_pending = 1U;
    read_address = address_from_command(data);
    return HAL_OK;
  }
  if (data[0] == 0x20U && length == 4U) {
    address = address_from_command(data) & ~(4096UL - 1UL);
    assert(write_enabled != 0U && address + 4096U <= sizeof(flash));
    memset(flash + address, 0xFF, 4096U);
    ++erase_count;
    write_enabled = 0U;
    return HAL_OK;
  }
  if (data[0] == 0x02U && length == 4U) {
    read_pending = 2U;
    read_address = address_from_command(data);
    return HAL_OK;
  }
  if (read_pending == 2U) {
    assert(write_enabled != 0U && read_address + length <= sizeof(flash));
    memcpy(flash + read_address, data, length);
    read_address += length;
    write_enabled = 0U;
    return HAL_OK;
  }
  assert(!"unexpected SPI transmit");
  return HAL_ERROR;
}

HAL_StatusTypeDef HAL_SPI_Receive(SPI_HandleTypeDef *handle, uint8_t *data,
                                  uint16_t length, uint32_t timeout)
{
  (void)handle;
  (void)timeout;
  assert(selected != 0U && read_pending == 1U &&
         read_address + length <= sizeof(flash));
  memcpy(data, flash + read_address, length);
  read_address += length;
  return HAL_OK;
}

HAL_StatusTypeDef HAL_SPI_TransmitReceive(SPI_HandleTypeDef *handle,
                                          uint8_t *tx, uint8_t *rx,
                                          uint16_t length, uint32_t timeout)
{
  (void)handle;
  (void)timeout;
  assert(selected != 0U && length == 2U && tx[0] == 0x05U);
  rx[0] = 0U;
  rx[1] = 0U;
  return HAL_OK;
}

int main(void)
{
  static const uint8_t first[] = {0xFFU, 0xD8U, 0x10U, 0x20U, 0x30U, 0xFFU};
  static const uint8_t second[] = {0xFFU, 0xD8U, 0x40U, 0x50U, 0x60U, 0xFFU};
  ImageStorage storage;

  test_flash_reset();
  assert(image_storage_init(&storage) == 0);
  assert(storage.has_image == 0U);
  assert(image_storage_store(&storage, 0U, first,
                             IMAGE_STORAGE_SLOT_SIZE) == -1);

  assert(image_storage_store(&storage, 3U, first, sizeof(first)) == 0);
  assert(erase_count == 1U);
  assert(image_storage_store(&storage, 4U, second, sizeof(second)) == 0);
  test_flash_corrupt(4U * IMAGE_STORAGE_SLOT_SIZE + IMAGE_STORAGE_HEADER_BYTES);
  assert(image_storage_init(&storage) == 0);
  assert(storage.has_image != 0U && image_storage_latest(&storage) == 3U);
  assert(image_storage_contains(&storage, 3U) != 0);
  assert(image_storage_contains(&storage, 4U) == 0);

  test_flash_reset();
  /* A plausible but uncommitted header with no valid payload must be ignored. */
  {
    uint8_t header[IMAGE_STORAGE_HEADER_BYTES] = {0x41U, 0x32U, 0x42U, 0x48,
                                                   99U, 0U, 0U, 0U,
                                                   4U, 0U, 0U, 0U};
    memcpy(flash, header, sizeof(header));
  }
  assert(image_storage_init(&storage) == 0);
  assert(storage.has_image == 0U);

  puts("image storage recovery tests passed");
  return 0;
}
