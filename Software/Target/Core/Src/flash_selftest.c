#include "flash_selftest.h"

#include "main.h"
#include "spi.h"

#include <string.h>

#define FLASH_CMD_WRITE_ENABLE 0x06U
#define FLASH_CMD_READ_STATUS 0x05U
#define FLASH_CMD_PAGE_PROGRAM 0x02U
#define FLASH_CMD_READ 0x03U
#define FLASH_CMD_SECTOR_ERASE 0x20U
#define FLASH_CMD_READ_ID 0x9FU
#define FLASH_SECTOR_SIZE 4096U

static void flash_select(void)
{
  HAL_GPIO_WritePin(FLASH_NSS_GPIO_Port, FLASH_NSS_Pin, GPIO_PIN_RESET);
}

static void flash_deselect(void)
{
  HAL_GPIO_WritePin(FLASH_NSS_GPIO_Port, FLASH_NSS_Pin, GPIO_PIN_SET);
}

static int flash_command(const uint8_t *command, uint16_t length)
{
  HAL_StatusTypeDef status;
  flash_select();
  status = HAL_SPI_Transmit(&hspi2, (uint8_t *)command, length, 100U);
  flash_deselect();
  return status == HAL_OK ? 0 : FLASH_SELFTEST_SPI_ERROR;
}

static int flash_write_enable(void)
{
  const uint8_t command = FLASH_CMD_WRITE_ENABLE;
  return flash_command(&command, 1U);
}

static int flash_status(uint8_t *status_byte)
{
  uint8_t tx[2] = {FLASH_CMD_READ_STATUS, 0U};
  uint8_t rx[2] = {0U};
  HAL_StatusTypeDef status;
  if (status_byte == NULL) return FLASH_SELFTEST_READ_ERROR;
  flash_select();
  status = HAL_SPI_TransmitReceive(&hspi2, tx, rx, 2U, 100U);
  flash_deselect();
  if (status != HAL_OK) return FLASH_SELFTEST_SPI_ERROR;
  *status_byte = rx[1];
  return 0;
}

static int flash_wait_ready(uint32_t timeout_ms)
{
  uint32_t start = HAL_GetTick();
  uint8_t status_byte;
  do {
    int result = flash_status(&status_byte);
    if (result != 0) return result;
    if ((status_byte & 1U) == 0U) return 0;
  } while ((HAL_GetTick() - start) < timeout_ms);
  return FLASH_SELFTEST_TIMEOUT;
}

static int flash_read_id(void)
{
  uint8_t tx[4] = {FLASH_CMD_READ_ID, 0U, 0U, 0U};
  uint8_t rx[4] = {0U};
  HAL_StatusTypeDef status;
  flash_select();
  status = HAL_SPI_TransmitReceive(&hspi2, tx, rx, 4U, 100U);
  flash_deselect();
  if (status != HAL_OK) return FLASH_SELFTEST_SPI_ERROR;
  if (rx[1] != 0x9DU || rx[2] != 0x60U || rx[3] != 0x18U)
    return FLASH_SELFTEST_ID_ERROR;
  return 0;
}

static int flash_erase(uint32_t address)
{
  uint8_t command[4] = {FLASH_CMD_SECTOR_ERASE,
                        (uint8_t)(address >> 16),
                        (uint8_t)(address >> 8),
                        (uint8_t)address};
  int result = flash_write_enable();
  if (result != 0) return result;
  result = flash_command(command, sizeof(command));
  if (result != 0) return result;
  return flash_wait_ready(3000U);
}

static int flash_program(uint32_t address, const uint8_t *data, uint16_t length)
{
  uint8_t command[4] = {FLASH_CMD_PAGE_PROGRAM,
                        (uint8_t)(address >> 16),
                        (uint8_t)(address >> 8),
                        (uint8_t)address};
  int result;
  if (data == NULL || length == 0U || length > 256U) return FLASH_SELFTEST_WRITE_ERROR;
  result = flash_write_enable();
  if (result != 0) return result;
  flash_select();
  if (HAL_SPI_Transmit(&hspi2, command, sizeof(command), 100U) != HAL_OK ||
      HAL_SPI_Transmit(&hspi2, (uint8_t *)data, length, 100U) != HAL_OK) {
    flash_deselect();
    return FLASH_SELFTEST_SPI_ERROR;
  }
  flash_deselect();
  return flash_wait_ready(1000U);
}

static int flash_read(uint32_t address, uint8_t *data, uint16_t length)
{
  uint8_t command[4] = {FLASH_CMD_READ,
                        (uint8_t)(address >> 16),
                        (uint8_t)(address >> 8),
                        (uint8_t)address};
  HAL_StatusTypeDef status;
  if (data == NULL || length == 0U) return FLASH_SELFTEST_READ_ERROR;
  flash_select();
  status = HAL_SPI_Transmit(&hspi2, command, sizeof(command), 100U);
  if (status == HAL_OK)
    status = HAL_SPI_Receive(&hspi2, data, length, 100U);
  flash_deselect();
  return status == HAL_OK ? 0 : FLASH_SELFTEST_SPI_ERROR;
}

int flash_selftest_run(uint32_t address)
{
  static const uint8_t pattern[16] =
      {0x48U, 0x42U, 0x2DU, 0x32U, 0x41U, 0x2DU, 0x46U, 0x53U,
       0x54U, 0x30U, 0x01U, 0x5AU, 0xA5U, 0xC3U, 0x3CU, 0x96U};
  uint8_t readback[sizeof(pattern)];
  int result = flash_read_id();
  if (result != 0) return result;
  if ((address & (FLASH_SECTOR_SIZE - 1U)) != 0U) return FLASH_SELFTEST_ERASE_ERROR;
  result = flash_erase(address);
  if (result != 0) return result == FLASH_SELFTEST_TIMEOUT ? result : FLASH_SELFTEST_ERASE_ERROR;
  result = flash_program(address, pattern, sizeof(pattern));
  if (result != 0) return result == FLASH_SELFTEST_TIMEOUT ? result : FLASH_SELFTEST_WRITE_ERROR;
  result = flash_read(address, readback, sizeof(readback));
  if (result != 0) return FLASH_SELFTEST_READ_ERROR;
  return memcmp(pattern, readback, sizeof(pattern)) == 0 ? 0 : FLASH_SELFTEST_READ_ERROR;
}
