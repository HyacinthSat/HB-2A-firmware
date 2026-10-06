#ifndef HB2A_FLASH_SELFTEST_H
#define HB2A_FLASH_SELFTEST_H

#include <stdint.h>

/* Reserved end-of-device sector; storage_service will own this region later. */
#define FLASH_SELFTEST_ADDRESS 0x00FF0000UL

enum {
  FLASH_SELFTEST_SPI_ERROR = -1,
  FLASH_SELFTEST_ID_ERROR = -2,
  FLASH_SELFTEST_WRITE_ERROR = -3,
  FLASH_SELFTEST_READ_ERROR = -4,
  FLASH_SELFTEST_ERASE_ERROR = -5,
  FLASH_SELFTEST_TIMEOUT = -6
};

int flash_selftest_run(uint32_t address);

#endif
