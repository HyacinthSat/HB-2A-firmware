#ifndef HB_I2CDEV_H
#define HB_I2CDEV_H
#include <stddef.h>
#include <stdint.h>
#include "esp_err.h"
typedef int i2c_port_t;
typedef int gpio_num_t;
typedef struct {
    i2c_port_t port;
    uint8_t addr;
    struct { gpio_num_t sda_io_num, scl_io_num; } cfg;
} i2c_dev_t;
/* Bare-metal foreground only. Never call these APIs from an ISR. */
#define I2C_DEV_TAKE_MUTEX(dev) ((void)(dev))
#define I2C_DEV_GIVE_MUTEX(dev) ((void)(dev))
#define I2C_DEV_CHECK(dev, call) do { int result_ = (call); if (result_) return result_; } while (0)
static inline int i2c_dev_create_mutex(i2c_dev_t *dev) { (void)dev; return ESP_OK; }
static inline int i2c_dev_delete_mutex(i2c_dev_t *dev) { (void)dev; return ESP_OK; }
esp_err_t i2c_dev_read_reg(const i2c_dev_t *dev, uint8_t reg, void *data, size_t size);
esp_err_t i2c_dev_write_reg(const i2c_dev_t *dev, uint8_t reg, const void *data, size_t size);
#endif
