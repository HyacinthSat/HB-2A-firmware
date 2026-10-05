#include "sensors.h"
#include "i2c.h"
#include "bme280.h"
#include "ina3221.h"
#include <string.h>

#ifndef BME280_64BIT_ENABLE
#error "Build Bosch driver and this adapter with BME280_64BIT_ENABLE"
#endif
#ifndef BME280_PRESSURE_UNCLAMPED
#error "Enable BME280_PRESSURE_UNCLAMPED for marked pressure telemetry"
#endif
#define I2C_TIMEOUT_MS 10U
#define INA_ADDRESS 0x40U
#define BME_ADDRESS 0x76U
#define SAMPLE_MAX_AGE_MS 3000U
static ina3221_t ina;
static struct bme280_dev bme;
static SensorSample sample;
static uint8_t ina_ready, bme_ready;
static uint32_t next_poll;

esp_err_t i2c_dev_read_reg(const i2c_dev_t *dev, uint8_t reg, void *data, size_t size)
{
  if (!dev || dev->port != 3 || !data || !size || size > 65535U) return ESP_ERR_INVALID_ARG;
  return HAL_I2C_Mem_Read(&hi2c3, (uint16_t)(dev->addr << 1), reg, I2C_MEMADD_SIZE_8BIT,
                         data, (uint16_t)size, I2C_TIMEOUT_MS) == HAL_OK ? ESP_OK : ESP_FAIL;
}
esp_err_t i2c_dev_write_reg(const i2c_dev_t *dev, uint8_t reg, const void *data, size_t size)
{
  if (!dev || dev->port != 3 || !data || !size || size > 65535U) return ESP_ERR_INVALID_ARG;
  return HAL_I2C_Mem_Write(&hi2c3, (uint16_t)(dev->addr << 1), reg, I2C_MEMADD_SIZE_8BIT,
                          (uint8_t *)data, (uint16_t)size, I2C_TIMEOUT_MS) == HAL_OK ? ESP_OK : ESP_FAIL;
}
static int8_t bme_read(uint8_t reg, uint8_t *data, uint32_t size, void *ctx)
{
  (void)ctx;
  if (!size || size > 65535U) return -1;
  return HAL_I2C_Mem_Read(&hi2c3, BME_ADDRESS << 1, reg, I2C_MEMADD_SIZE_8BIT,
                         data, (uint16_t)size, I2C_TIMEOUT_MS) == HAL_OK ? 0 : -1;
}
static int8_t bme_write(uint8_t reg, const uint8_t *data, uint32_t size, void *ctx)
{
  (void)ctx;
  if (!size || size > 65535U) return -1;
  return HAL_I2C_Mem_Write(&hi2c3, BME_ADDRESS << 1, reg, I2C_MEMADD_SIZE_8BIT,
                          (uint8_t *)data, (uint16_t)size, I2C_TIMEOUT_MS) == HAL_OK ? 0 : -1;
}
static void delay_us(uint32_t us, void *ctx)
{
  (void)ctx;
  HAL_Delay((us + 999U) / 1000U); /* Bosch requires minimum delay; init only. */
}
static int ina_word(uint8_t reg, uint16_t *value)
{
  uint8_t bytes[2];
  if (i2c_dev_read_reg(&ina.i2c_dev, reg, bytes, 2)) return -1;
  *value = ((uint16_t)bytes[0] << 8) | bytes[1];
  return 0;
}
void Sensors_Init(void)
{
  memset(&sample, 0, sizeof(sample));
  memset(&ina, 0, sizeof(ina));
  memset(&bme, 0, sizeof(bme));
  ina_ready = bme_ready = 0;
  sample.ina_status = sample.bme_status = -1;
  uint16_t manufacturer, device, config;
  (void)ina3221_init_desc(&ina, INA_ADDRESS, 3, 4, 7);
  for (unsigned i = 0; i < 3; ++i) ina.shunt[i] = 100U; /* R16/R17/R6, mOhm */
  /* Avoid upstream reset helper: it leaves cached RST set and initializes
     mask from CONFIG. Explicit words also avoid C bitfield-layout dependence. */
  ina.config.config_register = 0x7527U; /* all channels, 16 averages, 1.1ms, continuous */
  ina.mask.mask_register = INA3221_DEFAULT_MASK;
  if (!ina_word(0xFE, &manufacturer) && manufacturer == 0x5449U &&
      !ina_word(0xFF, &device) && device == 0x3220U &&
      !ina3221_sync(&ina) && !ina_word(0, &config) && config == 0x7527U) {
    ina_ready = 1;
    sample.ina_status = 0;
  }
  bme.intf = BME280_I2C_INTF;
  bme.intf_ptr = &hi2c3;
  bme.read = bme_read;
  bme.write = bme_write;
  bme.delay_us = delay_us;
  int8_t result = bme280_init(&bme); /* checks chip ID 0x60 and reads factory trim */
  struct bme280_settings settings = {0};
  settings.osr_p = settings.osr_t = settings.osr_h = BME280_OVERSAMPLING_1X;
  settings.filter = BME280_FILTER_COEFF_OFF;
  settings.standby_time = BME280_STANDBY_TIME_1000_MS;
  if (!result && (bme.calib_data.dig_t1 == 0U || bme.calib_data.dig_t1 == 65535U ||
                 bme.calib_data.dig_p1 == 0U || bme.calib_data.dig_p1 == 65535U)) result = -7;
  if (!result) result = bme280_set_sensor_settings(BME280_SEL_ALL_SETTINGS, &settings, &bme);
  if (!result) result = bme280_set_sensor_mode(BME280_POWERMODE_NORMAL, &bme);
  bme_ready = result == 0;
  sample.bme_status = result;
  HAL_Delay(120U); /* INA full scan: 3 * 2 * 1.1ms * 16 = 105.6ms */
  next_poll = HAL_GetTick();
  Sensors_Poll();
}

void Sensors_Poll(void)
{
  uint32_t now = HAL_GetTick();
  if ((int32_t)(now - next_poll) < 0) return;
  next_poll = now + 1000U;
  if (ina_ready) {
    uint16_t status;
    int result = ina_word(0x0F, &status);
    if (!result && (status & 1U)) {
      /* TI QFN: CH1 pins12/11=1V5, CH2 pins15/14=2V8, CH3 pins2/1=3V3. */
      static const ina3221_channel_t channels[3] = {INA3221_CHANNEL_3, INA3221_CHANNEL_2, INA3221_CHANNEL_1};
      SensorSample updated = sample;
      for (unsigned i = 0; i < 3 && !result; ++i) {
        float volts, milliamps;
        result = ina3221_get_bus_voltage(&ina, channels[i], &volts);
        if (!result) result = ina3221_get_shunt_value(&ina, channels[i], NULL, &milliamps);
        if (!result) {
          if (volts < 0.0f || volts > 26.0f) { result = -2; break; }
          updated.rail_mv[i] = (uint16_t)(volts * 1000.0f + 0.5f);
          float ua = milliamps * 1000.0f;
          updated.current_ua[i] = (int32_t)(ua + (ua < 0 ? -0.5f : 0.5f));
        }
      }
      if (!result) {
        memcpy(sample.rail_mv, updated.rail_mv, sizeof(sample.rail_mv));
        memcpy(sample.current_ua, updated.current_ua, sizeof(sample.current_ua));
        sample.rail_tick = HAL_GetTick();
        sample.valid |= SENSOR_RAILS_VALID;
      }
    }
    sample.ina_status = (int8_t)result;
    if (result) sample.valid &= (uint8_t)~SENSOR_RAILS_VALID;
  }
  if (bme_ready) {
    uint8_t status;
    int8_t result = bme280_get_regs(BME280_REG_STATUS, &status, 1, &bme);
    if (!result && !(status & (BME280_STATUS_MEAS_DONE | BME280_STATUS_IM_UPDATE))) {
      struct bme280_data data;
      result = bme280_get_sensor_data(BME280_ALL, &data, &bme);
      if (!result) {
        sample.temperature_centi_c = data.temperature;
        sample.humidity_centi_pct = (data.humidity * 100U + 512U) / 1024U;
        sample.pressure_pa = data.pressure / 100U + (data.pressure % 100U >= 50U);
        sample.env_tick = HAL_GetTick();
        sample.valid &= (uint8_t)~(SENSOR_TEMP_VALID | SENSOR_RH_VALID | SENSOR_PRESS_VALID);
        sample.out_of_range &= (uint8_t)~SENSOR_PRESS_VALID;
        /* Preserve compensated pressure outside the rated range, with *.
           Zero from the local Bosch extension means failed compensation. */
        if (data.pressure != 0U) {
          sample.valid |= SENSOR_PRESS_VALID;
          if (data.pressure < 3000000U || data.pressure > 11000000U)
            sample.out_of_range |= SENSOR_PRESS_VALID;
        }
        /* Existing temperature/humidity validity policy is unchanged. */
        if (data.temperature > -4000 && data.temperature < 8500) {
          sample.valid |= SENSOR_TEMP_VALID | SENSOR_RH_VALID;
        }
      }
    }
    sample.bme_status = result;
    if (result) sample.valid &= SENSOR_RAILS_VALID;
  }
}
void Sensors_Get(SensorSample *out)
{
  if (!out) return;
  *out = sample;
  uint32_t now = HAL_GetTick();
  if (now - out->rail_tick > SAMPLE_MAX_AGE_MS) out->valid &= (uint8_t)~SENSOR_RAILS_VALID;
  if (now - out->env_tick > SAMPLE_MAX_AGE_MS) out->valid &= SENSOR_RAILS_VALID;
}
