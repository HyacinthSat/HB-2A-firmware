# Sensor libraries

Upstream source and licenses are vendored locally. No Arduino,
ESP-IDF runtime, RTOS, heap allocator, or online dependency is required to build.

| Component | Upstream | Pinned commit | License |
|---|---|---|---|
| BME280 | https://github.com/boschsensortec/BME280_SensorAPI | `c90d419492e26dd95586598a794e65eb2760753a` | BSD-3-Clause, `BME280/LICENSE` |
| INA3221 | https://github.com/UncleRus/esp-idf-lib/tree/90374c67c5c6411553b35354e09ba75d917dddd3/components/ina3221 | `90374c67c5c6411553b35354e09ba75d917dddd3` | MIT, `INA3221/LICENSE` |

Retrieved 2026-10-04. Original upstream SHA256 values are recorded in
`SHA256SUMS.txt`; local build file hashes are in `SHA256SUMS.local.txt`.

Local change to `BME280/bme280.c`: `BME280_PRESSURE_UNCLAMPED` optionally bypasses
the 64-bit pressure path's 300..1100 hPa clamp. Its compensation formula is
unchanged. Division failure/nonpositive/uint32 overflow produces zero and is
reported as invalid by the adapter. Positive out-of-range pressure is preserved
and marked with `*` in APRS. Other Bosch files and the INA3221 source are unchanged.

Bosch uses its official calibration and 64-bit integer compensation path.
`BME280_64BIT_ENABLE` must be defined consistently for the driver and callers.
Temperature units are 0.01 C, pressure 0.01 Pa, humidity 1/1024 percent.

The INA3221 `port/` directory is a minimal local compatibility interface for the
upstream C driver. Only I2C3 is supported, mapped to STM32 HAL in `sensors.c`.
The mutex macros are intentionally no-ops: every caller runs in the single
foreground context, never in an interrupt. Future RTOS integration must replace
these with a real shared I2C bus lock, including SI5351 and Bosch accesses.

The application does not use the upstream `ina3221_reset()` helper: it leaves
the cached RST bit set and initializes mask from the configuration default.
Instead it probes manufacturer/device IDs and writes explicit configuration
and mask words, then verifies the configuration. All channel arguments are
fixed, valid enums; upstream public helpers do not consistently bounds-check.
No compiler-dependent configuration bitfields are set by this application.

HAL transactions have a 10 ms transfer timeout. Sensor callbacks do not log,
allocate, or change clocks. Bosch initialization delays round up to HAL ticks.
