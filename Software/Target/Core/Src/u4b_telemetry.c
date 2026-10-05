/* Traquito WsprEncoded Basic Telemetry algorithm, translated to C.
 * Source reference: WsprMessageTelemetryBasic.h (GPL-compatible project). */
#include "u4b_telemetry.h"

static uint32_t quantize_half_up(uint32_t value, uint32_t low, uint32_t step)
{
  return (value <= low) ? 0U : (value - low + step / 2U) / step;
}

static uint8_t encode_base36(uint32_t value)
{
  return value < 10U ? (uint8_t)('0' + value) : (uint8_t)('A' + value - 10U);
}

void U4B_Grid56FromLatLonE7(int32_t latitude_e7, int32_t longitude_e7, char grid56[3])
{
  /* Traquito's integer routine works in 1e-4 degree units. */
  int64_t lat = (int64_t)(latitude_e7 / 1000) + 900000LL;
  int64_t lon = (int64_t)(longitude_e7 / 1000) + 1800000LL;
  uint32_t lon_remainder;
  uint32_t lat_remainder;
  uint32_t g5;
  uint32_t g6;

  if (grid56 == 0) return;
  if (lat < 0 || lat > 1800000LL || lon < 0 || lon > 3600000LL) {
    grid56[0] = 'A'; grid56[1] = 'A'; grid56[2] = 0; return;
  }

  /* In 1e-4 degree units, the two-cell remainder is 20,000 / 10,000;
     each Maidenhead subsquare letter is quantized with 834 / 417 units. */
  lon_remainder = (uint32_t)((lon % 200000LL) % 20000LL);
  lat_remainder = (uint32_t)((lat % 100000LL) % 10000LL);
  g5 = lon_remainder / 834U;
  g6 = lat_remainder / 417U;
  if (g5 > 23U) g5 = 23U;
  if (g6 > 23U) g6 = 23U;
  grid56[0] = (char)('A' + g5);
  grid56[1] = (char)('A' + g6);
  grid56[2] = 0;
}

static void u4b_build_basic_grid(const U4B_Sample *sample, const char id13[3], const char grid56[3], U4B_Message *message)
{
  static const uint8_t power_list[19] = { 0, 3, 7, 10, 13, 17, 20, 23, 27, 30,
                                          33, 37, 40, 43, 47, 50, 53, 57, 60 };
  uint32_t altitude_index;
  uint32_t grid5;
  uint32_t grid6;
  uint32_t value;
  uint32_t temp_index;
  uint32_t voltage_index;
  uint32_t speed_index;
  uint32_t gps_index;
  uint32_t power_index;
  uint32_t g1, g2, g3, g4;

  if (sample == 0 || id13 == 0 || grid56 == 0 || message == 0) return;
  grid5 = (uint32_t)(grid56[0] - 'A');
  grid6 = (uint32_t)(grid56[1] - 'A');
  altitude_index = quantize_half_up(sample->altitude_m, 0U, 20U);
  if (altitude_index > 1067U) altitude_index = 1067U;

  value = grid5;
  value = value * 24U + grid6;
  value = value * 1068U + altitude_index;
  message->call_value = value;
  message->callsign[0] = id13[0];
  message->callsign[1] = (char)encode_base36(value / (26U * 26U * 26U) % 36U);
  message->callsign[2] = id13[1];
  value %= 26U * 26U * 26U;
  g1 = value / (26U * 26U); value %= 26U * 26U;
  g2 = value / 26U; g3 = value % 26U;
  message->callsign[3] = (char)('A' + g1);
  message->callsign[4] = (char)('A' + g2);
  message->callsign[5] = (char)('A' + g3);
  message->callsign[6] = 0;

  temp_index = sample->temperature_c < -50 ? 0U : (sample->temperature_c > 39 ? 89U : (uint32_t)(sample->temperature_c + 50));
  voltage_index = quantize_half_up(sample->voltage_mv, 3000U, 50U);
  if (voltage_index > 39U) voltage_index = 39U;
  voltage_index = (voltage_index + 20U) % 40U;
  speed_index = (sample->speed_knots + 1U) / 2U;
  if (speed_index > 41U) speed_index = 41U;
  gps_index = sample->gps_valid ? 1U : 0U;

  /* This radix chain matches WsprMessageTelemetryBasic::EncodeGridPower. */
  value = temp_index;
  value = value * 40U + voltage_index;
  value = value * 42U + speed_index;
  value = value * 2U + gps_index;
  value = value * 2U + 1U;
  message->grid_value = value;
  message->big_number = (uint64_t)message->call_value * 615600ULL + value;

  power_index = value % 19U; value /= 19U;
  g4 = value % 10U; value /= 10U;
  g3 = value % 10U; value /= 10U;
  g2 = value % 18U; value /= 18U;
  g1 = value % 18U;
  message->grid4[0] = (char)('A' + g1);
  message->grid4[1] = (char)('A' + g2);
  message->grid4[2] = (char)('0' + g3);
  message->grid4[3] = (char)('0' + g4);
  message->grid4[4] = 0;
  message->power_dbm = power_list[power_index];
}

void U4B_BuildBasic(const U4B_Sample *sample, const char id13[3], U4B_Message *message)
{
  char grid56[3];
  if (sample == 0) return;
  U4B_Grid56FromLatLonE7(sample->latitude_e7, sample->longitude_e7, grid56);
  u4b_build_basic_grid(sample, id13, grid56, message);
}

void U4B_BuildBasicWithGrid56(const U4B_Sample *sample, const char id13[3], const char grid56[3], U4B_Message *message)
{
  u4b_build_basic_grid(sample, id13, grid56, message);
}
