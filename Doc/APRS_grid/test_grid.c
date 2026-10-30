#include "aprs_1deg.h"
#include <assert.h>
#include <limits.h>
#include <stdio.h>

int main(int argc, char **argv)
{
  unsigned r, c, i;
  FILE *file;
  assert(argc == 2);
  file = fopen(argv[1], "rb");
  assert(file);
  for (r = 0; r < 180U; ++r) {
    for (c = 0; c < 360U; ++c) {
      int low = fgetc(file), high = fgetc(file);
      uint16_t expected;
      int32_t lat = (int32_t)r * 10000000L - 900000000L;
      int32_t lon = (int32_t)((int64_t)c * 10000000LL - 1800000000LL);
      assert(low != EOF && high != EOF);
      expected = (uint16_t)((unsigned)low | ((unsigned)high << 8U));
      /* Half-open grid boundaries, centers, and last e7 unit in every cell. */
      assert(aprs_grid_lookup_e7(lat, lon, 1) == expected);
      assert(aprs_grid_lookup_e7(lat + 5000000L, lon + 5000000L, 1) == expected);
      assert(aprs_grid_lookup_e7(lat + 9999999L, lon + 9999999L, 1) == expected);
    }
  }
  assert(fgetc(file) == EOF);
  fclose(file);
  for (r = 0; r < 180U; ++r) {
    int32_t lat = (int32_t)r * 10000000L - 900000000L;
    assert(aprs_grid_lookup_e7(lat, -1800000000L, 1) ==
           aprs_grid_lookup_e7(lat, 1800000000L, 1));
  }
  for (c = 0; c < 360U; ++c) {
    int32_t lon = (int32_t)((int64_t)c * 10000000LL - 1800000000LL);
    assert(aprs_grid_lookup_e7(900000000L, lon, 1) ==
           aprs_grid_lookup_e7(899999999L, lon, 1));
  }
  assert(aprs_grid_lookup_e7(0, 0, 0) == APRS_GRID_INVALID);
  assert(aprs_grid_lookup_e7(900000001L, 0, 1) == APRS_GRID_INVALID);
  assert(aprs_grid_lookup_e7(-900000001L, 0, 1) == APRS_GRID_INVALID);
  assert(aprs_grid_lookup_e7(0, 1800000001L, 1) == APRS_GRID_INVALID);
  assert(aprs_grid_lookup_e7(0, -1800000001L, 1) == APRS_GRID_INVALID);
  assert(aprs_grid_lookup_e7(INT32_MIN, INT32_MAX, 1) == APRS_GRID_INVALID);
  for (i = 0; i < 8U; ++i)
    assert(aprs_grid_single_frequency_hz((uint16_t)(1U << i)) == aprs_grid_frequencies_hz[i]);
  assert(aprs_grid_single_frequency_hz(0) == 0);
  assert(aprs_grid_single_frequency_hz(3) == 0);
  assert(aprs_grid_single_frequency_hz(APRS_GRID_UNKNOWN) == 0);
  assert(aprs_grid_single_frequency_hz(APRS_GRID_INVALID) == 0);
  printf("PASS: 194400 cell queries, date line, poles, invalid inputs, frequency masks\n");
  printf("C table bytes: %zu\n", sizeof(aprs_grid_frequencies_hz) +
         sizeof(aprs_grid_row_offsets) + sizeof(aprs_grid_palette) + sizeof(aprs_grid_runs));
  return 0;
}
