#include "ov5640.h"
#include <assert.h>
#include <stdio.h>
#include <string.h>

static uint8_t regs[65536];
static unsigned writes;
static int failed_write = -1;
static int corrupted_read = -1;

static int32_t bus_init(void) { return 0; }
static int32_t tick(void) { static int32_t now; return now++; }
static int32_t read_reg(uint16_t address, uint16_t reg, uint8_t *data, uint16_t length)
{
  (void)address;
  assert(length == 1U);
  *data = regs[reg];
  if (reg == corrupted_read) *data ^= 1U;
  return 0;
}
static int32_t write_reg(uint16_t address, uint16_t reg, uint8_t *data, uint16_t length)
{
  (void)address;
  assert(length == 1U);
  ++writes;
  if (reg == failed_write) return -1;
  regs[reg] = *data;
  return 0;
}
static unsigned reg16(unsigned reg)
{
  return ((unsigned)regs[reg] << 8U) | regs[reg + 1U];
}

int main(void)
{
  OV5640_Object_t sensor = {0};
  OV5640_IO_t io = {0};
  static const uint16_t sizes[][2] = {
    {128, 96}, {256, 192}, {352, 288}, {640, 480}, {752, 496},
    {1024, 768}, {1280, 960}, {1296, 976}, {1600, 1200},
    {1920, 1088}, {2592, 1936}, {480, 640}, {16, 16}
  };
  static const uint16_t invalid[][2] = {
    {0, 480}, {640, 0}, {2594, 1936}, {2592, 1946}, {641, 480}, {640, 481}
  };
  io.Init = bus_init;
  io.ReadReg = read_reg;
  io.WriteReg = write_reg;
  io.GetTick = tick;
  assert(OV5640_RegisterBusIO(&sensor, &io) == OV5640_OK);
  assert(OV5640_ResolutionForSize(640, 480) == OV5640_R640x480);
  assert(OV5640_ResolutionForSize(256, 192) == OV5640_R256x192);
  assert(OV5640_ResolutionForSize(1280, 960) == OV5640_ERROR);
  assert(OV5640_ResolutionForSize(0, 480) == OV5640_ERROR);
  assert(OV5640_SetOutputSize(&sensor, 640, 480) == OV5640_ERROR);
  assert(OV5640_Init(&sensor, OV5640_R640x480, OV5640_JPEG) == OV5640_OK);
  /* Existing small-image register state must not change beyond output size. */
  {
    static uint8_t original[sizeof(regs)];
    memcpy(original, regs, sizeof(regs));
    assert(OV5640_SetOutputSize(&sensor, 256, 192) == OV5640_OK);
    for (unsigned i = 0; i < sizeof(regs); ++i) {
      if (i < 0x3808U || i > 0x380BU) assert(original[i] == regs[i]);
    }
  }
  for (unsigned i = 0; i < sizeof(sizes) / sizeof(sizes[0]); ++i) {
    uint16_t w = sizes[i][0], h = sizes[i][1];
    unsigned full = w > 1280U || h > 960U;
    uint8_t flip = regs[0x3820] & 6U;
    uint8_t format = regs[0x3821] & 0x26U;
    uint8_t pll = regs[0x3036];
    assert(OV5640_SetOutputSize(&sensor, w, h) == OV5640_OK);
    assert(reg16(0x3808) == w && reg16(0x380A) == h);
    assert(reg16(0x380C) > w && reg16(0x380E) > h);
    assert(regs[0x3814] == (full ? 0x11U : 0x31U));
    assert(regs[0x3815] == regs[0x3814]);
    assert((regs[0x3820] & 6U) == flip && (regs[0x3821] & 0x26U) == format);
    assert(regs[0x3036] == pll && (regs[0x5001] & 0x20U));
    assert(regs[0x3008] == 0x02U);
  }
  for (unsigned i = 0; i < sizeof(invalid) / sizeof(invalid[0]); ++i) {
    unsigned before = writes;
    assert(OV5640_SetOutputSize(&sensor, invalid[i][0], invalid[i][1]) == OV5640_ERROR);
    assert(writes == before);
  }
  assert(OV5640_SetOutputSize(NULL, 640, 480) == OV5640_ERROR);
  failed_write = 0x3809;
  assert(OV5640_SetOutputSize(&sensor, 1600, 1200) == OV5640_ERROR);
  assert(regs[0x3008] & 0x40U);
  failed_write = -1;
  regs[0x3008] = 0x02U;
  corrupted_read = 0x380A;
  assert(OV5640_SetOutputSize(&sensor, 752, 496) == OV5640_ERROR);
  assert(regs[0x3008] & 0x40U);
  puts("OV5640 arbitrary size tests passed");
  return 0;
}
