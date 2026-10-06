// Source: https://github.com/afiskon/stm32-si5351 (MIT)
// Adapted for STM32U5 / hi2c3 in this project.

#include "main.h"
#include "i2c.h"
#include "si5351.h"

#define SI5351_ADDRESS 0x60
#define I2C_HANDLE hi2c3
extern I2C_HandleTypeDef I2C_HANDLE;

static int si5351_writeBulk(uint8_t baseaddr, int32_t P1, int32_t P2, int32_t P3, uint8_t divBy4, si5351RDiv_t rdiv);
static int si5351_write(uint8_t reg, uint8_t value);

enum {
 SI5351_REGISTER_3_OUTPUT_ENABLE_CONTROL = 3,
 SI5351_REGISTER_16_CLK0_CONTROL = 16,
 SI5351_REGISTER_17_CLK1_CONTROL = 17,
 SI5351_REGISTER_18_CLK2_CONTROL = 18,
 SI5351_REGISTER_19_CLK3_CONTROL = 19,
 SI5351_REGISTER_20_CLK4_CONTROL = 20,
 SI5351_REGISTER_21_CLK5_CONTROL = 21,
 SI5351_REGISTER_22_CLK6_CONTROL = 22,
 SI5351_REGISTER_23_CLK7_CONTROL = 23,
 SI5351_REGISTER_42_MULTISYNTH0_PARAMETERS_1 = 42,
 SI5351_REGISTER_50_MULTISYNTH1_PARAMETERS_1 = 50,
 SI5351_REGISTER_58_MULTISYNTH2_PARAMETERS_1 = 58,
 SI5351_REGISTER_165_CLK0_INITIAL_PHASE_OFFSET = 165,
 SI5351_REGISTER_166_CLK1_INITIAL_PHASE_OFFSET = 166,
 SI5351_REGISTER_167_CLK2_INITIAL_PHASE_OFFSET = 167,
 SI5351_REGISTER_177_PLL_RESET = 177,
 SI5351_REGISTER_183_CRYSTAL_INTERNAL_LOAD_CAPACITANCE = 183
};

typedef enum {
 SI5351_CRYSTAL_LOAD_6PF = (1 << 6),
 SI5351_CRYSTAL_LOAD_8PF = (2 << 6),
 SI5351_CRYSTAL_LOAD_10PF = (3 << 6)
} si5351CrystalLoad_t;

static int32_t si5351Correction;

int si5351_Init(int32_t correction) {
 si5351Correction = correction;

 if (si5351_EnableOutputs(0U)) return -1;
 for (uint8_t reg = 16U; reg <= 23U; ++reg)
  if (si5351_write(reg, 0x80U)) return -1;

 si5351CrystalLoad_t crystalLoad = SI5351_CRYSTAL_LOAD_10PF;
 return si5351_write(SI5351_REGISTER_183_CRYSTAL_INTERNAL_LOAD_CAPACITANCE, crystalLoad);
}

int si5351_SetupPLL(si5351PLL_t pll, si5351PLLConfig_t* conf) {
 int32_t P1, P2, P3;
 int32_t mult = conf->mult;
 int32_t num = conf->num;
 int32_t denom = conf->denom;

 P1 = 128 * mult + (128 * num) / denom - 512;
 P2 = (128 * num) % denom;
 P3 = denom;

 uint8_t baseaddr = (pll == SI5351_PLL_A ? 26 : 34);
 if (si5351_writeBulk(baseaddr, P1, P2, P3, 0, 0)) return -1;
 return si5351_write(SI5351_REGISTER_177_PLL_RESET, (1 << 7) | (1 << 5));
}

int si5351_SetupOutput(uint8_t output, si5351PLL_t pllSource, si5351DriveStrength_t driveStrength, si5351OutputConfig_t* conf, uint8_t phaseOffset) {
 int32_t div = conf->div;
 int32_t num = conf->num;
 int32_t denom = conf->denom;
 uint8_t divBy4 = 0;
 int32_t P1, P2, P3;

 if (output > 2) {
  return 1;
 }

 if ((!conf->allowIntegerMode) && ((div < 8) || ((div == 8) && (num == 0)))) {
  return 2;
 }

 if (div == 4) {
  P1 = 0;
  P2 = 0;
  P3 = 1;
  divBy4 = 0x3;
 } else {
  P1 = 128 * div + ((128 * num) / denom) - 512;
  P2 = (128 * num) % denom;
  P3 = denom;
 }

 uint8_t baseaddr = 0;
 uint8_t phaseOffsetRegister = 0;
 uint8_t clkControlRegister = 0;
 switch (output) {
  case 0:
   baseaddr = SI5351_REGISTER_42_MULTISYNTH0_PARAMETERS_1;
   phaseOffsetRegister = SI5351_REGISTER_165_CLK0_INITIAL_PHASE_OFFSET;
   clkControlRegister = SI5351_REGISTER_16_CLK0_CONTROL;
   break;
  case 1:
   baseaddr = SI5351_REGISTER_50_MULTISYNTH1_PARAMETERS_1;
   phaseOffsetRegister = SI5351_REGISTER_166_CLK1_INITIAL_PHASE_OFFSET;
   clkControlRegister = SI5351_REGISTER_17_CLK1_CONTROL;
   break;
  case 2:
   baseaddr = SI5351_REGISTER_58_MULTISYNTH2_PARAMETERS_1;
   phaseOffsetRegister = SI5351_REGISTER_167_CLK2_INITIAL_PHASE_OFFSET;
   clkControlRegister = SI5351_REGISTER_18_CLK2_CONTROL;
   break;
 }

 uint8_t clkControl = 0x0C | driveStrength;
 if (pllSource == SI5351_PLL_B) {
  clkControl |= (1 << 5);
 }

 if ((conf->allowIntegerMode) && ((num == 0) || (div == 4))) {
  clkControl |= (1 << 6);
 }

 if (si5351_write(clkControlRegister, clkControl) ||
     si5351_writeBulk(baseaddr, P1, P2, P3, divBy4, conf->rdiv)) return -1;
 return si5351_write(phaseOffsetRegister, (phaseOffset & 0x7F));
}

void si5351_Calc(int32_t Fclk, si5351PLLConfig_t* pll_conf, si5351OutputConfig_t* out_conf) {
 if (Fclk < 8000) Fclk = 8000;
 else if (Fclk > 160000000) Fclk = 160000000;

 out_conf->allowIntegerMode = 1;

 if (Fclk < 1000000) {
  Fclk *= 64;
  out_conf->rdiv = SI5351_R_DIV_64;
 } else {
  out_conf->rdiv = SI5351_R_DIV_1;
 }

 Fclk = Fclk - (int32_t)((((double)Fclk) / 100000000.0) * ((double)si5351Correction));

 const int32_t Fxtal = 25000000;
 int32_t a, b, c, x, y, z, t;

 if (Fclk < 81000000) {
  a = 36;
  b = 0;
  c = 1;
  int32_t Fpll = 900000000;
  x = Fpll / Fclk;
  t = (Fclk >> 20) + 1;
  y = (Fpll % Fclk) / t;
  z = Fclk / t;
 } else {
  if (Fclk >= 150000000) {
   x = 4;
  } else if (Fclk >= 100000000) {
   x = 6;
  } else {
   x = 8;
  }
  y = 0;
  z = 1;

  int32_t numerator = x * Fclk;
  a = numerator / Fxtal;
  t = (Fxtal >> 20) + 1;
  b = (numerator % Fxtal) / t;
  c = Fxtal / t;
 }

 pll_conf->mult = a;
 pll_conf->num = b;
 pll_conf->denom = c;
 out_conf->div = x;
 out_conf->num = y;
 out_conf->denom = z;
}

void si5351_CalcIQ(int32_t Fclk, si5351PLLConfig_t* pll_conf, si5351OutputConfig_t* out_conf) {
 const int32_t Fxtal = 25000000;
 int32_t Fpll;

 if (Fclk < 1400000) Fclk = 1400000;
 else if (Fclk > 100000000) Fclk = 100000000;

 Fclk = Fclk - ((Fclk / 1000000) * si5351Correction) / 100;
 out_conf->allowIntegerMode = 0;
 out_conf->rdiv = 0;

 if (Fclk < 4900000) {
  out_conf->div = 127;
 } else if (Fclk < 8000000) {
  out_conf->div = 625000000 / Fclk;
 } else {
  out_conf->div = 900000000 / Fclk;
 }
 out_conf->num = 0;
 out_conf->denom = 1;

 Fpll = Fclk * out_conf->div;
 pll_conf->mult = Fpll / Fxtal;
 pll_conf->num = (Fpll % Fxtal) / 24;
 pll_conf->denom = Fxtal / 24;
}

int si5351_SetupCLK0(int32_t Fclk, si5351DriveStrength_t driveStrength) {
 si5351PLLConfig_t pll_conf;
 si5351OutputConfig_t out_conf;

 si5351_Calc(Fclk, &pll_conf, &out_conf);
 if (si5351_SetupPLL(SI5351_PLL_A, &pll_conf)) return -1;
 return si5351_SetupOutput(0, SI5351_PLL_A, driveStrength, &out_conf, 0);
}

int si5351_SetupCLK2(int32_t Fclk, si5351DriveStrength_t driveStrength) {
 si5351PLLConfig_t pll_conf;
 si5351OutputConfig_t out_conf;

 si5351_Calc(Fclk, &pll_conf, &out_conf);
 if (si5351_SetupPLL(SI5351_PLL_B, &pll_conf)) return -1;
 return si5351_SetupOutput(2, SI5351_PLL_B, driveStrength, &out_conf, 0);
}

int si5351_EnableOutputs(uint8_t enabled) {
 uint8_t actual;
 uint8_t expected = (uint8_t)~enabled;
 if (si5351_write(SI5351_REGISTER_3_OUTPUT_ENABLE_CONTROL, expected)) return -1;
 if (HAL_I2C_Mem_Read(&I2C_HANDLE, SI5351_ADDRESS << 1, 3U,
                      I2C_MEMADD_SIZE_8BIT, &actual, 1U, 20U) != HAL_OK) return -1;
 return actual == expected ? 0 : -1;
}

static int si5351_write(uint8_t reg, uint8_t value) {
 return HAL_I2C_Mem_Write(&I2C_HANDLE,
                         (uint8_t)(SI5351_ADDRESS << 1),
                         reg,
                         I2C_MEMADD_SIZE_8BIT,
                         &value,
                         1,
                         20U) == HAL_OK ? 0 : -1;
}

static int si5351_writeBulk(uint8_t baseaddr, int32_t P1, int32_t P2, int32_t P3, uint8_t divBy4, si5351RDiv_t rdiv) {
 uint8_t bytes[8] = {
  (uint8_t)(P3 >> 8), (uint8_t)P3,
  (uint8_t)(((P1 >> 16) & 3) | ((divBy4 & 3) << 2) | ((rdiv & 7) << 4)),
  (uint8_t)(P1 >> 8), (uint8_t)P1,
  (uint8_t)(((P3 >> 12) & 0xF0) | ((P2 >> 16) & 15)),
  (uint8_t)(P2 >> 8), (uint8_t)P2
 };
 return HAL_I2C_Mem_Write(&I2C_HANDLE, SI5351_ADDRESS << 1, baseaddr,
                          I2C_MEMADD_SIZE_8BIT, bytes, sizeof(bytes), 20U) == HAL_OK ? 0 : -1;
}
