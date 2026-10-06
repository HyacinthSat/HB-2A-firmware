// Source: https://github.com/afiskon/stm32-si5351 (MIT)
// Adapted for this project toolchain/MCU.
#ifndef _SI5351_H_
#define _SI5351_H_

#include <stdint.h>

typedef enum {
 SI5351_PLL_A = 0,
 SI5351_PLL_B,
} si5351PLL_t;

typedef enum {
 SI5351_R_DIV_1 = 0,
 SI5351_R_DIV_2 = 1,
 SI5351_R_DIV_4 = 2,
 SI5351_R_DIV_8 = 3,
 SI5351_R_DIV_16 = 4,
 SI5351_R_DIV_32 = 5,
 SI5351_R_DIV_64 = 6,
 SI5351_R_DIV_128 = 7,
} si5351RDiv_t;

typedef enum {
 SI5351_DRIVE_STRENGTH_2MA = 0x00,
 SI5351_DRIVE_STRENGTH_4MA = 0x01,
 SI5351_DRIVE_STRENGTH_6MA = 0x02,
 SI5351_DRIVE_STRENGTH_8MA = 0x03,
} si5351DriveStrength_t;

typedef struct {
 int32_t mult;
 int32_t num;
 int32_t denom;
} si5351PLLConfig_t;

typedef struct {
 uint8_t allowIntegerMode;
 int32_t div;
 int32_t num;
 int32_t denom;
 si5351RDiv_t rdiv;
} si5351OutputConfig_t;

int si5351_Init(int32_t correction);
int si5351_SetupCLK0(int32_t Fclk, si5351DriveStrength_t driveStrength);
int si5351_SetupCLK2(int32_t Fclk, si5351DriveStrength_t driveStrength);
int si5351_EnableOutputs(uint8_t enabled);

void si5351_Calc(int32_t Fclk, si5351PLLConfig_t* pll_conf, si5351OutputConfig_t* out_conf);
void si5351_CalcIQ(int32_t Fclk, si5351PLLConfig_t* pll_conf, si5351OutputConfig_t* out_conf);

int si5351_SetupPLL(si5351PLL_t pll, si5351PLLConfig_t* conf);
int si5351_SetupOutput(uint8_t output, si5351PLL_t pllSource, si5351DriveStrength_t driveStength, si5351OutputConfig_t* conf, uint8_t phaseOffset);

#endif
