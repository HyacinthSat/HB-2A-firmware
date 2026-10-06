#include "main.h"

#include "app_main.h"
#include "gpio.h"
#include "usart.h"

#include <stdio.h>
#include <string.h>

#define BOARD_STAGE_MAGIC 0x43414D00UL

void board_set_stage(BoardStage stage)
{
  TAMP->BKP1R = BOARD_STAGE_MAGIC | (uint32_t)stage;
  __DSB();
}

static void log_reset_details(uint32_t flags, uint32_t previous_stage)
{
  static const char *const stages[] = {
    "idle", "camera-off", "camera-clock", "camera-rails", "camera-config",
    "dcmi-init", "capture", "image-store", "ssdv"
  };
  char line[160];
  uint32_t stage = previous_stage & 0xFFU;
  const char *name = "unknown";
  if ((previous_stage & 0xFFFFFF00UL) == BOARD_STAGE_MAGIC &&
      stage < sizeof(stages) / sizeof(stages[0])) name = stages[stage];
  (void)snprintf(line, sizeof(line),
                 "RESET BOR=%u PIN=%u SW=%u IWDG=%u WWDG=%u LPWR=%u OBL=%u previous=%s\r\n",
                 (unsigned)((flags & RCC_CSR_BORRSTF) != 0U),
                 (unsigned)((flags & RCC_CSR_PINRSTF) != 0U),
                 (unsigned)((flags & RCC_CSR_SFTRSTF) != 0U),
                 (unsigned)((flags & RCC_CSR_IWDGRSTF) != 0U),
                 (unsigned)((flags & RCC_CSR_WWDGRSTF) != 0U),
                 (unsigned)((flags & RCC_CSR_LPWRRSTF) != 0U),
                 (unsigned)((flags & RCC_CSR_OBLRSTF) != 0U), name);
  (void)HAL_UART_Transmit(&huart4, (uint8_t *)line, (uint16_t)strlen(line), 1000U);
}

static void board_release_resets(void)
{
  HAL_Delay(10U);
  HAL_GPIO_WritePin(FLASH_RST_GPIO_Port, FLASH_RST_Pin, GPIO_PIN_SET);
  HAL_GPIO_WritePin(GNSS_RST_GPIO_Port, GNSS_RST_Pin, GPIO_PIN_SET);
  HAL_GPIO_WritePin(RSY_GPIO_Port, RSY_Pin, GPIO_PIN_RESET);
  HAL_Delay(1U);
  HAL_GPIO_WritePin(RSY_GPIO_Port, RSY_Pin, GPIO_PIN_SET);
  HAL_Delay(5U);
}

void SystemClock_Config(void)
{
  RCC_OscInitTypeDef osc = {0};
  RCC_ClkInitTypeDef clk = {0};

  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK) {
    Error_Handler();
  }
  osc.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  osc.HSEState = RCC_HSE_ON;
  osc.PLL.PLLState = RCC_PLL_ON;
  osc.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  osc.PLL.PLLMBOOST = RCC_PLLMBOOST_DIV1;
  osc.PLL.PLLM = 1U;
  osc.PLL.PLLN = 20U;
  osc.PLL.PLLP = 2U;
  osc.PLL.PLLQ = 2U;
  osc.PLL.PLLR = 1U;
  osc.PLL.PLLRGE = RCC_PLLVCIRANGE_1;
  osc.PLL.PLLFRACN = 0U;
  if (HAL_RCC_OscConfig(&osc) != HAL_OK) Error_Handler();

  clk.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK |
                  RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2 |
                  RCC_CLOCKTYPE_PCLK3;
  clk.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  clk.AHBCLKDivider = RCC_SYSCLK_DIV1;
  clk.APB1CLKDivider = RCC_HCLK_DIV1;
  clk.APB2CLKDivider = RCC_HCLK_DIV1;
  clk.APB3CLKDivider = RCC_HCLK_DIV1;
  if (HAL_RCC_ClockConfig(&clk, FLASH_LATENCY_4) != HAL_OK) Error_Handler();
}

int main(void)
{
  uint32_t reset_flags = RCC->CSR;
  uint32_t previous_stage;
  char reset_line[64];

  HAL_Init();
  SystemClock_Config();
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_RCC_RTCAPB_CLK_ENABLE();
  HAL_PWR_EnableBkUpAccess();
  previous_stage = TAMP->BKP1R;
  board_set_stage(BOARD_STAGE_IDLE);
  MX_GPIO_Init();
  board_release_resets();
  MX_UART4_Init();
  (void)snprintf(reset_line, sizeof(reset_line),
                 "HB-2A reset CSR=0x%08lX\r\n",
                 (unsigned long)reset_flags);
  (void)HAL_UART_Transmit(&huart4, (uint8_t *)reset_line,
                          (uint16_t)strlen(reset_line), 1000U);
  __HAL_RCC_CLEAR_RESET_FLAGS();
  log_reset_details(reset_flags, previous_stage);
  {
    static const char build[] = "HB-2A build " __DATE__ " " __TIME__ "\r\n";
    (void)HAL_UART_Transmit(&huart4, (uint8_t *)build, sizeof(build) - 1U, 1000U);
  }
  app_main();
  return 0;
}

void Error_Handler(void)
{
  __disable_irq();
  while (1) { }
}
