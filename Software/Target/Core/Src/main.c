#include "main.h"

#include "app_main.h"
#include "gpio.h"
#include "usart.h"

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
  HAL_Init();
  SystemClock_Config();
  MX_GPIO_Init();
  board_release_resets();
  MX_UART4_Init();
  app_main();
  return 0;
}

void Error_Handler(void)
{
  __disable_irq();
  while (1) { }
}

