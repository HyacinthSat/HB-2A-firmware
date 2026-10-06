#ifndef CAMERA_TEST_HAL_H
#define CAMERA_TEST_HAL_H
#include <stdint.h>
#include <stddef.h>
typedef enum { HAL_OK, HAL_ERROR, HAL_BUSY, HAL_TIMEOUT } HAL_StatusTypeDef;
typedef enum { GPIO_PIN_RESET, GPIO_PIN_SET } GPIO_PinState;
typedef struct { uint32_t Pin, Mode, Pull, Speed, Alternate; } GPIO_InitTypeDef;
typedef struct { uint32_t unused; } GPIO_TypeDef;
typedef struct { uint32_t CCER, CR1, PSC, ARR, CCR2, CCMR1, EGR; } TIM_TypeDef;
typedef struct { void *Instance; uint32_t State, ErrorCode; } DMA_HandleTypeDef;
typedef struct { void *Instance; uint32_t State, ErrorCode; DMA_HandleTypeDef *DMA_Handle; } DCMI_HandleTypeDef;
typedef struct { uint32_t unused; } UART_HandleTypeDef;
typedef struct { uint32_t unused; } I2C_HandleTypeDef;
extern TIM_TypeDef mock_tim;
extern GPIO_TypeDef mock_gpio;
extern uint32_t mock_remaining;
#define TIM3 (&mock_tim)
#define GPIOA (&mock_gpio)
#define GPIOB (&mock_gpio)
#define GPIOC (&mock_gpio)
#define GPIO_PIN_0 1U
#define GPIO_PIN_1 2U
#define GPIO_PIN_2 4U
#define GPIO_PIN_3 8U
#define GPIO_PIN_4 16U
#define GPIO_PIN_5 32U
#define GPIO_PIN_6 64U
#define GPIO_PIN_7 128U
#define GPIO_PIN_8 256U
#define GPIO_PIN_9 512U
#define GPIO_PIN_10 1024U
#define GPIO_PIN_11 2048U
#define GPIO_PIN_12 4096U
#define GPIO_PIN_13 8192U
#define GPIO_PIN_14 16384U
#define GPIO_PIN_15 32768U
#define GPIO_MODE_OUTPUT_PP 1U
#define GPIO_MODE_AF_PP 2U
#define GPIO_NOPULL 0U
#define GPIO_SPEED_FREQ_VERY_HIGH 3U
#define GPIO_AF2_TIM3 2U
#define TIM_CCER_CC2E 1U
#define TIM_CCMR1_OC2M_Pos 4U
#define TIM_CCMR1_OC2PE 1U
#define TIM_EGR_UG 1U
#define TIM_CR1_ARPE 2U
#define TIM_CR1_CEN 1U
#define HAL_DMA_STATE_READY 1U
#define HAL_DMA_STATE_BUSY 2U
#define HAL_DCMI_STATE_READY 1U
#define HAL_DCMI_STATE_BUSY 2U
#define DCMI_MODE_SNAPSHOT 1U
#define DCMI_IT_FRAME 1U
#define DCMI_IT_OVR 2U
#define DCMI_IT_ERR 4U
#define DCMI_FLAG_FRAMERI 1U
#define DCMI_FLAG_OVRRI 2U
#define DCMI_FLAG_ERRRI 4U
#define __HAL_RCC_TIM3_CLK_ENABLE() ((void)0)
#define __HAL_RCC_GPIOC_CLK_ENABLE() ((void)0)
#define __HAL_DCMI_DISABLE_IT(h, flags) ((void)0)
#define __HAL_DCMI_ENABLE_IT(h, flags) ((void)0)
#define __HAL_DCMI_CLEAR_FLAG(h, flags) ((void)0)
#define __HAL_DMA_GET_COUNTER(h) mock_remaining
#define __DSB() ((void)0)
void HAL_GPIO_Init(GPIO_TypeDef *, GPIO_InitTypeDef *);
void HAL_GPIO_WritePin(GPIO_TypeDef *, uint32_t, GPIO_PinState);
GPIO_PinState HAL_GPIO_ReadPin(GPIO_TypeDef *, uint32_t);
HAL_StatusTypeDef HAL_UART_Transmit(UART_HandleTypeDef *, uint8_t *, uint16_t, uint32_t);
HAL_StatusTypeDef HAL_I2C_DeInit(I2C_HandleTypeDef *);
HAL_StatusTypeDef HAL_DMA_Abort(DMA_HandleTypeDef *);
HAL_StatusTypeDef HAL_DCMI_Stop(DCMI_HandleTypeDef *);
HAL_StatusTypeDef HAL_DCMI_Start_DMA(DCMI_HandleTypeDef *, uint32_t, uint32_t, uint32_t);
void HAL_Delay(uint32_t);
uint32_t HAL_GetTick(void);
#endif
