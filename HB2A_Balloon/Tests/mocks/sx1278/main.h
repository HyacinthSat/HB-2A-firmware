#ifndef TEST_SX_MAIN_H
#define TEST_SX_MAIN_H
#include <stdint.h>
#include <stddef.h>
typedef struct { uint32_t BSRR, ODR, MODER, OTYPER; } GPIO_TypeDef;
typedef struct { uint32_t Pin, Mode, Pull, Speed; } GPIO_InitTypeDef;
typedef struct { uint32_t CR1, DIER, SR, PSC, ARR, EGR, CNT; } TIM_TypeDef;
typedef struct { uint32_t CFGR2; } RCC_TypeDef;
typedef struct { uint32_t EXTICR[4], IMR1, RTSR1, RPR1; } EXTI_TypeDef;
extern EXTI_TypeDef mock_exti;
#define EXTI (&mock_exti)
uint32_t NVIC_GetEnableIRQ(int);
#define __get_PRIMASK() 0U
typedef enum { HAL_OK, HAL_ERROR } HAL_StatusTypeDef;
typedef enum { GPIO_PIN_RESET, GPIO_PIN_SET } GPIO_PinState;
extern GPIO_TypeDef mock_a, mock_b, mock_c;
extern TIM_TypeDef mock_tim;
extern RCC_TypeDef mock_rcc;
#define GPIOA (&mock_a)
#define GPIOB (&mock_b)
#define GPIOC (&mock_c)
#define TIM2 (&mock_tim)
#define RCC (&mock_rcc)
#define DIO1_Pin (1U<<11)
#define DIO2_Pin (1U<<12)
#define DIO1_GPIO_Port GPIOA
#define DIO2_GPIO_Port GPIOA
#define RSY_Pin (1U<<15)
#define RSY_GPIO_Port GPIOA
#define SX1278_NSS_Pin (1U<<12)
#define SX1278_NSS_GPIO_Port GPIOB
#define FLASH_NSS_Pin (1U<<11)
#define FLASH_NSS_GPIO_Port GPIOB
#define GPIO_PIN_2 4U
#define GPIO_MODE_INPUT 0U
#define GPIO_MODE_OUTPUT_PP 1U
#define GPIO_MODE_IT_RISING 2U
#define GPIO_MODE_IT_FALLING 3U
#define GPIO_NOPULL 0U
#define GPIO_PULLDOWN 2U
#define GPIO_SPEED_FREQ_LOW 0U
#define EXTI11_IRQn 11
#define EXTI12_IRQn 12
#define EXTI13_IRQn 13
#define TIM2_IRQn 2
#define TIM_SR_UIF 1U
#define TIM_DIER_UIE 1U
#define TIM_CR1_CEN 1U
#define TIM_EGR_UG 1U
#define RCC_CFGR2_PPRE1 1U
#define __DMB() ((void)0)
#define __HAL_GPIO_EXTI_CLEAR_IT(x) ((void)(x))
void test_generate_swit(uint32_t pin);
#define __HAL_GPIO_EXTI_GENERATE_SWIT(x) test_generate_swit(x)
#define __HAL_RCC_TIM2_CLK_ENABLE() ((void)0)
void HAL_GPIO_EXTI_Rising_Callback(uint16_t pin);
void HAL_GPIO_EXTI_Falling_Callback(uint16_t pin);
void HAL_GPIO_WritePin(GPIO_TypeDef*, uint32_t, GPIO_PinState);
GPIO_PinState HAL_GPIO_ReadPin(GPIO_TypeDef*, uint32_t);
void HAL_GPIO_Init(GPIO_TypeDef*, const GPIO_InitTypeDef*);
void HAL_NVIC_ClearPendingIRQ(int);
void HAL_NVIC_DisableIRQ(int);
void HAL_NVIC_EnableIRQ(int);
void HAL_NVIC_SetPriority(int,unsigned,unsigned);
uint32_t HAL_GetTick(void);
uint32_t HAL_RCC_GetPCLK1Freq(void);
void HAL_Delay(uint32_t);
#endif
